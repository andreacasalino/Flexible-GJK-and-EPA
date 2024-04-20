/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/shape/PointCloud.h>
#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>
#include <Flexible-GJK-and-EPA/shape/Sphere.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

#include <Utils.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>

namespace flx::utils {
namespace {
float get_sample() {
  return -1.f + 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}
} // namespace

Points make_random_cloud(const std::size_t samples) {
  std::vector<Vector3d> result;
  result.reserve(samples);
  for (std::size_t k = 0; k < samples; ++k) {
    result.emplace_back(get_sample(), get_sample(), get_sample());
  }
  return std::make_shared<std::vector<Vector3d>>(std::move(result));
}

hull::Coordinate to_coordinate(const Vector3d &subject) {
  return hull::Coordinate{subject.x(), subject.y(), subject.z()};
}

float dot_product(const Vector3d &subject, const hull::Coordinate &direction) {
  float result = subject.x() * direction.x;
  result += subject.y() * direction.y;
  result += subject.z() * direction.z;
  return result;
};

LogRootFolder::LogRootFolder() {
  if (std::filesystem::exists(root)) {
    std::filesystem::remove_all(root);
  }
  std::filesystem::create_directories(root);
}

namespace {
void to_json(nlohmann::json &recipient, const Vector3d &point) {
  recipient = nlohmann::json::array();
  recipient.emplace_back() = point.x();
  recipient.emplace_back() = point.y();
  recipient.emplace_back() = point.z();
}

std::vector<float> linspace(float min, float max, std::size_t N) {
  float delta = (max - min) / static_cast<float>(N);
  std::vector<float> res;
  res.push_back(min);
  float val = min + delta;
  for (std::size_t k = 0; k < N; ++k, val += delta) {
    res.push_back(val);
  }
  return res;
}

static constexpr float PI = 3.141592f;
static constexpr float PI_HALF = 0.5f * PI;
static constexpr float PI_DOUBLE = 2.f * PI;

void to_json(nlohmann::json &recipient, const hull::Coordinate &point) {
  recipient = nlohmann::json::array();
  recipient.emplace_back() = point.x;
  recipient.emplace_back() = point.y;
  recipient.emplace_back() = point.z;
}

struct Converter {
  static std::vector<Vector3d> convert(const shape::ConvexShape &shape) {
    std::vector<Vector3d> res;
    if (auto *ptr = dynamic_cast<const shape::TransformDecorator *>(&shape);
        ptr) {
      res = convert_(*ptr);
    } else if (auto *ptr = dynamic_cast<const shape::RoundDecorator *>(&shape);
               ptr) {
      res = convert_(*ptr);
    } else if (auto *ptr = dynamic_cast<const shape::Sphere *>(&shape); ptr) {
      res = convert_(*ptr);
    } else if (auto *ptr = dynamic_cast<const PointCloud *>(&shape); ptr) {
      res = convert_(*ptr);
    }
    return res;
  }

private:
  static std::vector<Vector3d>
  convert_(const shape::TransformDecorator &shape) {
    std::vector<Vector3d> res;
    for (const auto &point : convert(shape.getShape())) {
      hull::Coordinate trsf;
      trsf.x = point.x();
      trsf.y = point.y();
      trsf.z = point.z();
      shape.getTransformation().transform(trsf);
      res.emplace_back(trsf.x, trsf.y, trsf.z);
    }
    return res;
  }

  static std::vector<Vector3d> convert_(const shape::RoundDecorator &shape) {
    std::vector<Vector3d> res;
    for (const auto &point : convert(shape.getShape())) {
      auto ball = convert_(shape.getRay(), point);
      res.insert(res.end(), ball.begin(), ball.end());
    }
    return res;
  }

  static std::vector<Vector3d> convert_(float ray, const Vector3d &center) {
    std::vector<Vector3d> res;
    auto thetas = linspace(-PI_HALF, PI_HALF, 10);
    auto phis = linspace(0, PI_DOUBLE, 10);
    std::for_each(thetas.begin() + 1, thetas.end() - 1, [&](float theta) {
      std::for_each(phis.begin() + 1, phis.end(), [&](float phi) {
        float theta_cos = cosf(theta);
        res.emplace_back(center.x() + ray * cosf(phi) * theta_cos,
                         center.y() + ray * sinf(phi) * theta_cos,
                         center.z() + ray * sinf(theta));
      });
    });
    res.emplace_back(center.x(), center.y(), center.z() + ray);
    res.emplace_back(center.x(), center.y(), center.z() - ray);
    return res;
  }

  static std::vector<Vector3d> convert_(const shape::Sphere &shape) {
    const auto &center = shape.getCenter();
    return convert_(shape.getRay(), Vector3d{center.x, center.y, center.z});
  }

  using PointCloud = shape::PointCloud<std::vector<Vector3d>::const_iterator>;
  static std::vector<Vector3d> convert_(const PointCloud &shape) {
    return std::vector<Vector3d>{shape.begin(), shape.end()};
  }
};

void to_json(nlohmann::json &recipient, const shape::ConvexShape &shape) {
  recipient["tag"] = "shape";
  auto &res = recipient["obj"];
  res = nlohmann::json::array();
  for (const auto &point : Converter::convert(shape)) {
    to_json(res.emplace_back(), point);
  }
}
} // namespace

ShapesLog::ShapesLog(const std::string &title)
    : path{LogRootFolder::get().root / "results"} {
  std::filesystem::create_directories(path);
  path /= title;
  content = nlohmann::json::array();
}

ShapesLog::~ShapesLog() {
  std::ofstream{path} << content.dump(1);
  std::cout << "Visualize results by running '" << PYTHON_CMD
            << " --specificResult " << path.filename() << '\'' << std::endl;
}

void ShapesLog::add(const shape::ConvexShape &shape) {
  to_json(content.emplace_back(), shape);
}

namespace {
void to_json(nlohmann::json &recipient, const CoordinatePair &coordinates,
             bool collision_present) {
  recipient["tag"] = "coordinates";
  auto &res = recipient["obj"];
  to_json(res["pointA"], coordinates.point_in_shape_a);
  to_json(res["pointB"], coordinates.point_in_shape_b);
}
} // namespace

void ShapesLog::add(const CoordinatePair &coordinates, bool collision_present) {
  to_json(content.emplace_back(), coordinates, collision_present);
}

void ShapesLog::logSample(const std::string &title,
                          const shape::ConvexShape &shapeA,
                          const shape::ConvexShape &shapeB,
                          const QueryResult &result) {
  ShapesLog tmp{title};
  tmp.add(shapeA);
  tmp.add(shapeB);
  tmp.add(result.result, !result.is_closest_pair_or_penetration_info);
}

#ifdef GJK_EPA_DIAGNOSTIC

GjkEpaLogger::GjkEpaLogger(const std::string &title)
    : log_folder{LogRootFolder::get().root / "gjk-epa" / title} {}

void GjkEpaLogger::onEvent(Event ev) {
  switch (ev) {
  case Event::InitialGjkStarted:
    log_path_current = log_folder / "InitialGjk";
    break;
  case Event::FinalGjkStarted:
    log_path_current = log_folder / "FinalGjk";
    break;
  case Event::EpaStarted:
    log_path_current = log_folder / "Epa";
    break;
  }
  std::filesystem::create_directories(log_path_current);
  iter = 0;
}

namespace {
void to_json(nlohmann::json &recipient, const MinkowskiDiffCoordinate &point) {
  to_json(recipient["vertex_in_shape_a"], point.vertex_in_shape_a);
  to_json(recipient["vertex_in_shape_b"], point.vertex_in_shape_b);
  to_json(recipient["vertex_in_Minkowski_diff"],
          point.vertex_in_Minkowski_diff);
}

void to_json(nlohmann::json &recipient,
             const GjkIteration::ClosestToRegionInfo &info) {
  to_json(recipient["point"], info.point);
  auto &region = recipient["region"];
  switch (info.region) {
  case ClosestRegionToOrigin::vertex_A:
    region = "vertex_A";
    break;
  case ClosestRegionToOrigin::edge_AB:
    region = "edge_AB";
    break;
  case ClosestRegionToOrigin::edge_AC:
    region = "edge_AC";
    break;
  case ClosestRegionToOrigin::face_ABC:
    region = "face_ABC";
    break;
  }
}

void to_json(nlohmann::json &recipient,
             const GjkIteration::TethraedronFacetInfo &info) {
  recipient["isOriginVisible"] = info.isOriginVisible;
  to_json(recipient["Normal"], info.normal);
  if (info.closest) {
    to_json(recipient["closest"], info.closest.value());
  } else {
    recipient["closest"] = nullptr;
  }
}

std::filesystem::path make_res_fldr(const std::filesystem::path &parent,
                                    std::size_t iter) {
  std::filesystem::path res = parent;
  res /= "Step-" + std::to_string(iter);
  return res;
}
} // namespace

void GjkEpaLogger::onUpdate(const GjkIteration &info) {
  nlohmann::json res;
  res["collision"] = info.plex.collision;
  to_json(res["direction"], info.plex.search_direction);
  auto &vertices = res["vertices"];
  vertices = nlohmann::json::array();
  for (std::size_t k = 0; k < info.plex.size + 1; ++k) {
    to_json(vertices.emplace_back(), *info.plex.vertices[k]);
  }
  struct Visitor {
    nlohmann::json &recipient;

    void operator()(const GjkIteration::ClosestToRegionInfo &subject) const {
      to_json(recipient, subject);
    }

    void operator()(const std::array<GjkIteration::TethraedronFacetInfo, 3>
                        &subject) const {
      recipient = nlohmann::json::array();
      for (const auto &subject_k : subject) {
        to_json(recipient.emplace_back(), subject_k);
      }
    }
  };
  Visitor visitor{res["info"]};
  std::visit(visitor, info.info);
  std::ofstream{make_res_fldr(log_path_current, iter++)} << res.dump(1);
}

void GjkEpaLogger::onUpdate(const EpaIteration &info) {
  nlohmann::json res;

  const auto &points = info.hull.getVertices();
  auto facet_to_json = [&points](nlohmann::json &recipient,
                                 const hull::Facet &subject) {
    to_json(recipient["vertexA"],
            points[subject.vertexA].vertex_in_Minkowski_diff);
    to_json(recipient["vertexB"],
            points[subject.vertexB].vertex_in_Minkowski_diff);
    to_json(recipient["vertexC"],
            points[subject.vertexC].vertex_in_Minkowski_diff);
    to_json(recipient["Normal"], subject.normal);
  };

  res = nlohmann::json::array();
  for (const auto &info : info.hull.getFacets()) {
    auto &added = res.emplace_back();
    added["distance_to_origin"] = info.distance_to_origin;
    facet_to_json(added, *info.facet);
  }
  std::ofstream{make_res_fldr(log_path_current, iter++)} << res.dump(1);
}

void GjkEpaLogger::addResult(const shape::ConvexShape &shapeA,
                             const shape::ConvexShape &shapeB,
                             const CoordinatePair &coordinates,
                             bool collision_present) {
  std::filesystem::create_directories(log_folder);
  std::filesystem::path res_path = log_folder / "results";
  auto res = nlohmann::json::array();
  to_json(res.emplace_back(), shapeA);
  to_json(res.emplace_back(), shapeB);
  to_json(res.emplace_back(), coordinates, collision_present);
  std::ofstream{res_path} << res.dump(1);
}
#endif
} // namespace flx::utils
