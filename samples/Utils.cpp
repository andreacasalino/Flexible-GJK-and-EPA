/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Utils.h"
#include <Flexible-GJK-and-EPA/Error.h>
#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>
#include <fstream>
#include <iostream>
#include <math.h>

namespace {
float get_sample() {
  return -1.f + 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}
} // namespace

std::vector<Vector3d> make_random_cloud(const std::size_t samples) {
  std::vector<Vector3d> result;
  result.reserve(samples);
  for (std::size_t k = 0; k < samples; ++k) {
    result.emplace_back(get_sample(), get_sample(), get_sample());
  }
  return result;
}

hull::Coordinate
to_coordinate(const std::vector<Vector3d>::const_iterator &subject) {
  return hull::Coordinate{subject->x(), subject->y(), subject->z()};
};

float dot_product(const std::vector<Vector3d>::const_iterator &subject,
                  const hull::Coordinate &direction) {
  float result = subject->x() * direction.x;
  result += subject->y() * direction.y;
  result += subject->z() * direction.z;
  return result;
};

namespace logger {

namespace {
class Interval {
public:
  Interval(const float &min, const float &max, const std::size_t &nPoints)
      : delta((max - min) / static_cast<float>(nPoints - 1)), min(min) {
    this->reset();
  };

  inline void reset() { this->value = this->min; };

  inline Interval &operator++() {
    this->value += this->delta;
    return *this;
  };

  inline const float &eval() const { return this->value; };

private:
  float value;
  const float delta;
  const float min;
};
constexpr float PI = 3.14159f;
constexpr std::uint8_t N_ALFA = 5;
constexpr std::uint8_t N_BETA = 10;
std::vector<Vector3d> get_sphere_cloud(const float ray) {
  std::vector<Vector3d> result;
  float Calfa, Salfa, Cbeta, Sbeta;
  Interval alfaInterval(-0.5f * PI, 0.5f * PI, N_ALFA);
  Interval betaInterval(0.f, 2.f * PI, N_BETA);
  for (std::uint8_t beta = 0; beta < N_BETA; ++beta) {
    Cbeta = cosf(betaInterval.eval());
    Sbeta = sinf(betaInterval.eval());
    alfaInterval.reset();
    for (std::uint8_t alfa = 0; alfa < N_ALFA; ++alfa) {
      Calfa = cosf(alfaInterval.eval());
      Salfa = sinf(alfaInterval.eval());
      result.emplace_back(Calfa * Cbeta * ray, Calfa * Sbeta * ray,
                          Salfa * ray);
      ++betaInterval;
    }
    ++alfaInterval;
  }
  return result;
};
} // namespace

std::vector<Vector3d> &
CloudsMemoizer::getCloudVertices(const flx::shape::ConvexShape &shape) {
  auto clouds_it = clouds.find(&shape);
  if (clouds_it == clouds.end()) {
    std::vector<Vector3d> points;
    const auto *as_cloud = dynamic_cast<const Vector3dCloud *>(&shape);
    const auto *as_transformer =
        dynamic_cast<const flx::shape::TransformDecorator *>(&shape);
    const auto *as_rounded =
        dynamic_cast<const flx::shape::RoundDecorator *>(&shape);
    if (nullptr != as_cloud) {
      points = as_cloud->getPoints();
      goto label;
    }
    if (nullptr != as_transformer) {
      points = this->getCloudVertices(as_transformer->getShape());
      hull::Coordinate temp;
      for (auto &point : points) {
        temp.x = point.x();
        temp.y = point.y();
        temp.z = point.z();
        as_transformer->getTransformation().transform(temp);
        point = Vector3d{temp.x, temp.y, temp.z};
      }
      goto label;
    }
    if (nullptr != as_rounded) {
      auto sphere = get_sphere_cloud(as_rounded->getRay());
      for (const auto &point : this->getCloudVertices(as_rounded->getShape())) {
        for (const auto &point_sphere : sphere) {
          points.emplace_back(point.x() + point_sphere.x(),
                              point.y() + point_sphere.y(),
                              point.z() + point_sphere.z());
        }
      }
      goto label;
    }

  label:
    clouds_it = clouds.emplace(&shape, std::move(points)).first;
  }
  return clouds_it->second;
}

namespace {
void to_json(nlohmann::json &recipient, const Vector3d &point) {
  recipient = std::array<float, 3>{point.x(), point.y(), point.z()};
}

void to_json(nlohmann::json &recipient, const std::vector<Vector3d> &points) {
  auto array = nlohmann::json::array();
  for (const auto &point : points) {
    to_json(array.emplace_back(), point);
  }
  recipient = std::move(array);
}
} // namespace

void SubPlot::toJson(nlohmann::json &recipient) const {
  recipient["Politopes"] = this->shapes;
  recipient["Lines"] = this->lines;
}

void SubPlot::addShape(const flx::shape::ConvexShape &shape) {
  to_json(shapes.emplace_back()["Vertices"],
          collection->getCloudVertices(shape));
}

void SubPlot::addLine(const hull::Coordinate &a, const hull::Coordinate &b) {
  auto &new_line = lines.emplace_back();
  std::vector<Vector3d> vertices;
  vertices.emplace_back(a.x, a.y, a.z);
  vertices.emplace_back(b.x, b.y, b.z);
  to_json(new_line["Peers"], vertices);
}

SubPlot &Figure::addSubPlot(const std::string &title) {
  SubPlot new_sub_plot(collection, title);
  return this->sub_plots.emplace_back(std::move(new_sub_plot));
}

void Figure::log(const std::string &file_name) const {
  std::ofstream stream(file_name);
  if (!stream.is_open()) {
    throw flx::Error{"Invalid file location to generate log"};
  }
  auto sub_plots_as_json = nlohmann::json::array();
  for (const auto &sub_plot : sub_plots) {
    sub_plot.toJson(sub_plots_as_json.emplace_back());
  }
  stream << sub_plots_as_json.dump();
  std::cout << "Use `python3 Visualize.py " << file_name
            << "` to see the results" << std::endl;
}

Figure Manager::makeFigure() const { return Figure{collection}; }

void Manager::logSingleQuery(const flx::shape::ConvexShape &shape_a,
                             const flx::shape::ConvexShape &shape_b,
                             const flx::QueryResult &result,
                             const std::string &file_name) {
  Figure fig = this->makeFigure();
  {
    auto &subplot = fig.addSubPlot("Shapes object of the GJK-EPA query");
    subplot.addLine(result.result.point_in_shape_a,
                    result.result.point_in_shape_b);
    subplot.addShape(shape_a);
    subplot.addShape(shape_b);
  }
  {
    std::string title;
    hull::Coordinate delta;
    hull::diff(delta, result.result.point_in_shape_a,
               result.result.point_in_shape_b);
    if (result.is_closest_pair_or_penetration_info) {
      title = "Original positions of the shapes";
    } else {
      title = "METTERE ... TODO";
      hull::invert(delta);
    }
    auto &subplot = fig.addSubPlot(title);

    subplot.addShape(shape_a);

    std::vector<Vector3d> shape_b_points =
        collection->getCloudVertices(shape_b);
    for (auto &shape_b_point : shape_b_points) {
      shape_b_point =
          Vector3d{shape_b_point.x() + delta.x, shape_b_point.y() + delta.y,
                   shape_b_point.z() + delta.z};
    }
    Vector3dCloud shape_b_transformed(std::move(shape_b_points));
    subplot.addShape(shape_b_transformed);
  }
  fig.log(file_name);
}
} // namespace logger
