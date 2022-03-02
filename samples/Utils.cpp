/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Utils.h"
#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>
#include <math.h>

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

std::vector<Vector3d>
get_describing_cloud(const flx::shape::ConvexShape &shape) {
  {
    const auto *as_cloud = dynamic_cast<const Vector3dCloud *>(&shape);
    if (nullptr != as_cloud) {
      return as_cloud->getPoints();
    }
  }
  {
    const auto *as_transformer =
        dynamic_cast<const flx::shape::TransformDecorator *>(&shape);
    if (nullptr != as_transformer) {
      auto result = get_describing_cloud(as_transformer->getShape());
      hull::Coordinate temp;
      for (auto &point : result) {
        temp.x = point.x();
        temp.y = point.y();
        temp.z = point.z();
        as_transformer->getTransformation().transform(temp);
        point = Vector3d{temp.x, temp.y, temp.z};
      }
      return result;
    }
  }
  {
    const auto *as_rounded =
        dynamic_cast<const flx::shape::RoundDecorator *>(&shape);
    if (nullptr != as_rounded) {
      std::vector<Vector3d> result;
      auto sphere = get_sphere_cloud(as_rounded->getRay());
      for (const auto &point : get_describing_cloud(as_rounded->getShape())) {
        for (const auto &point_sphere : sphere) {
          result.emplace_back(point.x() + point_sphere.x(),
                              point.y() + point_sphere.y(),
                              point.z() + point_sphere.z());
        }
      }
      return result;
    }
  }
  return {};
}

void to_json(nlohmann::json &json, const Vector3d &point) {
  json = std::vector<float>{point.x(), point.y(), point.z()};
}

void to_json(nlohmann::json &json, const flx::shape::ConvexShape &shape) {
  json["V"] = get_describing_cloud(shape);
};

const nlohmann::json &get_shape_json(
    std::map<const flx::shape::ConvexShape *, nlohmann::json> &container,
    const flx::shape::ConvexShape &shape) {
  auto container_it = container.find(&shape);
  if (container_it == container.end()) {
    auto &result = container[&shape];
    to_json(result, shape);
    return result;
  }
  return container_it->second;
}
} // namespace

void ResultLogger::logResult(const flx::shape::ConvexShape &shape_a,
                             const flx::shape::ConvexShape &shape_b,
                             const flx::QueryResult &result,
                             const std::string &file_name) {
  const auto &shape_a_json =
      get_shape_json(already_encountered_shapes, shape_a);
  const auto &shape_b_json =
      get_shape_json(already_encountered_shapes, shape_b);
  // do something else
}

/*
#include <Flexible-GJK-and-EPA/Error.h>
#include <fstream>
#include <iostream>
#include <math.h>

std::shared_ptr<std::list<Vector>>
Vector::getRandomCloud(const std::size_t &samplesNumber) {
  std::shared_ptr<std::list<Vector>> samples =
      std::make_shared<std::list<Vector>>();
  for (std::size_t k = 0; k < samplesNumber; ++k) {
    samples->emplace_back(
        static_cast<float>(rand()) / static_cast<float>(RAND_MAX),
        static_cast<float>(rand()) / static_cast<float>(RAND_MAX),
        static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
  }
  return samples;
};

SampleLogger::SampleLogger(const std::string &fileName)
    : shapes("Politopes"), results("Lines"), logFile(fileName) {}

SampleLogger::~SampleLogger() {
  std::ofstream f(this->logFile);
  if (!f.is_open()) {
    std::cout << this->logFile << " it is an invalid log file location"
              << std::endl;
    return;
  }
  f << '{' << std::endl;
  f << this->shapes.str() << std::endl;
  f << ',' << this->results.str() << std::endl;
  f << '}';
}

SampleLogger::VerticesArray::VerticesArray(const std::string &name) {
  this->stream << '\"' << name << "\":[";
}

void SampleLogger::VerticesArray::add(const std::list<Vector> &element) {
  if (this->isFirstElement) {
    this->isFirstElement = false;
  } else {
    this->stream << ',';
  }
  this->stream << "{\"V\":[" << std::endl;
  if (!element.empty()) {
    auto it = element.begin();
    this->stream << '[' << it->x() << "," << it->y() << "," << it->z() << ']'
                 << std::endl;
    ++it;
    for (it; it != element.end(); ++it) {
      this->stream << ",[" << it->x() << "," << it->y() << "," << it->z() << ']'
                   << std::endl;
    }
  }
  this->stream << "]}" << std::endl;
}

std::string SampleLogger::VerticesArray::str() {
  this->stream << "]";
  return this->stream.str();
}

std::list<Vector>
SampleLogger::getDescribingCloud(const flx::shape::ConvexShape &shape) {
  const flx::shape::ConvexCloud<std::list<Vector>> *ptr1 =
      dynamic_cast<const flx::shape::ConvexCloud<std::list<Vector>> *>(&shape);
  if (nullptr != ptr1) {
    return _getDescribingCloud(*ptr1);
  }

  const flx::shape::TransformDecorator *ptr2 =
      dynamic_cast<const flx::shape::TransformDecorator *>(&shape);
  if (nullptr != ptr2) {
    return _getDescribingCloud(*ptr2);
  }

  const flx::shape::RoundDecorator *ptr3 =
      dynamic_cast<const flx::shape::RoundDecorator *>(&shape);
  if (nullptr != ptr3) {
    return _getDescribingCloud(*ptr3);
  }

  throw flx::Error("Unsupported shape");
}

std::list<Vector>
SampleLogger::_getDescribingCloud(const flx::shape::TransformDecorator &shape) {
  auto temp = getDescribingCloud(shape.getShape());
  hull::Coordinate c;
  for (auto it = temp.begin(); it != temp.end(); ++it) {
    c = {it->x(), it->y(), it->z()};
    shape.transform(c);
    *it = Vector(c.x, c.y, c.z);
  }
  return temp;
}

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
std::list<Vector>
SampleLogger::_getDescribingCloud(const flx::shape::RoundDecorator &shape) {
  auto temp = getDescribingCloud(shape.getShape());

  std::size_t dim = temp.size();
  auto it = temp.begin();
  std::uint8_t alfa, beta;
  Interval alfaInterval(-0.5f * PI, 0.5f * PI, N_ALFA);
  Interval betaInterval(0.f, 2.f * PI, N_BETA);
  float Calfa, Salfa, Cbeta, Sbeta;
  float coor[3];
  for (std::size_t d = 0; d < dim; ++d) {
    for (beta = 0; beta < N_BETA; ++beta) {
      alfaInterval.reset();
      Cbeta = cosf(betaInterval.eval());
      Sbeta = sinf(betaInterval.eval());
      for (alfa = 0; alfa < N_ALFA; ++alfa) {
        Calfa = cosf(alfaInterval.eval());
        Salfa = sinf(alfaInterval.eval());
        coor[0] = Calfa * Cbeta * shape.getRay() + it->x();
        coor[1] = Calfa * Sbeta * shape.getRay() + it->y();
        coor[2] = Salfa * shape.getRay() + it->z();
        temp.emplace_back(coor[0], coor[1], coor[2]);
        ++betaInterval;
      }
      ++alfaInterval;
    }
    ++it;
  }

  return temp;
}

#ifdef FLX_LOGGER_ENABLED
std::size_t SampleLogger::logCounter = 0;
#endif

void SampleLogger::doComplexQuery(flx::GjkEpa &solver,
                                  const flx::GjkEpa::ShapePair &pair) {
  flx::GjkEpa::CoordinatePair result;
  flx::GjkEpa::ResultType res = solver.doComplexQuery(
      pair, result
#ifdef FLX_LOGGER_ENABLED
      ,
      std::string { "SolverLog" + std::to_string(++logCounter) }
#endif
  );

  this->addShape(pair.shapeA);
  this->addShape(pair.shapeB);
  this->addQueryResult(result);

  if (flx::GjkEpa::ResultType::closestPoints == res) {
    std::cout << "collision absent, closest points" << std::endl;
  } else {
    std::cout << "collision present, penetration depth" << std::endl;
  }
  std::cout << "<" << result.pointA.x << "," << result.pointA.y << ","
            << result.pointA.z << ">" << std::endl;
  std::cout << "<" << result.pointB.x << "," << result.pointB.y << ","
            << result.pointB.z << ">" << std::endl;
  std::cout << std::endl;
}
*/
