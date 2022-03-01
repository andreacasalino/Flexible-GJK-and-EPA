/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Coordinate.h>
#include <vector>

/**
 * @brief Just an example of a coordinate representation
 **/
class Vector3d {
public:
  Vector3d(const float x, const float y, const float z) {
    data[0] = x;
    data[1] = y;
    data[2] = z;
  };

  float x() const { return data[0]; }
  float y() const { return data[1]; }
  float z() const { return data[2]; }

private:
  float data[3];
};

hull::Coordinate to_coordinate(const Vector3d &subject);

float dot_product(const Vector3d &subject, const hull::Coordinate &direction);

std::vector<Vector3d> make_random_cloud(const std::size_t samples);

#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <map>
#include <nlohmann/json.hpp>

// You can dig into the sources to understand this class
// if you are really interested. However, this is not strictly
// required to understand how to use the functionalities
// in GjkEpa.h.
//
// In essence, this class is simply generating json files
// in order to generate cool python plots later.
class ResultLogger {
public:
  ResultLogger() = default;

  void logResult(const flx::shape::ConvexShape &shape_a,
                 const flx::shape::ConvexShape &shape_b,
                 const std::string &file_name);

private:
  std::map<const flx::shape::ConvexShape *, nlohmann::json>
      already_encountered_shapes;
};

// class SampleLogger {
// public:
//   SampleLogger(const std::string &fileName);
//   ~SampleLogger();

//   inline void addShape(const flx::shape::ConvexShape &shape) {
//     this->shapes.add(getDescribingCloud(shape));
//   };

//   inline void addQueryResult(const flx::GjkEpa::CoordinatePair &result) {
//     this->results.add(
//         {Vector(result.pointA.x, result.pointA.y, result.pointA.z),
//          Vector(result.pointB.x, result.pointB.y, result.pointB.z)});
//   };

//   void doComplexQuery(flx::GjkEpa &solver, const flx::GjkEpa::ShapePair
//   &pair);

// private:
//   static std::list<Vector>
//   getDescribingCloud(const flx::shape::ConvexShape &shape);
//   inline static std::list<Vector>
//   _getDescribingCloud(const flx::shape::ConvexCloud<std::list<Vector>>
//   &shape) {
//     return shape.getPoints();
//   };
//   static std::list<Vector>
//   _getDescribingCloud(const flx::shape::TransformDecorator &shape);
//   static std::list<Vector>
//   _getDescribingCloud(const flx::shape::RoundDecorator &shape);

//   class VerticesArray {
//   public:
//     VerticesArray(const std::string &name);

//     void add(const std::list<Vector> &element);

//     std::string str();

//   private:
//     bool isFirstElement = true;
//     std::stringstream stream;
//   };
//   mutable VerticesArray shapes;
//   mutable VerticesArray results;

//   std::string logFile;
// #ifdef FLX_LOGGER_ENABLED
//   static std::size_t logCounter;
// #endif
// };
