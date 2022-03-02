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

#include <Flexible-GJK-and-EPA/shape/PointCloud.h>

class Vector3dStorer {
public:
  Vector3dStorer(std::vector<Vector3d> &&buffer) : points(std::move(buffer)){};

  const std::vector<Vector3d> &getPoints() const { return points; }

protected:
  const std::vector<Vector3d> points;
};

class Vector3dCloud : public Vector3dStorer,
                      public PointCloud<std::vector<Vector3d>::const_iterator> {
public:
  Vector3dCloud(std::vector<Vector3d> &&buffer)
      : Vector3dStorer(std::move(buffer)),
        PointCloud<std::vector<Vector3d>::const_iterator>(
            points.begin(), points.end(), dot_product, to_coordinate){};
};

#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <map>
#include <nlohmann/json.hpp>

// You can dig into the sources to understand this class
// if you are really interested. However, this is not strictly
// required to understand how to use the functionalities
// in GjkEpa.h.
//
// In essence, this class is simply generating json files
// in order to later display the results in cool python plots.
class ResultLogger {
public:
  ResultLogger() = default;

  void logResult(const flx::shape::ConvexShape &shape_a,
                 const flx::shape::ConvexShape &shape_b,
                 const flx::QueryResult &result, const std::string &file_name);

private:
  std::map<const flx::shape::ConvexShape *, nlohmann::json>
      already_encountered_shapes;
};
