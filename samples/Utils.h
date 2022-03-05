/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Coordinate.h>
#include <memory>
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

hull::Coordinate
to_coordinate(const std::vector<Vector3d>::const_iterator &subject);

float dot_product(const std::vector<Vector3d>::const_iterator &subject,
                  const hull::Coordinate &direction);

std::vector<Vector3d> make_random_cloud(const std::size_t samples);

#include <Flexible-GJK-and-EPA/shape/PointCloud.h>

class Vector3dCloud : public flx::shape::ConvexShape {
public:
  Vector3dCloud(const std::vector<Vector3d> &buffer) : points(buffer){};

  const std::vector<Vector3d> &getPoints() const { return points; }

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const final {
    auto it_max = points.begin();
    float dot_max = dot_product(it_max, direction);
    for (auto it = points.begin(); it != points.end(); ++it) {
      float dot = dot_product(it, direction);
      if (dot > dot_max) {
        dot_max = dot;
        it_max = it;
      }
    }
    support = to_coordinate(it_max);
  }

private:
  const std::vector<Vector3d> points;
};
