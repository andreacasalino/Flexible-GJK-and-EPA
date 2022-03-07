/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Coordinate.h>

/**
 * @brief Just an example of a 3D coordinate representation
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

#include <vector>

hull::Coordinate
to_coordinate(const std::vector<Vector3d>::const_iterator &subject);

float dot_product(const std::vector<Vector3d>::const_iterator &subject,
                  const hull::Coordinate &direction);

#include <Flexible-GJK-and-EPA/shape/PointCloud.h>
#include <memory>

using Points = std::shared_ptr<std::vector<Vector3d>>;
Points make_random_cloud(const std::size_t samples);

class Vector3dCloud
    : public flx::shape::PointCloud<std::vector<Vector3d>::const_iterator> {
public:
  Vector3dCloud(const Points &wrapped)
      : flx::shape::PointCloud<std::vector<Vector3d>::const_iterator>(
            wrapped->begin(), wrapped->end(), dot_product, to_coordinate) {
    this->points = wrapped;
  };

  const std::vector<Vector3d> &getPoints() const { return *points; };

private:
  Points points;
};
