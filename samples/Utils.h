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

hull::Coordinate
to_coordinate(const std::vector<Vector3d>::const_iterator &subject);

float dot_product(const std::vector<Vector3d>::const_iterator &subject,
                  const hull::Coordinate &direction);

std::vector<Vector3d> make_random_cloud(const std::size_t samples);

#include <Flexible-GJK-and-EPA/shape/PointCloud.h>

class Vector3dStorer {
public:
  Vector3dStorer(const std::vector<Vector3d> &buffer) : points(buffer){};

  const std::vector<Vector3d> &getPoints() const { return points; }

protected:
  const std::vector<Vector3d> points;
};

class Vector3dCloud
    : public Vector3dStorer,
      public flx::shape::PointCloud<std::vector<Vector3d>::const_iterator> {
public:
  Vector3dCloud(const std::vector<Vector3d> &buffer)
      : Vector3dStorer(buffer),
        flx::shape::PointCloud<std::vector<Vector3d>::const_iterator>(
            points.begin(), points.end(), dot_product, to_coordinate){};
};
