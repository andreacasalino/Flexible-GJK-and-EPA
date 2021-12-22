/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Error.h>
#include <memory>
#include <shape/ConvexShape.h>

namespace flx::shape {
/** @brief A point cloud of 3d points.
 * Container should be an iterable container, storing the
 * vertices of the cloud. Each element in the container
 * should be an object having the following const methods: x(), y(), z()
 * that returns the coordinate of the point.
 */
template <typename Container> class ConvexCloud : public ConvexShape {
public:
  ConvexCloud(std::shared_ptr<Container> vertices) : vertices(vertices) {
    if (nullptr == vertices)
      throw flx::Error("a cloud should contain at least one vertex");
    if (this->vertices->empty())
      throw flx::Error("a cloud should contain at least one vertex");
  };

  void getSupport(Coordinate &result,
                  const Coordinate &direction) const override {
    float max_distance, att_distance;
    auto it = this->vertices->begin();
    const auto *extremal = &(*it);
    max_distance = it->x() * direction.x;
    max_distance += it->y() * direction.y;
    max_distance += it->z() * direction.z;
    auto itEnd = this->vertices->end();
    ++it;
    for (it; it != itEnd; ++it) {
      att_distance = it->x() * direction.x;
      att_distance += it->y() * direction.y;
      att_distance += it->z() * direction.z;
      if (att_distance > max_distance) {
        max_distance = att_distance;
        extremal = &(*it);
      }
    }
    if (nullptr == extremal)
      throw Error("found empty collection of vertices while searching for "
                  "extremal in ConvexCloud");
    result.x = extremal->x();
    result.y = extremal->y();
    result.z = extremal->z();
  };

  inline const Container &getPoints() const { return *this->vertices.get(); };

private:
  std::shared_ptr<Container> vertices;
};
} // namespace flx::shape
