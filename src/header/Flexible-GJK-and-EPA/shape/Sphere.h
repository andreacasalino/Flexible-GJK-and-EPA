/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>

namespace flx::shape {
class Sphere : public ConvexShape {
public:
  Sphere(const float &ray);
  Sphere(const float &ray, const hull::Coordinate &center);

  const float &getRay() const { return this->ray; };
  const hull::Coordinate &getCenter() const { return this->center; };

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const override;

private:
  float ray;
  hull::Coordinate center;
};
} // namespace flx::shape
