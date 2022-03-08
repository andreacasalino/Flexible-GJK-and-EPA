/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexDecorator.h>

namespace flx::shape {
/** @brief An object representing the Minkowski sum of a convex shape and a
 * sphere of a certain radius.
 */
class RoundDecorator : public ConvexDecorator {
public:
  RoundDecorator(std::unique_ptr<ConvexShape> shape, const float &ray);
  RoundDecorator(ConvexDecorator &&decorator_o, const float &ray);

  inline const float &getRay() const { return this->ray; };

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const override;

private:
  float ray;
};
} // namespace flx::shape
