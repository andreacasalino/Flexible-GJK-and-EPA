/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/Error.h>
#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>

namespace flx::shape {
RoundDecorator::RoundDecorator(std::unique_ptr<ConvexShape> shape,
                               const float &ray)
    : ConvexDecorator(std::move(shape)), ray(ray) {
  if (ray <= 0) {
    throw Error{"invalid ray"};
  }
}

RoundDecorator::RoundDecorator(ConvexDecorator &&decorator_o, const float &ray)
    : ConvexDecorator(std::move(decorator_o)), ray(ray) {
  if (ray <= 0) {
    throw Error{"invalid ray"};
  }
}

void RoundDecorator::getSupport(hull::Coordinate &support,
                                const hull::Coordinate &direction) const {
  this->getShape().getSupport(support, direction);
  support.x += this->ray * direction.x;
  support.y += this->ray * direction.y;
  support.z += this->ray * direction.z;
}
} // namespace flx::shape
