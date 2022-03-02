/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/Error.h>
#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>
#include <memory>

namespace flx::shape {
class ConvexDecorator : public ConvexShape {
public:
  inline const ConvexShape &getShape() const { return *this->shape; };

  ConvexDecorator(ConvexDecorator &&o);
  ConvexDecorator &operator=(ConvexDecorator &&o);

protected:
  ConvexDecorator(std::unique_ptr<ConvexShape> shape)
      : shape(std::move(shape)) {
    if (nullptr == this->shape) {
      throw Error{"null shape found when building a shape decorator"};
    }
  };

private:
  std::unique_ptr<ConvexShape> shape;
};
} // namespace flx::shape
