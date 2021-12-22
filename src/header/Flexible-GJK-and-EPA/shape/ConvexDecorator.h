/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>
#include <memory>

namespace flx::shape {
class ConvexDecorator : public ConvexShape {
public:
  inline const ConvexShape &getShape() const { return *this->shape; };

protected:
  ConvexDecorator(std::unique_ptr<ConvexShape> shape)
      : shape(std::move(shape)){};

  std::unique_ptr<ConvexShape> shape;
};
} // namespace flx::shape