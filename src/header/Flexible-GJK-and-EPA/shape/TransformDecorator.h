/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <shape/ConvexDecorator.h>

namespace flx::shape {
/** @brief An object representing a roto-traslated convex shape.
 */
class TransformDecorator : public ConvexDecorator {
public:
  TransformDecorator(std::unique_ptr<ConvexShape> shape);
  TransformDecorator(std::unique_ptr<ConvexShape> shape,
                     const Coordinate &position,
                     const Coordinate &rotation_XYZ = {0.f, 0.f, 0.f});

  // const Coordinate& getRotationXYZ() const; //todo
  void setRotationXYZ(const Coordinate &rotation_XYZ);

  inline const Coordinate &getTraslation() const { return this->traslation; };
  inline void setTraslation(const Coordinate &newTraslation) {
    this->traslation = newTraslation;
  };

  void getSupport(Coordinate &result,
                  const Coordinate &direction) const override;

  /** @brief Get the passed position, as seen from the frame describing the
   * roto-traslation of this object.
   */
  inline void transform(Coordinate &point) const {
    float temp[3];
    temp[0] = this->rotation[0][0] * point.x + this->rotation[0][1] * point.y +
              this->rotation[0][2] * point.z + this->traslation.x;
    temp[1] = this->rotation[1][0] * point.x + this->rotation[1][1] * point.y +
              this->rotation[1][2] * point.z + this->traslation.y;
    temp[2] = this->rotation[2][0] * point.x + this->rotation[2][1] * point.y +
              this->rotation[2][2] * point.z + this->traslation.z;
    point.x = temp[0];
    point.y = temp[1];
    point.z = temp[2];
  };

private:
  float rotation[3][3]; //[rows][columns]
  Coordinate traslation;
  mutable Coordinate transformedDirection;
};
} // namespace flx::shape
