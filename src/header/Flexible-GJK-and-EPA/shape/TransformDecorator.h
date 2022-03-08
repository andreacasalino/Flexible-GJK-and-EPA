/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexDecorator.h>
#include <array>

namespace flx::shape {
using RotationXYZ = std::array<float, 3>;

class Transformation {
public:
  Transformation();

  Transformation(const hull::Coordinate &traslation);
  Transformation(const RotationXYZ &rotation_XYZ);

  Transformation(const hull::Coordinate &traslation,
                 const RotationXYZ &rotation_XYZ);

  void setTraslation(const hull::Coordinate &new_traslation);
  void setRotationXYZ(const RotationXYZ &new_rotation_XYZ);

  /** @brief Transform the point into the global world coordinate system,
   * assuming that the passed value intially stores the point coordinates as
   * seen from the frame represented by this transformation
   */
  void transform(hull::Coordinate &point) const;

  /** @brief Representative of a rotation matrix [rows][columns]
   */
  using Rotation = std::array<std::array<float, 3>, 3>;

  const Rotation &getRotation() const { return rotation; };
  const hull::Coordinate &getTraslation() const { return traslation; };

private:
  Rotation rotation;
  hull::Coordinate traslation;
};

/** @brief An object representing a roto-traslated convex shape.
 */
class TransformDecorator : public ConvexDecorator {
public:
  TransformDecorator(std::unique_ptr<ConvexShape> shape,
                     const Transformation &transformation);
  TransformDecorator(ConvexDecorator &&decorator_o,
                     const Transformation &transformation);

  const Transformation &getTransformation() const { return transformation; };
  Transformation &getTransformation() { return transformation; };
  void setTransformation(const Transformation &new_transformation) {
    transformation = new_transformation;
  };

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const override;

private:
  Transformation transformation;
};
} // namespace flx::shape
