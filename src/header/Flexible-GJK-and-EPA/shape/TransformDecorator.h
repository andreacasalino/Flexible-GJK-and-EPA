/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexDecorator.h>

#include <array>
#include <optional>

namespace flx::shape {
using RotationXYZ = std::array<float, 3>;

class Transformation {
public:
  Transformation() = default;

  void setTraslation(const hull::Coordinate &new_traslation) {
    traslation.emplace(new_traslation);
  }
  void setRotationXYZ(const RotationXYZ &new_rotation_XYZ);

  /** @brief Transform the point into the global world coordinate system,
   * assuming that the passed value intially stores the point coordinates as
   * seen from the frame represented by this transformation
   */
  void transform(hull::Coordinate &point) const;

  /** @brief Representative of a rotation matrix [rows][columns]
   */
  using RotationMatrix = std::array<std::array<float, 3>, 3>;

  const auto &getRotation() const { return rotation; };
  const auto &getTraslation() const { return traslation; };

private:
  std::optional<RotationMatrix> rotation;
  std::optional<hull::Coordinate> traslation;
};

struct TransformationBuilder {
  TransformationBuilder() = default;

  TransformationBuilder &setTraslation(const hull::Coordinate &new_traslation) {
    traslation = new_traslation;
    return *this;
  }
  TransformationBuilder &setRotationXYZ(const RotationXYZ &new_rotation_XYZ) {
    rotation = new_rotation_XYZ;
    return *this;
  }

  Transformation make() const;

  operator Transformation() const { return make(); }

private:
  std::optional<RotationXYZ> rotation;
  std::optional<hull::Coordinate> traslation;
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

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const override;

private:
  Transformation transformation;
};
} // namespace flx::shape
