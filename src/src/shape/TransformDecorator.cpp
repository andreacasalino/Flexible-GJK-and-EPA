/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/Error.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>
#include <math.h>

namespace flx::shape {
namespace {
Transformation::RotationMatrix
get_rotation_matrix(const RotationXYZ &rotation_XYZ) {
  Transformation::RotationMatrix rotation;

  float C1 = cosf(rotation_XYZ[0]), S1 = sinf(rotation_XYZ[0]);
  float C2 = cosf(rotation_XYZ[1]), S2 = sinf(rotation_XYZ[1]);
  float C3 = cosf(rotation_XYZ[2]), S3 = sinf(rotation_XYZ[2]);

  rotation[0][0] = C2 * C3;
  rotation[0][1] = -C2 * S3;
  rotation[0][2] = S2;

  rotation[1][0] = S1 * S2 * C3 + C1 * S3;
  rotation[1][1] = -S1 * S2 * S3 + C1 * C3;
  rotation[1][2] = -S1 * C2;

  rotation[2][0] = -C1 * S2 * C3 + S1 * S3;
  rotation[2][1] = C1 * S2 * S3 + S1 * C3;
  rotation[2][2] = C1 * C2;

  return rotation;
}
} // namespace

void Transformation::transform(hull::Coordinate &point) const {
  if (rotation.has_value()) {
    const auto &rot = rotation.value();
    hull::Coordinate point_trsm;
    point_trsm.x =
        rot[0][0] * point.x + rot[0][1] * point.y + rot[0][2] * point.z;
    point_trsm.y =
        rot[1][0] * point.x + rot[1][1] * point.y + rot[1][2] * point.z;
    point_trsm.z =
        rot[2][0] * point.x + rot[2][1] * point.y + rot[2][2] * point.z;
    point = point_trsm;
  }
  if (traslation.has_value()) {
    point.x += traslation->x;
    point.y += traslation->y;
    point.z += traslation->z;
  }
};

void Transformation::setRotationXYZ(const RotationXYZ &new_rotation_XYZ) {
  rotation = get_rotation_matrix(new_rotation_XYZ);
}

Transformation TransformationBuilder::make() const {
  Transformation res;
  if (rotation.has_value()) {
    res.setRotationXYZ(rotation.value());
  }
  if (traslation.has_value()) {
    res.setTraslation(traslation.value());
  }
  return res;
}

TransformDecorator::TransformDecorator(std::unique_ptr<ConvexShape> shape,
                                       const Transformation &transformation)
    : ConvexDecorator(std::move(shape)), transformation(transformation) {}

TransformDecorator::TransformDecorator(ConvexDecorator &&decorator_o,
                                       const Transformation &transformation)
    : ConvexDecorator(std::move(decorator_o)), transformation(transformation) {}

void TransformDecorator::getSupport(hull::Coordinate &result,
                                    const hull::Coordinate &direction) const {
  const auto &maybe_rotation = transformation.getRotation();
  hull::Coordinate transformedDirection;
  if (maybe_rotation.has_value()) {
    const auto &rotation = maybe_rotation.value();
    transformedDirection.x = rotation[0][0] * direction.x +
                             rotation[1][0] * direction.y +
                             rotation[2][0] * direction.z;
    transformedDirection.y = rotation[0][1] * direction.x +
                             rotation[1][1] * direction.y +
                             rotation[2][1] * direction.z;
    transformedDirection.z = rotation[0][2] * direction.x +
                             rotation[1][2] * direction.y +
                             rotation[2][2] * direction.z;
  } else {
    transformedDirection = direction;
  }
  getShape().getSupport(result, transformedDirection);
  transformation.transform(result);
}
} // namespace flx::shape