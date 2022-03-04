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
Transformation::Rotation get_identity_matrix() {
  Transformation::Rotation rotation;

  rotation[0][0] = 1.f;
  rotation[1][0] = 0.f;
  rotation[2][0] = 0.f;

  rotation[0][1] = 0.f;
  rotation[1][1] = 1.f;
  rotation[2][1] = 0.f;

  rotation[0][2] = 0.f;
  rotation[1][2] = 0.f;
  rotation[2][2] = 1.f;

  return rotation;
}

Transformation::Rotation get_rotation_matrix(const RotationXYZ &rotation_XYZ) {
  Transformation::Rotation rotation;

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
  hull::Coordinate result;
  result.x = this->rotation[0][0] * point.x + this->rotation[0][1] * point.y +
             this->rotation[0][2] * point.z + this->traslation.x;
  result.y = this->rotation[1][0] * point.x + this->rotation[1][1] * point.y +
             this->rotation[1][2] * point.z + this->traslation.y;
  result.z = this->rotation[2][0] * point.x + this->rotation[2][1] * point.y +
             this->rotation[2][2] * point.z + this->traslation.z;
  point = result;
};

Transformation::Transformation()
    : rotation(get_identity_matrix()), traslation(hull::Coordinate{0, 0, 0}) {}

Transformation::Transformation(const hull::Coordinate &traslation)
    : rotation(get_identity_matrix()), traslation(traslation) {}
Transformation::Transformation(const RotationXYZ &rotation_XYZ)
    : rotation(get_rotation_matrix(rotation_XYZ)),
      traslation(hull::Coordinate{0, 0, 0}) {}

Transformation::Transformation(const hull::Coordinate &traslation,
                               const RotationXYZ &rotation_XYZ)
    : rotation(get_rotation_matrix(rotation_XYZ)), traslation(traslation) {}

TransformDecorator::TransformDecorator(std::unique_ptr<ConvexShape> shape,
                                       const Transformation &transformation)
    : ConvexDecorator(std::move(shape)), transformation(transformation) {}

TransformDecorator::TransformDecorator(ConvexDecorator &&decorator_o,
                                       const Transformation &transformation)
    : ConvexDecorator(std::move(decorator_o)), transformation(transformation) {}

void TransformDecorator::getSupport(hull::Coordinate &result,
                                    const hull::Coordinate &direction) const {
  const auto &rotation = transformation.getRotation();
  hull::Coordinate transformedDirection;

  transformedDirection.x = rotation[0][0] * direction.x +
                           rotation[1][0] * direction.y +
                           rotation[2][0] * direction.z;
  transformedDirection.y = rotation[0][1] * direction.x +
                           rotation[1][1] * direction.y +
                           rotation[2][1] * direction.z;
  transformedDirection.z = rotation[0][2] * direction.x +
                           rotation[1][2] * direction.y +
                           rotation[2][2] * direction.z;
  getShape().getSupport(result, transformedDirection);
  transformation.transform(result);
}
} // namespace flx::shape