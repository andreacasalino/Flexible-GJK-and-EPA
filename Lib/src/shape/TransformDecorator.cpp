/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <shape/TransformDecorator.h>
#include <Error.h>
#include <math.h>

namespace flx::shape {
    TransformDecorator::TransformDecorator(std::unique_ptr<ConvexShape> shape)
        : shape(std::move(shape)) {
        if(nullptr == this->shape) throw flx::Error("found null shape when building TransformDecorator");
        if(nullptr != dynamic_cast<TransformDecorator*>(this->shape.get())) throw flx::Error("found null shape when building TransformDecorator");
        // initialize transformation with identity homogeneus matrix
        this->rotation[0][0] = 1.f;
        this->rotation[1][0] = 0.f;
        this->rotation[2][0] = 0.f;

        this->rotation[0][1] = 0.f;
        this->rotation[1][1] = 1.f;
        this->rotation[2][1] = 0.f;

        this->rotation[0][2] = 0.f;
        this->rotation[1][2] = 0.f;
        this->rotation[2][2] = 1.f;

        this->traslation.x = 0.f;
        this->traslation.y = 0.f;
        this->traslation.z = 0.f;
    }

    TransformDecorator::TransformDecorator(std::unique_ptr<ConvexShape> shape, const Coordinate& position, const Coordinate& rotation_XYZ)
        : TransformDecorator(std::move(shape)) {
        this->setRotationXYZ(rotation_XYZ);
        this->setTraslation(position);
    }

    void TransformDecorator::getSupport(Coordinate& result, const Coordinate& direction) const {
        this->transformedDirection.x = this->rotation[0][0]*direction.x + this->rotation[1][0]*direction.y + this->rotation[2][0]*direction.z;
        this->transformedDirection.y = this->rotation[0][1]*direction.x + this->rotation[1][1]*direction.y + this->rotation[2][1]*direction.z;
        this->transformedDirection.z = this->rotation[0][2]*direction.x + this->rotation[1][2]*direction.y + this->rotation[2][2]*direction.z;
        this->shape->getSupport(result, this->transformedDirection);
        result.x = this->rotation[0][0]*result.x + this->rotation[0][1]*result.y + this->rotation[0][2]*result.z + this->traslation.x;
        result.y = this->rotation[1][0]*result.x + this->rotation[1][1]*result.y + this->rotation[1][2]*result.z + this->traslation.y;
        result.z = this->rotation[2][0]*result.x + this->rotation[2][1]*result.y + this->rotation[2][2]*result.z + this->traslation.z;
    }

    void TransformDecorator::setRotationXYZ(const Coordinate& rotation_XYZ) {
        float C1 = cosf(rotation_XYZ.x), S1 = sinf(rotation_XYZ.x);
        float C2 = cosf(rotation_XYZ.y), S2 = sinf(rotation_XYZ.y);
        float C3 = cosf(rotation_XYZ.z), S3 = sinf(rotation_XYZ.z);

        this->rotation[0][0] = C2 * C3;
        this->rotation[0][1] = -C2 * S3;
        this->rotation[0][2] = S2;

        this->rotation[1][0] = S1 * S2*C3 + C1 * S3;
        this->rotation[1][1] = -S1 * S2*S3 + C1 * C3;
        this->rotation[1][2] = -S1 * C2;

        this->rotation[2][0] = -C1 * S2*C3 + S1 * S3;
        this->rotation[2][1] = C1 * S2*S3 + S1 * C3;
        this->rotation[2][2] = C1 * C2;
    }
}