/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <shape/RoundDecorator.h>
#include <Error.h>

namespace flx::shape {
    RoundDecorator::RoundDecorator(std::unique_ptr<ConvexShape> shape, const float& ray)
        : ray(ray)
        , shape(std::move(shape)) {
        if(ray <= 0.f) throw flx::Error("invalid ray for RoundDecorator");
        if(nullptr == this->shape) throw flx::Error("found null shape when building RoundDecorator");
        if(nullptr != dynamic_cast<RoundDecorator*>(this->shape.get())) throw flx::Error("found null shape when building RoundDecorator");
    }

    void RoundDecorator::getSupport(Coordinate& result, const Coordinate& direction) const {
        this->shape->getSupport(result, direction);
        result.x += this->ray * direction.x;
        result.y += this->ray * direction.y;
        result.z += this->ray * direction.z;
    }
}