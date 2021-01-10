/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_ROUNDDECORATOR_H
#define FLX_ROUNDDECORATOR_H

#include <shape/ConvexShape.h>
#include <memory>

namespace flx::shape {
    class RoundDecorator : public ConvexShape {
    public:
        RoundDecorator(std::unique_ptr<ConvexShape> shape, const float& ray);

        inline const float& getRay() const { return this->ray; };

        void getSupport(Coordinate& result, const Coordinate& direction) const override;

    private:
        float ray;
        std::unique_ptr<ConvexShape> shape;
    };
}

#endif