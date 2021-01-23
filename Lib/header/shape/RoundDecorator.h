/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_ROUNDDECORATOR_H
#define FLX_ROUNDDECORATOR_H

#include <shape/ConvexDecorator.h>

namespace flx::shape {
    /** @brief An object representing the Minkowski sum of a convex shape and a sphere.
     */
    class RoundDecorator : public ConvexDecorator {
    public:
        RoundDecorator(std::unique_ptr<ConvexShape> shape, const float& ray);

        inline const float& getRay() const { return this->ray; };

        void getSupport(Coordinate& result, const Coordinate& direction) const override;

    private:
        float ray;
    };
}

#endif