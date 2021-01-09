/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_TRSFDECORATOR_H
#define FLX_TRSFDECORATOR_H

#include <shape/ConvexShape.h>
#include <memory>

namespace flx::shape {
    class TransformDecorator : public ConvexShape {
    public:
        TransformDecorator(std::unique_ptr<ConvexShape> shape);
        TransformDecorator(std::unique_ptr<ConvexShape> shape, const Coordinate& position, const Coordinate& rotation_XYZ = {0.f,0.f,0.f});

        // const Coordinate& getRotationXYZ() const;
        void setRotationXYZ(const Coordinate& rotation_XYZ);

        inline const Coordinate& getTraslation() const { return this->traslation; };
        inline void setTraslation(const Coordinate& rotation) { this->traslation = rotation; };

    private:
        void getSupport(Coordinate& result, const Coordinate& direction) const override;

        float rotation[3][3]; //[rows][columns]
        Coordinate traslation;
        std::unique_ptr<ConvexShape> shape;
        mutable Coordinate transformedDirection;
    };
}

#endif