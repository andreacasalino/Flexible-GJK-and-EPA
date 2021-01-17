/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_TRSFDECORATOR_H
#define FLX_TRSFDECORATOR_H

#include <shape/ConvexDecorator.h>

namespace flx::shape {
    class TransformDecorator : public ConvexDecorator {
    public:
        TransformDecorator(std::unique_ptr<ConvexShape> shape);
        TransformDecorator(std::unique_ptr<ConvexShape> shape, const Coordinate& position, const Coordinate& rotation_XYZ = {0.f,0.f,0.f});

        // const Coordinate& getRotationXYZ() const;
        void setRotationXYZ(const Coordinate& rotation_XYZ);

        inline const Coordinate& getTraslation() const { return this->traslation; };
        inline void setTraslation(const Coordinate& newTraslation) { this->traslation = newTraslation; };

        void getSupport(Coordinate& result, const Coordinate& direction) const override;

        void toRelative(Coordinate& point);
        void toAbsolute(Coordinate& point);

    private:
        float rotation[3][3]; //[rows][columns]
        Coordinate traslation;
        mutable Coordinate transformedDirection;
    };
}

#endif