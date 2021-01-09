/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_CONVEXSHAPE_H
#define FLX_CONVEXSHAPE_H

#include <Coordinate.h>

namespace flx::shape {
    class ConvexShape {
    public:
        virtual void getSupport(Coordinate& result, const Coordinate& direction) const = 0;
    };
}

#endif