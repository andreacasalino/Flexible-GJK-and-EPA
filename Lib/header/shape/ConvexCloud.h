/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_CONVEXCLOUD_H
#define FLX_CONVEXCLOUD_H

#include <shape/ConvexShape.h>
#include <Error.h>
#include <memory>
#include <algorithm>

namespace flx::shape {
    template<typename V, typename V_Container>
    class ConvexCloud {
    public:
        ConvexCloud(std::shared_ptr<V_Container> vertices) : vertices(vertices) {
            if(nullptr == vertices) throw flx::Error("a cloud should contain at least one vertex");
        };

    private:
        void getSupport(Coordinate& result, const Coordinate& direction) const override {
            float max_distance = 0.f, att_distance;
            const V* extremal = nullptr;
            std::for_each(this->vertices.begin(), this->vertices.end(), [&](const V& v){
                att_distance = dot(*it, direction);
                if(att_distance > max_distance) {
                    max_distance = att_distance;
                    extremal = &v;
                }
            });
            if(nullptr == extremal) throw Error("found empty collection of vertices while searching for extremal in ConvexCloud");
            result.x = extremal.x();
            result.y = extremal.y();
            result.z = extremal.z();            
        };

        V_Container vertices;
    };
}

#endif