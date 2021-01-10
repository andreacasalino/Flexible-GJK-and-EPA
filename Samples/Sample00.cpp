/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <shape/ConvexCloud.h>
#include <shape/TransformDecorator.h>
#include <list>
#include <iostream>

class V {
public:
    V(const float& x, const float& y, const float& z) {
        this->v = {x, y, z};
    };

    inline float x() const { return this->v.x; };
    inline float y() const { return this->v.y; };
    inline float z() const { return this->v.z; };

private:
    flx::Coordinate v;
};

std::ostream& operator<<(std::ostream& s, const flx::Coordinate& c) {
    s << "<" << c.x << "," << c.y << "," << c.z << ">";
    return s;
};

int main() {
    std::unique_ptr<flx::shape::ConvexShape> shape;
    {
        std::shared_ptr<std::list<V>> coordinates = std::make_shared<std::list<V>>();
        coordinates->emplace_back(1.f, 0.f, 0.f);
        coordinates->emplace_back(-1.f, 0.f, 0.f);
        coordinates->emplace_back(0.f, 1.f, 0.f);
        coordinates->emplace_back(0.f, -1.f, 0.f);
        coordinates->emplace_back(0.f, 0.f, 1.f);
        coordinates->emplace_back(0.f, 0.f, -1.f);
        shape = std::make_unique<flx::shape::ConvexCloud<std::list<V>>>(coordinates);
    }
    shape = std::make_unique<flx::shape::TransformDecorator>(std::move(shape));
    // static_cast<flx::shape::TransformDecorator*>(shape.get())->setTraslation(flx::Coordinate{0.2f, 0.2f, 0.f}); 
    static_cast<flx::shape::TransformDecorator*>(shape.get())->setRotationXYZ(flx::Coordinate{0.f, 0.f, 0.2f}); 

    flx::Coordinate result;

    shape->getSupport(result , flx::Coordinate{1.f, 0.1f, 0.f});
    std::cout << result << std::endl;

    shape->getSupport(result , flx::Coordinate{-1.f, 0.f, 0.f});
    std::cout << result << std::endl;

    return EXIT_SUCCESS;
}