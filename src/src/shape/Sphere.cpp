/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/Error.h>
#include <Flexible-GJK-and-EPA/shape/Sphere.h>

namespace flx::shape {
Sphere::Sphere(const float &ray) : Sphere(ray, hull::Coordinate{0, 0, 0}) {}

Sphere::Sphere(const float &ray, const hull::Coordinate &center)
    : ray(ray), center(center) {
  if (ray < 0.f) {
    throw Error{"Negative ray for sphere"};
  }
}

void Sphere::getSupport(hull::Coordinate &support,
                        const hull::Coordinate &direction) const {
  support.x = direction.x * ray + center.x;
  support.y = direction.y * ray + center.y;
  support.z = direction.z * ray + center.z;
}
} // namespace flx::shape
