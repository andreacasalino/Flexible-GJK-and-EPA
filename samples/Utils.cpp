/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Utils.h"
#include <algorithm>
#include <math.h>

namespace {
float get_sample() {
  return -1.f + 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}
} // namespace

std::vector<Vector3d> make_random_cloud(const std::size_t samples) {
  std::vector<Vector3d> result;
  result.reserve(samples);
  for (std::size_t k = 0; k < samples; ++k) {
    result.emplace_back(get_sample(), get_sample(), get_sample());
  }
  return result;
}

hull::Coordinate
to_coordinate(const std::vector<Vector3d>::const_iterator &subject) {
  return hull::Coordinate{subject->x(), subject->y(), subject->z()};
};

float dot_product(const std::vector<Vector3d>::const_iterator &subject,
                  const hull::Coordinate &direction) {
  float result = subject->x() * direction.x;
  result += subject->y() * direction.y;
  result += subject->z() * direction.z;
  return result;
};

std::vector<Vector3d> Vector3dCloud::getPoints() const {
  std::vector<Vector3d> result;
  result.reserve(std::distance(begin, end));
  std::for_each(begin, end, [&result](const Vector3d &element) {
    result.push_back(element);
  });
  return result;
}
