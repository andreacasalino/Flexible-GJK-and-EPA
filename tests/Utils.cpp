#include "Utils.h"

#include <math.h>
#include <random>

namespace flx {
namespace shape {
float get_support(const PointsIterator &point,
                  const hull::Coordinate &direction) {
  return hull::dot(*point, direction);
}

hull::Coordinate convert(const PointsIterator &point) { return *point; };

TestCloud::TestCloud(Points &&buffer)
    : PointsStore(std::move(buffer)), PointCloud<PointsIterator>(
                                          points.begin(), points.end(),
                                          get_support, convert) {}

TestCloud make_prism_cloud(const std::vector<Point2D> &polygon,
                           const float height) {
  std::vector<hull::Coordinate> points;
  points.reserve(polygon.size() * 2);
  for (const auto &point : polygon) {
    points.push_back(hull::Coordinate{point.x, point.y, height});
    points.push_back(hull::Coordinate{point.x, point.y, -height});
  }
  return points;
}

std::unique_ptr<ConvexShape>
make_prism_cloud_ptr(const std::vector<Point2D> &polygon, const float height) {
  return std::make_unique<TestCloud>(make_prism_cloud(polygon, height));
}

float sample_coordinate() {
  return 2.f * static_cast<float>(rand()) / static_cast<float>(RAND_MAX) - 1.f;
}

TestCloud make_random_cloud(const std::size_t points) {
  std::vector<hull::Coordinate> buffer;
  buffer.reserve(points);
  for (std::size_t k = 0; k < points; ++k) {
    buffer.push_back(hull::Coordinate{sample_coordinate(), sample_coordinate(),
                                      sample_coordinate()});
  }
  return buffer;
}
} // namespace shape

float delta_squared_lenght(const CoordinatePair &coordinates) {
  hull::Coordinate diff;
  hull::diff(diff, coordinates.point_in_shape_a, coordinates.point_in_shape_b);
  return hull::normSquared(diff);
}

bool almost_equal(const float a, const float b) { return abs(a - b) < 1e-4; }
} // namespace flx
