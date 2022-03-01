#include "Utils.h"

#include <math.h>

namespace flx {
namespace shape {
float get_support(const PointsIterator &point,
                  const hull::Coordinate &direction) {
  return hull::dot(*point, direction);
}

hull::Coordinate convert(const PointsIterator &point) { return *point; };

TestCloud::TestCloud(const Points &points)
    : PointCloud<PointsIterator>(points->begin(), points->end(), get_support,
                                 convert){};

TestCloud make_prism_cloud(const std::vector<Point2D> &polygon,
                           const float height) {
  std::vector<hull::Coordinate> points;
  points.reserve(polygon.size() * 2);
  for (const auto &point : points) {
    points.push_back(hull::Coordinate{point.x, point.y, height});
    points.push_back(hull::Coordinate{point.x, point.y, -height});
  }
  return std::make_shared<const std::vector<hull::Coordinate>>(
      std::move(points));
}
} // namespace shape

float delta_squared_lenght(const CoordinatePair &coordinates) {
  hull::Coordinate diff;
  hull::diff(diff, coordinates.point_in_shape_a, coordinates.point_in_shape_b);
  return hull::normSquared(diff);
}

bool almost_equal(const float a, const float b) { return abs(a - b) < 1e-4; }
} // namespace flx
