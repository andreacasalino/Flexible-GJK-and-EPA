#include "Utils.h"

Vector3dCloud make_prism_cloud(const std::vector<Point2D> &polygon,
                               const float height) {
  std::vector<Vector3d> points;
  points.reserve(polygon.size() * 2);
  for (const auto &point : polygon) {
    points.emplace_back(point.x, point.y, height);
    points.emplace_back(point.x, point.y, -height);
  }
  return points;
}

std::unique_ptr<Vector3dCloud>
make_prism_cloud_ptr(const std::vector<Point2D> &polygon, const float height) {
  return std::make_unique<Vector3dCloud>(make_prism_cloud(polygon, height));
}

float delta_squared_lenght(const flx::CoordinatePair &coordinates) {
  hull::Coordinate diff;
  hull::diff(diff, coordinates.point_in_shape_a, coordinates.point_in_shape_b);
  return hull::normSquared(diff);
}

bool almost_equal(const float a, const float b) { return abs(a - b) < 1e-4; }
