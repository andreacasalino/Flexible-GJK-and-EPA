#include "Utils.h"

flx::QueryResult make_test_query(const std::vector<Vector3d> &a,
                                 const std::vector<Vector3d> &b) {
  return flx::get_closest_points_or_penetration_info(
      Vector3dCloud{a.begin(), a.end(), dot_product, to_coordinate},
      Vector3dCloud{b.begin(), b.end(), dot_product, to_coordinate});
}

std::vector<Vector3d> make_prism(const std::vector<Point2D> &polygon,
                                 const float height) {
  std::vector<Vector3d> points;
  points.reserve(polygon.size() * 2);
  for (const auto &point : polygon) {
    points.emplace_back(point.x, point.y, height);
    points.emplace_back(point.x, point.y, -height);
  }
  return points;
}

std::unique_ptr<Vector3dCloudTest>
make_cloud_test(const std::vector<Vector3d> &points) {
  return std::make_unique<Vector3dCloudTest>(points);
}

float delta_squared_lenght(const flx::CoordinatePair &coordinates) {
  hull::Coordinate diff;
  hull::diff(diff, coordinates.point_in_shape_a, coordinates.point_in_shape_b);
  return hull::normSquared(diff);
}

bool almost_equal(const float a, const float b) { return abs(a - b) < 1e-2; }

bool almost_equal(const hull::Coordinate &a, const hull::Coordinate &b) {
  return almost_equal(a.x, b.x) && almost_equal(a.y, b.y) &&
         almost_equal(a.z, b.z);
}

bool almost_equal2(const hull::Coordinate &a, const float expected_x,
                   const float expected_y) {
  return almost_equal(a.x, expected_x) && almost_equal(a.y, expected_y);
}
