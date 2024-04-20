#include "UtilsMore.h"

namespace flx::utils {
flx::QueryResult make_test_query(const Points &a, const Points &b
#ifdef GJK_EPA_DIAGNOSTIC
                                 ,
                                 const std::string &subpath
#endif
) {

#ifdef GJK_EPA_DIAGNOSTIC
  GjkEpaLogger logger{subpath};
#endif
  auto query = flx::get_closest_points_or_penetration_info(Vector3dCloud{a},
                                                           Vector3dCloud{b}
#ifdef GJK_EPA_DIAGNOSTIC
                                                           ,
                                                           &logger
#endif
  );

#ifdef GJK_EPA_DIAGNOSTIC
  logger.addResult(Vector3dCloud{a}, Vector3dCloud{b}, query.result,
                   !query.is_closest_pair_or_penetration_info);
#endif

  return query;
}

Points make_prism(const std::vector<Point2D> &polygon, const float height) {
  Points points = std::make_shared<std::vector<Vector3d>>();
  points->reserve(polygon.size() * 2);
  for (const auto &point : polygon) {
    points->emplace_back(point.x, point.y, height);
    points->emplace_back(point.x, point.y, -height);
  }
  return points;
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
} // namespace flx::utils