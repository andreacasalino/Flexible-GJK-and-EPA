#include <catch2/catch_test_macros.hpp>

#include "Utils.h"
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

TEST_CASE("Polygons and transformations", "[prisms][transform]") {
  Points tethraedron_points = std::make_shared<std::vector<Vector3d>>();
  tethraedron_points->emplace_back(0, 0, 0);
  tethraedron_points->emplace_back(-1.f, -1.f, 0);
  tethraedron_points->emplace_back(-1.f, 1.f, 0);
  tethraedron_points->emplace_back(-1.f, 0, 1.f);
  Vector3dCloud tethraedron(tethraedron_points);

  auto square_points = make_prism(
      std::vector<Point2D>{{1.f, 1.f}, {-1.f, 1.f}, {-1.f, -1.f}, {1.f, -1.f}},
      1.f);

  SECTION("Traslation") {
    flx::shape::Transformation trsf(hull::Coordinate{5.f, 0, 0});
    flx::shape::TransformDecorator shape_b_trsf(
        std::make_unique<Vector3dCloud>(square_points), trsf);

    auto query =
        flx::get_closest_points_or_penetration_info(tethraedron, shape_b_trsf);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 4.f, 0));
  }

  SECTION("Roto traslation") {
    flx::shape::Transformation trsf(
        hull::Coordinate{5.f, 0, 0},
        flx::shape::RotationXYZ{0, 0.5f * 3.141f, 0});
    flx::shape::TransformDecorator shape_b_trsf(
        std::make_unique<Vector3dCloud>(square_points), trsf);

    auto query =
        flx::get_closest_points_or_penetration_info(tethraedron, shape_b_trsf);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 4.f, 0));
  }
}
