#include <catch2/catch_test_macros.hpp>

#include "Utils.h"
#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>
#include <Flexible-GJK-and-EPA/shape/Sphere.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

Points make_tethreadron_points() {
  std::vector<Vector3d> points{Vector3d{0, 0, 0}, Vector3d{-1.f, -1.f, 0},
                               Vector3d{-1.f, 1.f, 0}, Vector3d{-1.f, 0, 1.f}};
  return std::make_shared<std::vector<Vector3d>>(points);
}

TEST_CASE("Polygons and sphere", "[prisms][inflation]") {
  auto tethreadron_points = make_tethreadron_points();
  Vector3dCloud shape_a(tethreadron_points);

  flx::shape::Sphere sphere_b(0.5f, hull::Coordinate{5.f, 0, 0});

  auto query = flx::get_closest_points_or_penetration_info(shape_a, sphere_b);

  CHECK(query.is_closest_pair_or_penetration_info);
  CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
  CHECK(almost_equal2(query.result.point_in_shape_b, 4.5f, 0));
}

TEST_CASE("Polygons and round inflation", "[prisms][inflation]") {
  auto tethreadron_points = make_tethreadron_points();
  Vector3dCloud shape_a(tethreadron_points);

  std::unique_ptr<Vector3dCloud> shape_b =
      std::make_unique<Vector3dCloud>(make_prism(
          std::vector<Point2D>{
              {1.f, 1.f}, {-1.f, 1.f}, {-1.f, -1.f}, {1.f, -1.f}},
          1.f));

  SECTION("Traslation") {
    flx::shape::Transformation trsf(hull::Coordinate{5.f, 0, 0});
    flx::shape::TransformDecorator shape_b_trsf(
        flx::shape::RoundDecorator{std::move(shape_b), 0.5f}, trsf);

    auto query =
        flx::get_closest_points_or_penetration_info(shape_a, shape_b_trsf);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 3.5f, 0));
  }

  SECTION("Roto traslation") {
    flx::shape::Transformation trsf(
        hull::Coordinate{5.f, 0, 0},
        flx::shape::RotationXYZ{0, 0.5f * 3.141f, 0});
    flx::shape::TransformDecorator shape_b_trsf(
        flx::shape::RoundDecorator{std::move(shape_b), 0.5f}, trsf);
    auto query =
        flx::get_closest_points_or_penetration_info(shape_a, shape_b_trsf);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 3.5f, 0));
  }
}
