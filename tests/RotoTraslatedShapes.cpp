#include <catch2/catch_test_macros.hpp>

#include "Utils.h"
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

TEST_CASE("Polygons and transformations", "[prisms][transform]") {
  std::vector<Vector3d> tethraedron_a =
      std::vector<Vector3d>{Vector3d{0, 0, 0}, Vector3d{-1.f, -1.f, 0},
                            Vector3d{-1.f, 1.f, 0}, Vector3d{-1.f, 0, 1.f}};
  Vector3dCloud shape_a(tethraedron_a.begin(), tethraedron_a.end(), dot_product,
                        to_coordinate);

  std::vector<Vector3d> square_b = make_prism(
      std::vector<Point2D>{{1.f, 1.f}, {-1.f, 1.f}, {-1.f, -1.f}, {1.f, -1.f}},
      1.f);
  std::unique_ptr<Vector3dCloud> shape_b = std::make_unique<Vector3dCloud>(
      square_b.begin(), square_b.end(), dot_product, to_coordinate);

  SECTION("Traslation") {
    flx::shape::Transformation trsf(hull::Coordinate{5.f, 0, 0});
    flx::shape::TransformDecorator shape_b_trsf(std::move(shape_b), trsf);

    auto query =
        flx::get_closest_points_or_penetration_info(shape_a, shape_b_trsf);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 4.f, 0));
  }

  SECTION("Roto traslation") {
    flx::shape::Transformation trsf(
        hull::Coordinate{5.f, 0, 0},
        flx::shape::RotationXYZ{0, 0.5f * 3.141f, 0});
    flx::shape::TransformDecorator shape_b_trsf(std::move(shape_b), trsf);

    auto query =
        flx::get_closest_points_or_penetration_info(shape_a, shape_b_trsf);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 4.f, 0));
  }
}
