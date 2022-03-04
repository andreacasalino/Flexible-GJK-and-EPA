#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "Utils.h"
#include <Flexible-GJK-and-EPA/GjkEpa.h>

// #include <iostream>
// TEST_CASE("Debug", "[debug]") {
//   std::vector<Point2D> polygon_a = {
//       {-1.f, -2.f}, {1.f, -2.f}, {1.f, 3.f}, {-1.f, 3.f}};
//   std::vector<Point2D> polygon_b = {
//       {-1.f, 2.f}, {1.f, 2.f}, {1.f, 5.f}, {-1.f, 5.f}};

//   auto query = flx::get_closest_points_or_penetration_info(
//       make_prism_cloud(polygon_a, 1), make_prism_cloud(polygon_b, 1));
//   if (query.is_closest_pair_or_penetration_info) {
//     std::cout << "closest pair" << std::endl;
//   } else {
//     std::cout << "penetration info" << std::endl;
//   }
//   std::cout << query.result.point_in_shape_a.x << ' '
//             << query.result.point_in_shape_a.y << ' '
//             << query.result.point_in_shape_a.z << std::endl;
//   std::cout << query.result.point_in_shape_b.x << ' '
//             << query.result.point_in_shape_b.y << ' '
//             << query.result.point_in_shape_b.z << std::endl;
// }

TEST_CASE("Triangles, vertex-vertex proximity", "[prisms]") {
  std::vector<Point2D> polygon_a = {{0.f, 0.f}, {-1.f, 1.f}, {-1.f, -1.f}};

  SECTION("Far away") {
    std::vector<Point2D> polygon_b = {{2.f, 0.f}, {3.f, 1.f}, {3.f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        make_prism_cloud(polygon_a, 1), make_prism_cloud(polygon_b, 1));

    REQUIRE(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal(delta_squared_lenght(query.result), 2 * 2));
  }

  SECTION("In collision") {
    std::vector<Point2D> polygon_b = {
        {-0.3f, 0.f}, {-0.3f, 1.f}, {-0.3f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        make_prism_cloud(polygon_a, 1), make_prism_cloud(polygon_b, 1));

    REQUIRE(!query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal(delta_squared_lenght(query.result), 0.3 * 0.3));
  }

  SECTION("Just in touch") {
    std::vector<Point2D> polygon_b = {{0.f, 0.f}, {0.f, 1.f}, {0.f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        make_prism_cloud(polygon_a, 1), make_prism_cloud(polygon_b, 1));

    CHECK(almost_equal(delta_squared_lenght(query.result), 0));
  }
}

TEST_CASE("Triangles, vertex-edge proximity", "[prisms]") {
  std::vector<Point2D> polygon_a = {{0.f, 1.f}, {0.f, -1.f}, {-1.f, 0.f}};

  SECTION("Far away") {
    std::vector<Point2D> polygon_b = {{2.f, 0.f}, {3.f, 1.f}, {3.f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        make_prism_cloud(polygon_a, 1), make_prism_cloud(polygon_b, 1));

    REQUIRE(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal(delta_squared_lenght(query.result), 2 * 2));
  }

  SECTION("Just in touch") {
    std::vector<Point2D> polygon_b = {{0.f, 0.f}, {0.f, 1.f}, {0.f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        make_prism_cloud(polygon_a, 1), make_prism_cloud(polygon_b, 1));

    CHECK(almost_equal(delta_squared_lenght(query.result), 0));
  }
}
