#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "Utils.h"
#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

constexpr float GREEK_PI = 3.141f;

TEST_CASE("Triangles, vertex-vertex proximity with traslation",
          "[traslation][prisms]") {
  std::vector<flx::shape::Point2D> polygon_a = {
      {0.f, 0.f}, {-1.f, 1.f}, {-1.f, -1.f}};
  std::vector<flx::shape::Point2D> polygon_b = {
      {0.f, 0.f}, {0.f, 1.f}, {0.f, -1.f}};

  SECTION("Far away, horizontal traslation") {
    flx::shape::TransformDecorator shape_b(make_prism_cloud_ptr(polygon_b, 1),
                                           hull::Coordinate{2.f, 0, 0});

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1), shape_b);

    REQUIRE(query.is_closest_pair_or_penetration_info);
    CHECK(flx::almost_equal(flx::delta_squared_lenght(query.result), 2 * 2));
  }

  SECTION("Far away, crooked traslation") {
    flx::shape::TransformDecorator shape_b(
        make_prism_cloud_ptr(polygon_b, 1), hull::Coordinate{10.f, 0, 0},
        hull::Coordinate{0, 0, 45.f * GREEK_PI / 180.f});

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1), shape_b);

    REQUIRE(query.is_closest_pair_or_penetration_info);
    CHECK(flx::almost_equal(flx::delta_squared_lenght(query.result), 10 * 10));
  }

  SECTION("In collision") {
    flx::shape::TransformDecorator shape_b(make_prism_cloud_ptr(polygon_b, 1),
                                           hull::Coordinate{-0.3f, 0, 0});

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1), shape_b);

    REQUIRE(!query.is_closest_pair_or_penetration_info);
    CHECK(
        flx::almost_equal(flx::delta_squared_lenght(query.result), 0.3 * 0.3));
  }
}

TEST_CASE("Triangles, vertex-edge proximity", "[traslation][prisms]") {
  std::vector<flx::shape::Point2D> polygon_a = {
      {0.f, 0.f}, {-1.f, 1.f}, {-1.f, -1.f}};
  std::vector<flx::shape::Point2D> polygon_b = {
      {0.f, 0.f}, {0.f, 1.f}, {0.f, -1.f}};

  SECTION("Far away, crooked traslation") {
    flx::shape::TransformDecorator shape_b(
        make_prism_cloud_ptr(polygon_b, 1), hull::Coordinate{10.f, 0, 0},
        hull::Coordinate{0, 0, 45.f * GREEK_PI / 180.f});

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1), shape_b);

    REQUIRE(query.is_closest_pair_or_penetration_info);
    CHECK(flx::almost_equal(flx::delta_squared_lenght(query.result), 10 * 10));
  }
}
