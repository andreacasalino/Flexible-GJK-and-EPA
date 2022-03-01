#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "Utils.h"
#include <Flexible-GJK-and-EPA/GjkEpa.h>

TEST_CASE("Triangles, vertex-vertex proximity", "[prisms]") {
  std::vector<flx::shape::Point2D> polygon_a = {
      {0.f, 0.f}, {-1.f, 1.f}, {-1.f, -1.f}};

  SECTION("Far away") {
    std::vector<flx::shape::Point2D> polygon_b = {
        {2.f, 0.f}, {3.f, 1.f}, {3.f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1),
        flx::shape::make_prism_cloud(polygon_b, 1));

    REQUIRE(query.is_closest_pair_or_penetration_info);
    CHECK(flx::almost_equal(flx::delta_squared_lenght(query.result), 2 * 2));
  }

  SECTION("In collision") {
    std::vector<flx::shape::Point2D> polygon_b = {
        {-0.3f, 0.f}, {-0.3f, 1.f}, {-0.3f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1),
        flx::shape::make_prism_cloud(polygon_b, 1));

    REQUIRE(!query.is_closest_pair_or_penetration_info);
    CHECK(
        flx::almost_equal(flx::delta_squared_lenght(query.result), 0.3 * 0.3));
  }

  SECTION("Just in touch") {
    std::vector<flx::shape::Point2D> polygon_b = {
        {0.f, 0.f}, {0.f, 1.f}, {0.f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1),
        flx::shape::make_prism_cloud(polygon_b, 1));

    CHECK(flx::almost_equal(flx::delta_squared_lenght(query.result), 0));
  }
}

TEST_CASE("Triangles, vertex-edge proximity", "[prisms]") {
  std::vector<flx::shape::Point2D> polygon_a = {
      {0.f, 1.f}, {0.f, -1.f}, {-1.f, 0.f}};

  SECTION("Far away") {
    std::vector<flx::shape::Point2D> polygon_b = {
        {2.f, 0.f}, {3.f, 1.f}, {3.f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1),
        flx::shape::make_prism_cloud(polygon_b, 1));

    REQUIRE(query.is_closest_pair_or_penetration_info);
    CHECK(flx::almost_equal(flx::delta_squared_lenght(query.result), 2 * 2));
  }

  SECTION("Just in touch") {
    std::vector<flx::shape::Point2D> polygon_b = {
        {0.f, 0.f}, {0.f, 1.f}, {0.f, -1.f}};

    auto query = flx::get_closest_points_or_penetration_info(
        flx::shape::make_prism_cloud(polygon_a, 1),
        flx::shape::make_prism_cloud(polygon_b, 1));

    CHECK(flx::almost_equal(flx::delta_squared_lenght(query.result), 0));
  }
}
