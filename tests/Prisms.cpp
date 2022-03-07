#include <catch2/catch_test_macros.hpp>

#include "Utils.h"

TEST_CASE("Polygons, vertex-vertex proximity", "[prisms][polygons]") {
  SECTION("Far away") {
    auto polygon_a = make_prism(
        std::vector<Point2D>{{0.f, 0.f}, {-1.f, 1.f}, {-1.f, -1.f}}, 1.f);

    auto polygon_b = make_prism(
        std::vector<Point2D>{{2.f, 0.f}, {3.f, 1.f}, {3.f, -1.f}}, 1.f);

    auto query = make_test_query(polygon_a, polygon_b);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 2.f, 0));
  }

  SECTION("Just in touch") {
    auto polygon_a = make_prism(
        std::vector<Point2D>{{0.f, 0.f}, {-1.f, 1.f}, {-1.f, -1.f}}, 1.f);

    auto polygon_b = make_prism(
        std::vector<Point2D>{{0.f, 0.f}, {3.f, 1.f}, {3.f, -1.f}}, 1.f);

    auto query = make_test_query(polygon_a, polygon_b);

    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 0, 0));
  }
}

TEST_CASE("Polygons, vertex-edge proximity", "[prisms][polygons]") {
  SECTION("Far away") {
    auto polygon_a = make_prism(
        std::vector<Point2D>{{0.f, 0.f}, {-1.f, 1.f}, {-1.f, -1.f}}, 1.f);

    auto polygon_b = make_prism(
        std::vector<Point2D>{{2.f, 1.f}, {2.f, -1.f}, {3.f, 0.f}}, 1.f);

    auto query = make_test_query(polygon_a, polygon_b);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 2.f, 0));
  }

  SECTION("Just in touch") {
    auto polygon_a = make_prism(
        std::vector<Point2D>{{0.f, 0.f}, {-1.f, 1.f}, {-1.f, -1.f}}, 1.f);

    auto polygon_b =
        make_prism(std::vector<Point2D>{{0, 1.f}, {0, -1.f}, {3.f, 0.f}}, 1.f);

    auto query = make_test_query(polygon_a, polygon_b);

    CHECK(almost_equal2(query.result.point_in_shape_a, 0, 0));
    CHECK(almost_equal2(query.result.point_in_shape_b, 0, 0));
  }
}

TEST_CASE("Polygons, edge-edge proximity", "[prisms][polygons]") {
  SECTION("Far away") {
    auto polygon_a = make_prism(
        std::vector<Point2D>{
            {0.f, 1.f}, {0.f, -1.f}, {-1.f, 1.f}, {-1.f, -1.f}},
        1.f);

    auto polygon_b = make_prism(
        std::vector<Point2D>{{2.f, 1.f}, {2.f, -1.f}, {3.f, 1.f}, {3.f, -1.f}},
        1.f);

    auto query = make_test_query(polygon_a, polygon_b);

    CHECK(query.is_closest_pair_or_penetration_info);
    CHECK(almost_equal(query.result.point_in_shape_a.x, 0));
    CHECK(almost_equal(query.result.point_in_shape_b.x, 2.f));
  }

  SECTION("In collision") {
    auto polygon_a = make_prism(
        std::vector<Point2D>{
            {0.f, 1.f}, {0.f, -1.f}, {-1.f, 1.f}, {-1.f, -1.f}},
        1.f);

    auto polygon_b = make_prism(
        std::vector<Point2D>{
            {-0.5f, 1.f}, {-0.5f, -1.f}, {1.f, 1.f}, {1.f, -1.f}},
        1.f);

    auto query = make_test_query(polygon_a, polygon_b);

    CHECK_FALSE(query.is_closest_pair_or_penetration_info);
    hull::Coordinate penetration_vector;
    hull::diff(penetration_vector, query.result.point_in_shape_a,
               query.result.point_in_shape_b);
    CHECK(almost_equal(hull::normSquared(penetration_vector), 0.5 * 0.5));
    CHECK(almost_equal(query.result.point_in_shape_a.x, 0));
    CHECK(almost_equal(query.result.point_in_shape_b.x, -0.5));
  }
}
