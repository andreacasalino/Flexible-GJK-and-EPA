#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <math.h>

#include "Utils.h"

float to_rad(const float angle) { return 3.14159f * angle / 180.f; }

std::vector<float> get_angles(const std::size_t size) {
  std::vector<float> angles;
  angles.reserve(size);
  float delta_angle = to_rad(360.f) / static_cast<float>(size);
  float angle = 0.f;
  for (std::size_t v = 0; v < size; ++v) {
    angles.emplace_back(angle);
    angle += delta_angle;
  }
  return angles;
}

std::vector<Vector3d> get_polygon(const std::size_t size) {
  std::vector<Vector3d> vertices;
  vertices.reserve(size * 2);
  for (const auto &angle : get_angles(size)) {
    vertices.emplace_back(cosf(angle), sinf(angle), 0.25f);
    vertices.emplace_back(cosf(angle), sinf(angle), -0.25f);
  }
  return vertices;
}

TEST_CASE("Polygon", "[support]") {
  auto polygon_size = GENERATE(3, 5, 9);

  Vector3dCloud polygon(get_polygon(polygon_size));

  for (const auto &angle : get_angles(polygon_size)) {
    const float angle_cos = cosf(angle);
    const float angle_sin = sinf(angle);
    hull::Coordinate support;
    polygon.getSupport(support, hull::Coordinate{angle_cos, angle_sin, 0});
    CHECK(almost_equal(angle_cos, support.x));
    CHECK(almost_equal(angle_sin, support.y));
  }
}

#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>
TEST_CASE("Transform decorator", "[support]") {
  const std::size_t polygon_size = 5;
  Vector3dCloud polygon(get_polygon(polygon_size));

  SECTION("Traslations") {
    auto traslation = GENERATE(
        hull::Coordinate{2.f, 0, 0}, hull::Coordinate{0, 2.f, 0},
        hull::Coordinate{0, 2.f, 0}, hull::Coordinate{0.f, 2.f, 0},
        hull::Coordinate{-2.f, 3.f, 0}, hull::Coordinate{2.f, -3.f, 0});

    auto polygon_points = polygon.getPoints();
    flx::shape::TransformDecorator polygon_translated(
        std::make_unique<Vector3dCloud>(std::move(polygon_points)),
        flx::shape::Transformation{traslation});

    for (const auto &angle : get_angles(polygon_size)) {
      hull::Coordinate direction =
          hull::Coordinate{cosf(angle), sinf(angle), 0};

      hull::Coordinate polygon_support;
      polygon.getSupport(polygon_support, direction);

      hull::Coordinate polygon_traslated_support;
      polygon_translated.getSupport(polygon_support, direction);

      CHECK(almost_equal(polygon_traslated_support.x,
                         polygon_support.x + traslation.x));
      CHECK(almost_equal(polygon_traslated_support.y,
                         polygon_support.y + traslation.y));
    }
  }

  SECTION("Rotations") {
    throw 0; // TODO
  }

  SECTION("Traslations and rotations") {
    throw 0; // TODO
  }
}

#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>
TEST_CASE("Round decorator", "[support]") {
  auto polygon_size = GENERATE(3, 5, 9);

  Vector3dCloud polygon(get_polygon(polygon_size));

  const float ray = 1.5f;
  auto polygon_points = polygon.getPoints();
  flx::shape::RoundDecorator polygon_inflated(
      std::make_unique<Vector3dCloud>(std::move(polygon_points)), ray);

  for (const auto &angle : get_angles(polygon_size)) {
    hull::Coordinate direction = hull::Coordinate{cosf(angle), sinf(angle), 0};

    hull::Coordinate polygon_support;
    polygon.getSupport(polygon_support, direction);

    hull::Coordinate polygon_inflated_support;
    polygon_inflated.getSupport(polygon_support, direction);
    CHECK(almost_equal(polygon_inflated_support.x, polygon_support.x + ray));
    CHECK(almost_equal(polygon_inflated_support.y, polygon_support.y + ray));
  }
}
