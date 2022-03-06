#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <math.h>

#include "Utils.h"

float to_rad(const float angle) { return 3.14159f * angle / 180.f; }

std::vector<float> make_angles(const std::size_t size) {
  std::vector<float> angles;
  angles.reserve(size);
  float delta_angle = to_rad(360.f) / static_cast<float>(size);
  float angle = 0.f;
  for (std::size_t v = 0; v < size; ++v) {
    angles.push_back(angle);
    angle += delta_angle;
  }
  return angles;
}

std::vector<Point2D> make_polygon(const std::size_t size) {
  std::vector<Point2D> vertices;
  vertices.reserve(size);
  for (const auto &angle : make_angles(size)) {
    vertices.push_back(Point2D{cosf(angle), sinf(angle)});
  }
  return vertices;
}

TEST_CASE("Polygon", "[support]") {
  auto polygon_size = GENERATE(3, 5, 9);

  Vector3dCloudTest polygon(make_prism(make_polygon(polygon_size), 0.5f));

  hull::Coordinate support;
  for (const auto &angle : make_angles(polygon_size)) {
    const float angle_cos = cosf(angle);
    const float angle_sin = sinf(angle);
    polygon.getSupport(support, hull::Coordinate{angle_cos, angle_sin, 0});
    CHECK(almost_equal(angle_cos, support.x));
    CHECK(almost_equal(angle_sin, support.y));
  }
}

#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>
TEST_CASE("Transform decorator", "[support]") {
  auto polygon_size = GENERATE(3, 5, 9);

  float delta_x = 1.5f;
  float delta_y = -0.5f;
  float delta_rot_z = to_rad(25);

  flx::shape::Transformation transform(
      hull::Coordinate{delta_x, delta_y, 0},
      flx::shape::RotationXYZ{0, 0, delta_rot_z});

  flx::shape::TransformDecorator polygon_transformed(
      make_cloud_test(make_prism(make_polygon(polygon_size), 0.5f)), transform);

  hull::Coordinate support;
  for (const auto &angle : make_angles(polygon_size)) {
    const float angle_cos = cosf(angle + delta_rot_z);
    const float angle_sin = sinf(angle + delta_rot_z);
    polygon_transformed.getSupport(support,
                                   hull::Coordinate{angle_cos, angle_sin, 0});
    CHECK(almost_equal(angle_cos + delta_x, support.x));
    CHECK(almost_equal(angle_sin + delta_y, support.y));
  }
}

#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>
TEST_CASE("Round decorator", "[support]") {
  auto polygon_size = GENERATE(3, 5, 9);

  float ray = 2.5f;

  flx::shape::RoundDecorator polygon_inflated(
      make_cloud_test(make_prism(make_polygon(polygon_size), 0.5f)), ray);

  hull::Coordinate support;
  for (const auto &angle : make_angles(polygon_size)) {
    const float angle_cos = cosf(angle);
    const float angle_sin = sinf(angle);
    polygon_inflated.getSupport(support,
                                hull::Coordinate{angle_cos, angle_sin, 0});
    CHECK(almost_equal(angle_cos * (1.f + ray), support.x));
    CHECK(almost_equal(angle_sin * (1.f + ray), support.y));
  }
}
