#pragma once

#include "../samples/Utils.h"
#include <Flexible-GJK-and-EPA/CoordinatePair.h>
#include <Flexible-GJK-and-EPA/GjkEpa.h>

#include <memory>
#include <vector>

flx::QueryResult make_test_query(const Points &a, const Points &b);

struct Point2D {
  float x;
  float y;
};
Points make_prism(const std::vector<Point2D> &polygon, const float height);

float delta_squared_lenght(const flx::CoordinatePair &coordinates);

bool almost_equal(const float a, const float b);

bool almost_equal(const hull::Coordinate &a, const hull::Coordinate &b);

bool almost_equal2(const hull::Coordinate &a, const float expected_x,
                   const float expected_y);
