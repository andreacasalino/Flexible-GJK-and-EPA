#pragma once

#include "../samples/Utils.h"
#include <Flexible-GJK-and-EPA/CoordinatePair.h>

#include <memory>
#include <vector>

struct Point2D {
  float x;
  float y;
};
Vector3dCloud make_prism_cloud(const std::vector<Point2D> &polygon,
                               const float height);

std::unique_ptr<Vector3dCloud>
make_prism_cloud_ptr(const std::vector<Point2D> &polygon, const float height);

float delta_squared_lenght(const flx::CoordinatePair &coordinates);

bool almost_equal(const float a, const float b);
