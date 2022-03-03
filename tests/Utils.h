#pragma once

#include <Flexible-GJK-and-EPA/CoordinatePair.h>
#include <Flexible-GJK-and-EPA/shape/PointCloud.h>

#include <memory>
#include <vector>

namespace flx {
namespace shape {
using Points = std::vector<hull::Coordinate>;
using PointsIterator = Points::const_iterator;

float get_support(const PointsIterator &point,
                  const hull::Coordinate &direction);

hull::Coordinate convert(const PointsIterator &point);

class PointsStore {
public:
  PointsStore(Points &&buffer) : points(std::move(buffer)){};

  const Points &getPoints() const { return points; };

protected:
  const Points points;
};

class TestCloud : public PointsStore, public PointCloud<PointsIterator> {
public:
  TestCloud(Points &&buffer);
};

struct Point2D {
  float x;
  float y;
};
TestCloud make_prism_cloud(const std::vector<Point2D> &polygon,
                           const float height);

std::unique_ptr<ConvexShape>
make_prism_cloud_ptr(const std::vector<Point2D> &polygon, const float height);

TestCloud make_random_cloud(const std::size_t points);
} // namespace shape

float delta_squared_lenght(const CoordinatePair &coordinates);

bool almost_equal(const float a, const float b);
} // namespace flx
