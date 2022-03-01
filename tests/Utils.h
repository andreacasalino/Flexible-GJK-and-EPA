#pragma once

#include <Flexible-GJK-and-EPA/CoordinatePair.h>
#include <Flexible-GJK-and-EPA/shape/PointCloud.h>

#include <memory>
#include <vector>

namespace flx {
namespace shape {
using PointsIterator = std::vector<hull::Coordinate>::const_iterator;
using Points = std::shared_ptr<const std::vector<hull::Coordinate>>;

float get_support(const PointsIterator &point,
                  const hull::Coordinate &direction);

hull::Coordinate convert(const PointsIterator &point);

class TestCloud : public PointCloud<PointsIterator> {
public:
  TestCloud(const Points &points);

private:
  Points points;
};

struct Point2D {
  float x;
  float y;
};
TestCloud make_prism_cloud(const std::vector<Point2D> &polygon,
                           const float height);

TestCloud make_random_cloud(const std::size_t points);
} // namespace shape

float delta_squared_lenght(const CoordinatePair &coordinates);

bool almost_equal(const float a, const float b);
} // namespace flx
