#pragma once

#include "../samples/Utils.h"
#include <Flexible-GJK-and-EPA/CoordinatePair.h>
#include <Flexible-GJK-and-EPA/GjkEpa.h>

#include <memory>
#include <vector>

flx::QueryResult make_test_query(const std::vector<Vector3d> &a,
                                 const std::vector<Vector3d> &b);

struct Point2D {
  float x;
  float y;
};
std::vector<Vector3d> make_prism(const std::vector<Point2D> &polygon,
                                 const float height);

class Vector3dCloudTest : public flx::shape::ConvexShape {
public:
  Vector3dCloudTest(const std::vector<Vector3d> &points) {
    cloud = std::make_shared<std::vector<Vector3d>>(points);
    shape = std::make_unique<
        flx::shape::PointCloud<std::vector<Vector3d>::const_iterator>>(
        cloud->begin(), cloud->end(), dot_product, to_coordinate);
  };

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const final {
    shape->getSupport(support, direction);
  };

private:
  std::shared_ptr<std::vector<Vector3d>> cloud;
  std::unique_ptr<flx::shape::PointCloud<std::vector<Vector3d>::const_iterator>>
      shape;
};

std::unique_ptr<Vector3dCloudTest>
make_cloud_test(const std::vector<Vector3d> &points);

float delta_squared_lenght(const flx::CoordinatePair &coordinates);

bool almost_equal(const float a, const float b);

bool almost_equal(const hull::Coordinate &a, const hull::Coordinate &b);

bool almost_equal2(const hull::Coordinate &a, const float expected_x,
                   const float expected_y);
