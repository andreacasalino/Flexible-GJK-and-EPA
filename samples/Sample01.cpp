/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

#include <Utils.h>

#include <iostream>
using namespace std;

using namespace flx::utils;

int main() {
  {
    // get a random point clouds, both centred around the origin: a
    // collision detection is expected among the 2
    Vector3dCloud shapeA(make_random_cloud(50));
    auto shapeB_points = make_random_cloud(50);

    {
      Vector3dCloud shapeB(shapeB_points);
      auto query_result =
          flx::get_closest_points_or_penetration_info(shapeA, shapeB);
      // log results

      ShapesLog::logSample("Result_1_a", shapeA, shapeB, query_result);
    }

    {
      // translate one of the shape away: no collision is expected now
      flx::shape::TransformDecorator shapeB_translated(
          std::make_unique<Vector3dCloud>(shapeB_points),
          flx::shape::TransformationBuilder{}.setTraslation(
              hull::Coordinate{3.f, 3.f, 3.f}));

      auto query_result = flx::get_closest_points_or_penetration_info(
          shapeA, shapeB_translated);
      // log results
      ShapesLog::logSample("Result_1_b", shapeA, shapeB_translated,
                           query_result);
    }
  }

  // repeat above steps with a point cloud with more points
  {
    Vector3dCloud shapeA(make_random_cloud(500));
    auto shapeB_points = make_random_cloud(500);

    {
      Vector3dCloud shapeB(shapeB_points);
      auto query_result =
          flx::get_closest_points_or_penetration_info(shapeA, shapeB);
      // log results
      ShapesLog::logSample("Result_1_c", shapeA, shapeB, query_result);
    }

    {
      // translate one of the shape away: no collision is expected now
      flx::shape::TransformDecorator shapeB_translated(
          std::make_unique<Vector3dCloud>(shapeB_points),
          flx::shape::TransformationBuilder{}.setTraslation(
              hull::Coordinate{3.f, 3.f, 3.f}));

      auto query_result = flx::get_closest_points_or_penetration_info(
          shapeA, shapeB_translated);
      // log results
      ShapesLog::logSample("Result_1_d", shapeA, shapeB_translated,
                           query_result);
    }
  }
  return EXIT_SUCCESS;
}
