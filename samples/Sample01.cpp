/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

#include "Logger.h"
#include "Utils.h"
#include <iostream>
using namespace std;

int main() {
  logger::Manager logger;

  {
    // get random shapes with few points, centred around the origin: a collision
    // detection is expected
    auto shapeA_points = make_random_cloud(50);
    auto shapeB_points = make_random_cloud(50);

    Vector3dCloud shapeA(shapeA_points);

    std::unique_ptr<Vector3dCloud> shapeB =
        std::make_unique<Vector3dCloud>(shapeB_points);

    {
      auto query_result =
          flx::get_closest_points_or_penetration_info(shapeA, *shapeB);
      // log results
      logger.logSingleQuery(shapeA, *shapeB, query_result, "Result_1_a.json");
    }

    {
      // translate one of the shape away: no collision is expected now
      flx::shape::TransformDecorator shapeB_translated(
          std::move(shapeB),
          flx::shape::Transformation{hull::Coordinate{3.f, 3.f, 3.f}});

      auto query_result = flx::get_closest_points_or_penetration_info(
          shapeA, shapeB_translated);
      // log results
      logger.logSingleQuery(shapeA, shapeB_translated, query_result,
                            "Result_1_b.json");
    }
  }

  // repeat above steps with bigger point clouds
  {
    // get random shapes with few points, centred around the origin: a collision
    // detection is expected
    auto shapeA_points = make_random_cloud(500);
    auto shapeB_points = make_random_cloud(500);

    Vector3dCloud shapeA(shapeA_points);

    std::unique_ptr<Vector3dCloud> shapeB =
        std::make_unique<Vector3dCloud>(shapeB_points);

    {
      auto query_result =
          flx::get_closest_points_or_penetration_info(shapeA, *shapeB);
      // log results
      logger.logSingleQuery(shapeA, *shapeB, query_result, "Result_1_c.json");
    }

    {
      // translate one of the shape away: no collision is expected now
      flx::shape::TransformDecorator shapeB_translated(
          std::move(shapeB),
          flx::shape::Transformation{hull::Coordinate{3.f, 3.f, 3.f}});

      auto query_result = flx::get_closest_points_or_penetration_info(
          shapeA, shapeB_translated);
      // log results
      logger.logSingleQuery(shapeA, shapeB_translated, query_result,
                            "Result_1_d.json");
    }
  }

  return EXIT_SUCCESS;
}
