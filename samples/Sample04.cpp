/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

#include "Logger.h"
#include "Utils.h"
#include <iostream>
using namespace std;

int main() {
  {
    // get random shapes with few points
    auto shapeA_points = make_random_cloud(20);
    auto shapeB_points = make_random_cloud(20);
    const float ray = 0.2f;

    // build inflated shapes
    std::unique_ptr<flx::shape::ConvexShape> shapeA =
        std::make_unique<flx::shape::RoundDecorator>(
            std::make_unique<Vector3dCloud>(shapeA_points), ray);

    std::unique_ptr<flx::shape::ConvexShape> shapeB =
        std::make_unique<flx::shape::RoundDecorator>(
            std::make_unique<Vector3dCloud>(shapeB_points), ray);
    // traslateaway the second shape
    shapeB = std::make_unique<flx::shape::TransformDecorator>(
        std::move(shapeB),
        flx::shape::Transformation{hull::Coordinate{3.f, 3.f, 3.f}});

    auto query_result =
        flx::get_closest_points_or_penetration_info(*shapeA, *shapeB);
    // log results
    logger::logSingleQuery(*shapeA, *shapeB, query_result, "Result_4.json");
  }
  return EXIT_SUCCESS;
}
