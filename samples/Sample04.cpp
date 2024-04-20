/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <Flexible-GJK-and-EPA/shape/RoundDecorator.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>

#include <Utils.h>

#include <iostream>
using namespace std;

using namespace flx::utils;

int main() {
  {
    // get random shapes with few points
    auto shapeA_points = make_random_cloud(6);
    auto shapeB_points = make_random_cloud(6);
    const float ray = 1.f;

    // build inflated shapes
    std::unique_ptr<flx::shape::ConvexShape> shapeA =
        std::make_unique<flx::shape::RoundDecorator>(
            std::make_unique<Vector3dCloud>(shapeA_points), ray);

    std::unique_ptr<flx::shape::ConvexShape> shapeB =
        std::make_unique<flx::shape::RoundDecorator>(
            std::make_unique<Vector3dCloud>(shapeB_points), ray);

    // traslateaway the second shape
    shapeB = std::make_unique<flx::shape::TransformDecorator>(
        std::move(shapeB), flx::shape::TransformationBuilder{}.setTraslation(
                               hull::Coordinate{3.f, 3.f, 3.f}));

    auto query_result =
        flx::get_closest_points_or_penetration_info(*shapeA, *shapeB);
    // log results
    ShapesLog::logSample("Result_4", *shapeA, *shapeB, query_result);
  }
  return EXIT_SUCCESS;
}
