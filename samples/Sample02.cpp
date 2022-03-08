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

flx::shape::Transformation getRandomTransformation(const float &min_val,
                                                   const float &max_val) {
  auto randUnif = [](const float &min_val, const float &max_val) {
    float delta = max_val - min_val;
    return min_val +
           delta * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  };

  return flx::shape::Transformation{
      hull::Coordinate{randUnif(min_val, max_val), randUnif(min_val, max_val),
                       randUnif(min_val, max_val)},
      flx::shape::RotationXYZ{randUnif(0.f, 0.5f * 3.14159f),
                              randUnif(0.f, 0.5f * 3.14159f),
                              randUnif(0.f, 0.5f * 3.14159f)}};
}

std::shared_ptr<std::vector<Vector3d>> getPoligon(const std::size_t N_edges) {
  std::vector<Vector3d> cloud;
  float delta_angle = 2.f * 3.14159f / static_cast<float>(N_edges);
  float angle = 0.f;
  float temp[2];
  cloud.reserve(N_edges * 2);
  for (std::size_t k = 0; k < N_edges; k++) {
    temp[0] = cosf(angle);
    temp[1] = sinf(angle);
    cloud.emplace_back(temp[0], temp[1], 0.2f);
    cloud.emplace_back(temp[0], temp[1], -0.2f);
    angle += delta_angle;
  }
  return std::make_shared<std::vector<Vector3d>>(cloud);
}

int main() {
  // build two poligons with a specified number of vertices
  auto shapeA_points = getPoligon(6);
  auto shapeB_points = getPoligon(3);

  // a collision is expected: get penetration depth
  {
    Vector3dCloud shapeA(shapeA_points);
    Vector3dCloud shapeB(shapeB_points);
    auto query_result =
        flx::get_closest_points_or_penetration_info(shapeA, shapeB);
    // log results
    logger::logSingleQuery(shapeA, shapeB, query_result, "Result_2_a.json");
  }

  // get the closest points
  {
    // get two random rototraslations for both the shapes
    flx::shape::TransformDecorator shapeA_translated(
        std::make_unique<Vector3dCloud>(shapeA_points),
        flx::shape::Transformation{getRandomTransformation(1.f, 2.f)});
    flx::shape::TransformDecorator shapeB_translated(
        std::make_unique<Vector3dCloud>(shapeB_points),
        flx::shape::Transformation{getRandomTransformation(-2.f, -1.f)});

    auto query_result = flx::get_closest_points_or_penetration_info(
        shapeA_translated, shapeB_translated);
    // log results
    logger::logSingleQuery(shapeA_translated, shapeB_translated, query_result,
                           "Result_2_b.json");
  }

  return EXIT_SUCCESS;
}