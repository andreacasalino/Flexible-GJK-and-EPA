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

using ShapePtr = std::unique_ptr<flx::shape::ConvexShape>;

// Get a random poligon, shifted with a random position and orientation
ShapePtr makeRandomShape() {
  auto randUnif = [](const float &min_val, const float &max_val) {
    float delta = max_val - min_val;
    return min_val +
           delta * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  };

  hull::Coordinate trasl = {randUnif(-7.f, 7.f), randUnif(-7.f, 7.f),
                            randUnif(-7.f, 7.f)};
  flx::shape::RotationXYZ rot = {randUnif(0, 3.14159f), randUnif(0, 3.14159f),
                                 randUnif(0, 3.14159f)};

  ShapePtr shape =
      std::make_unique<Vector3dCloud>(make_random_cloud(rand() % 9 + 5));
  return std::make_unique<flx::shape::TransformDecorator>(
      std::move(shape),
      flx::shape::TransformationBuilder{}.setTraslation(trasl).setRotationXYZ(
          rot));
}

// see Sample_01 and Sample_02 before this one
int main() {
  ShapesLog logger("Result_3");

  std::size_t N_shapes = 5;
  vector<ShapePtr> shapes;
  shapes.reserve(N_shapes);
  // get some random shapes in space
  for (std::size_t k = 0; k < N_shapes; ++k) {
    auto &added = shapes.emplace_back(makeRandomShape());
    logger.add(*added);
  }

  // compute the closest points in all the possible pair of shapes
  for (std::size_t r = 0; r < shapes.size(); ++r) {
    for (std::size_t c = r + 1; c < shapes.size(); ++c) {
      const auto &shape_A = *shapes[r];
      const auto &shape_B = *shapes[c];
      auto query_result = flx::get_closest_points(shape_A, shape_B);
      // log results
      if (query_result != std::nullopt) {
        logger.add(query_result.value(), false);
      }
    }
  }

  return EXIT_SUCCESS;
}