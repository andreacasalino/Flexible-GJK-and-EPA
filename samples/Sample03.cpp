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

using Points = std::shared_ptr<std::vector<Vector3d>>;
using ShapePtr = std::unique_ptr<flx::shape::ConvexShape>;

struct PointsAndShape {
  Points points;
  ShapePtr shape;
};
// Get a random poligon, shifted with a random position and orientation
PointsAndShape makeRandomShape() {
  auto randUnif = [](const float &min_val, const float &max_val) {
    float delta = max_val - min_val;
    return min_val +
           delta * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  };

  hull::Coordinate trasl = {randUnif(-5.f, 5.f), randUnif(-5.f, 5.f),
                            randUnif(-5.f, 5.f)};
  flx::shape::RotationXYZ rot = {randUnif(-3.14159f, 3.14159f),
                                 randUnif(-3.14159f, 3.14159f),
                                 randUnif(-3.14159f, 3.14159f)};

  Points points = std::make_shared<std::vector<Vector3d>>(
      make_random_cloud(rand() % 9 + 5));
  ShapePtr new_shape = std::make_unique<Vector3dCloud>(*points);
  return PointsAndShape{
      points,
      std::make_unique<flx::shape::TransformDecorator>(
          std::move(new_shape), flx::shape::Transformation{trasl, rot})};
}

// see Sample_01 and Sample_02 before this one
int main() {
  logger::Manager logger;

  std::size_t N_shapes = 5;
  vector<PointsAndShape> shapes;
  shapes.reserve(N_shapes);
  // get some random shapes in space
  for (std::size_t k = 0; k < N_shapes; ++k) {
    shapes.emplace_back(makeRandomShape());
  }

  auto logger_figure = logger.makeFigure();
  auto &logger_plot = logger_figure.addSubPlot("Random shapes queries");

  // check all the possible pair of shapes
  for (std::size_t r = 0; r < shapes.size(); ++r) {
    for (std::size_t c = r + 1; c < shapes.size(); ++c) {
      const auto &shape_A = *shapes[r].shape;
      const auto &shape_B = *shapes[c].shape;
      auto query_result =
          flx::get_closest_points_or_penetration_info(shape_A, shape_B);
      // log results
      logger_plot.addShape(shape_A);
      logger_plot.addShape(shape_B);
      logger_plot.addLine(query_result.result.point_in_shape_a,
                          query_result.result.point_in_shape_b);
    }
  }
  // Result_3
  logger_figure.log("Result_3.json");

  return EXIT_SUCCESS;
}