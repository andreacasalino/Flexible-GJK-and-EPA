/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Utils.h"
#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <Flexible-GJK-and-EPA/shape/ConvexCloud.h>
#include <Flexible-GJK-and-EPA/shape/TransformDecorator.h>
#include <iostream>
using namespace std;

typedef std::unique_ptr<flx::shape::ConvexShape> ShapePtr;

// Add to the list of politopes a random poligon, with a random position and
// orientation
void addRandomShape(std::list<ShapePtr> &poligons) {
  auto randUnif = [](const float &min_val, const float &max_val) {
    float delta = max_val - min_val;
    return min_val +
           delta * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  };

  poligons.emplace_back(std::make_unique<flx::shape::ConvexCloud<list<Vector>>>(
      Vector::getRandomCloud(rand() % 9 + 5)));

  hull::Coordinate rot = {randUnif(-3.14159f, 3.14159f),
                          randUnif(-3.14159f, 3.14159f),
                          randUnif(-3.14159f, 3.14159f)};
  hull::Coordinate trasl = {randUnif(-5.f, 5.f), randUnif(-5.f, 5.f),
                            randUnif(-5.f, 5.f)};

  poligons.back() = std::make_unique<flx::shape::TransformDecorator>(
      std::move(poligons.back()), trasl, rot);
}

// see Sample_01 and Sample_02 before this one
int main() {
  std::size_t N_shapes = 5;
  list<ShapePtr> shapes;

  // build the solver to use for the subsequent queries
  flx::GjkEpa solver;
  SampleLogger result("Result_3.json");

  // sample N_shapes politopes
  for (std::size_t k = 0; k < N_shapes; ++k) {
    addRandomShape(shapes);
    result.addShape(*shapes.back());
  }

  // check all the possible pairs
  auto it2 = shapes.begin();
  flx::GjkEpa::CoordinatePair resultVector;
  for (auto it = shapes.begin(); it != shapes.end(); it++) {
    it2 = it;
    ++it2;
    for (it2; it2 != shapes.end(); ++it2) {
      flx::GjkEpa::ResultType res =
          solver.doComplexQuery({**it, **it2}, resultVector
#ifdef FLX_LOGGER_ENABLED
                                ,
                                ""
#endif
          );
      result.addQueryResult(resultVector);
    }
  }

  // analyze the .json(s) storing the results using the python script
  // Visualize03.py
  std::cout << "Use Visualize03.py to see the results" << std::endl;

  return EXIT_SUCCESS;
}