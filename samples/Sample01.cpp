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

int main() {
  // build the solver to use for the subsequent queries
  flx::GjkEpa solver;

  // get random shapes with few points, centred around the origin: a collision
  // detection is expected
  std::unique_ptr<flx::shape::ConvexShape> shapeA =
      std::make_unique<flx::shape::ConvexCloud<list<Vector>>>(
          Vector::getRandomCloud(50));
  std::unique_ptr<flx::shape::ConvexShape> shapeB =
      std::make_unique<flx::shape::ConvexCloud<list<Vector>>>(
          Vector::getRandomCloud(50));
  {
    SampleLogger result("Result_1_a.json");
    result.doComplexQuery(solver, {*shapeA, *shapeB});
  }

  // translate one of the shape away: no collision is expected anymore
  shapeB = std::make_unique<flx::shape::TransformDecorator>(std::move(shapeB));
  static_cast<flx::shape::TransformDecorator *>(shapeB.get())
      ->setTraslation({3.f, 3.f, 3.f});
  {
    SampleLogger result("Result_1_b.json");
    result.doComplexQuery(solver, {*shapeA, *shapeB});
  }

  // repeat above steps with bigger point clouds
  shapeA = std::make_unique<flx::shape::ConvexCloud<list<Vector>>>(
      Vector::getRandomCloud(500));
  shapeB = std::make_unique<flx::shape::ConvexCloud<list<Vector>>>(
      Vector::getRandomCloud(500));
  {
    SampleLogger result("Result_1_c.json");
    result.doComplexQuery(solver, {*shapeA, *shapeB});
  }

  shapeB = std::make_unique<flx::shape::TransformDecorator>(std::move(shapeB));
  static_cast<flx::shape::TransformDecorator *>(shapeB.get())
      ->setTraslation({3.f, 3.f, 3.f});
  {
    SampleLogger result("Result_1_d.json");
    result.doComplexQuery(solver, {*shapeA, *shapeB});
  }

  // analyze the .json(s) storing the results using the python script
  // Visualize01.py
  std::cout << "Use Visualize01.py to see the results" << std::endl;

  return EXIT_SUCCESS;
}