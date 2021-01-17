/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <GjkEpa.h>
#include <shape/ConvexCloud.h>
#include <shape/TransformDecorator.h>
#include "Utils.h"
using namespace std;
using namespace flx;

int main() {
    GjkEpa solver;
    GjkEpa::CoordinatePair result;

    std::unique_ptr<shape::ConvexShape> shapeA = std::make_unique<shape::ConvexCloud<list<Vector>>>(Vector::getRandomCloud(10));    
    std::unique_ptr<shape::ConvexShape> shapeB = std::make_unique<shape::ConvexCloud<list<Vector>>>(Vector::getRandomCloud(10));

    // solver.doComplexQuery({*shapeA, *shapeB}, result, "LogPenetration.json");

    shapeB = std::make_unique<shape::TransformDecorator>(std::move(shapeB));
    static_cast<shape::TransformDecorator*>(shapeB.get())->setTraslation({3.f, 3.f, 3.f});

    solver.doComplexQuery({*shapeA, *shapeB}, result, "LogClosest.json");

    return EXIT_SUCCESS;
}