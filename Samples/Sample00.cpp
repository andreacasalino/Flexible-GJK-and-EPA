/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <shape/ConvexCloud.h>
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
    std::unique_ptr<shape::ConvexShape> shapeA = std::make_unique<shape::ConvexCloud<list<Vector>>>(Vector::getRandomCloud(10));    
    std::unique_ptr<shape::ConvexShape> shapeB = std::make_unique<shape::ConvexCloud<list<Vector>>>(Vector::getRandomCloud(10));
    
    shapeB = std::make_unique<shape::TransformDecorator>(std::move(shapeB));
    static_cast<shape::TransformDecorator*>(shapeB.get())->setTraslation({3.f, 3.f, 3.f});

    GjkEpa solver;

    GjkEpa::CoordinatePair result;
    solver.doComplexQuery({*shapeA, *shapeB}, result, "Iterations.txt");

    return EXIT_SUCCESS;
}