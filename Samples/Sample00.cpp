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

    std::unique_ptr<shape::ConvexShape> shapeA = std::make_unique<shape::ConvexCloud<list<Vector>>>(Vector::getRandomCloud(10));    
    std::unique_ptr<shape::ConvexShape> shapeB = std::make_unique<shape::ConvexCloud<list<Vector>>>(Vector::getRandomCloud(10));
   
    {
        SampleLogger result("Result1.json");
        result.doComplexQuery(solver, { *shapeA, *shapeB });
    }

    shapeB = std::make_unique<shape::TransformDecorator>(std::move(shapeB));
    static_cast<shape::TransformDecorator*>(shapeB.get())->setTraslation({3.f, 3.f, 3.f});
    {
        SampleLogger result("Result2.json");
        result.doComplexQuery(solver, { *shapeA, *shapeB });
    }

    return EXIT_SUCCESS;
}