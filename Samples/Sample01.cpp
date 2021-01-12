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
// build the solver to use for the subsequent queries
	GjkEpa solver;
// used later
	flx::GjkEpa::CoordinatePair result;
	std::shared_ptr<std::list<Vector>> pointBufferA, pointBufferB;
{
// get random shapes with few points, centred around the origin
	pointBufferA = Vector::getRandomCloud(50);
	pointBufferB = Vector::getRandomCloud(50);
	shape::ConvexCloud<std::list<Vector>> shapeA(pointBufferA);
	shape::ConvexCloud<std::list<Vector>> shapeB(pointBufferB);
// collision is expected
	doComplexQuery(solver, {shapeA, shapeB}, result);
// translate one of the shape
	shape::TransformDecorator shapeB_traslated(std::make_unique<shape::ConvexCloud<std::list<Vector>>>(pointBufferA));
	shapeB_traslated.setTraslation(flx::Coordinate{3.f, 3.f, 3.f});
// no collision is expected now
	doComplexQuery(solver, {shapeA, shapeB}, result);
}

// repeat above steps with bigger point clouds
{
	pointBufferA = Vector::getRandomCloud(500);
	pointBufferB = Vector::getRandomCloud(500);
	shape::ConvexCloud<std::list<Vector>> shapeA(pointBufferA);
	shape::ConvexCloud<std::list<Vector>> shapeB(pointBufferB);
// collision is expected
	doComplexQuery(solver, {shapeA, shapeB}, result);
	shape::TransformDecorator shapeB_traslated(std::make_unique<shape::ConvexCloud<std::list<Vector>>>(pointBufferA));
	shapeB_traslated.setTraslation(flx::Coordinate{3.f, 3.f, 3.f});
// no collision is expected now
	doComplexQuery(solver, {shapeA, shapeB}, result);
}

	return EXIT_SUCCESS;
}