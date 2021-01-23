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

flx::Coordinate getRandomCorodinate(const float& min_val, const float& max_val) {
	auto randUnif = [](const float& min_val, const float& max_val) {
		float delta = max_val - min_val;
		return min_val + delta * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
	};

	return {
		randUnif(min_val, max_val),
		randUnif(min_val, max_val),
		randUnif(min_val, max_val)
	};
}

std::shared_ptr<std::list<Vector>> getPoligon(const std::size_t& N_edges) {
	std::shared_ptr<std::list<Vector>> cloud = std::make_shared<std::list<Vector>>();
	float delta_angle = 2.f * 3.14159f / static_cast<float>(N_edges);
	float angle = 0.f;
	float temp[2];
	for (std::size_t k = 0; k < N_edges; k++) {
		temp[0] = std::cosf(angle);
		temp[1] = std::sinf(angle);
		cloud->emplace_back(temp[0], temp[1],  0.2f);
		cloud->emplace_back(temp[0], temp[1], -0.2f);
		angle += delta_angle;
	}
	return cloud;
}

int main() {
// build the solver to use for the subsequent queries
	flx::GjkEpa solver;

//build two poligons with a specified number of vertices
	std::unique_ptr<flx::shape::ConvexShape> shapeA = std::make_unique<flx::shape::ConvexCloud<list<Vector>>>(getPoligon(6));
	std::unique_ptr<flx::shape::ConvexShape> shapeB = std::make_unique<flx::shape::ConvexCloud<list<Vector>>>(getPoligon(3));

//get the penetration distance and the closest points
	{
		SampleLogger result("Result_2_a.json");
		result.doComplexQuery(solver, { *shapeA, *shapeB });
	}

//get two random rototraslations for both the shapes
	shapeA = std::make_unique<flx::shape::TransformDecorator>(std::move(shapeA), getRandomCorodinate(2.f, 3.5f), getRandomCorodinate(0.f, 0.5f * 3.14159f));
	shapeB = std::make_unique<flx::shape::TransformDecorator>(std::move(shapeB), getRandomCorodinate(-3.5f, -2.f), getRandomCorodinate(-0.5f * 3.14159f, 0.f));

//get the closest points
	{
		SampleLogger result("Result_2_b.json");
		result.doComplexQuery(solver, { *shapeA, *shapeB });
	}
	
// analyze the .json storing the results using the python script Plotter.py

	return 0;
}