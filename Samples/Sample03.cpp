/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../src/GJK_EPA.h"
#include "../src/Vector3d_basic.h"
#include "../src/Result_Log.h"
using namespace std;

struct shape {
	list<v3>		Points;		//vertices
	v3				Angles;		//orientation: rotation along x -> rotation along y -> rotation along z
	v3				Traslations; //poisition 
};

// Add to the list of politopes a random poligon, with a random position and orientation
void add_random_shape( list<shape>* shapes ) {

	shapes->push_back(shape());

	size_t N_edge = rand() % 9 + 3;
	get_3dpoligon(&shapes->back().Points, N_edge);

	shapes->back().Angles._x = rand_unif(-PI, PI);
	shapes->back().Angles._y = rand_unif(-PI, PI);
	shapes->back().Angles._z = rand_unif(-PI, PI);

	shapes->back().Traslations._x = rand_unif(-5.f, 5.f);
	shapes->back().Traslations._y = rand_unif(-5.f, 5.f);
	shapes->back().Traslations._z = rand_unif(-5.f, 5.f);

}

//see Sample_01 and Sample_02 before this one
int main() {

	size_t N_shapes = 5;
	list<shape> shapes;

//build the solver to use for the subsequent queries
	GJK_EPA solver;
//build a logger to save the results (only for debugging purposes)
	Logger result_logger;

// sample N_shapes politopes
	list<v3> trasformed_points;
	for (size_t k = 0; k < N_shapes; k++) {
		add_random_shape(&shapes);
		trasform_points(&trasformed_points, shapes.back().Points, shapes.back().Angles, shapes.back().Traslations); //this is just for the debuggin purpose
		result_logger.Add_politope(trasformed_points); //add the sampled politope to the list of shapes to log in the JSON
	}

//check all the possible pairs: every pair is checked a single time.
	auto it2 = shapes.begin();
	v3 Point_in_A, Point_in_B;
	for (auto it = shapes.begin(); it != shapes.end(); it++) {
		it2 = it;
		it2++;
		for (it2; it2 != shapes.end(); it2++) {
			solver.Set_shape_A_transformed(&it->Points, it->Angles, it->Traslations);
			solver.Set_shape_B_transformed(&it2->Points, it2->Angles, it2->Traslations);

//collision check
			if (solver.Are_in_collision()) {
//collision detected: compute penetration distance
				solver.Get_penetration(&Point_in_A, &Point_in_B);
				result_logger.Add_line(Point_in_A, Point_in_B);
			}
			else {
//collision absent: compute closest points
				solver.Get_distance(&Point_in_A, &Point_in_B);
				result_logger.Add_line(Point_in_A, Point_in_B);
			}
		}
	}

//log results
	result_logger.Write_JSON("../Result_visualization/Sample_03_Log"); //you can use the python script in ../Result_visualization/Main.py to visualize the results

	return 0;
}