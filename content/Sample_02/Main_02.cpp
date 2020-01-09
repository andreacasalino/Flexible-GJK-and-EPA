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



void set_random_v3(v3* result, const float& min_val, const float& max_val) {

	result->_x = rand_unif(min_val, max_val);
	result->_y = rand_unif(min_val, max_val);
	result->_z = rand_unif(min_val, max_val);

}

int main() {

	list<v3> cloud_A, cloud_B;
	v3 Angles_XYZ, Traslation_XYZ; //default values are assumed (0,0,0)
	list<v3> transformed;
	v3 Point_in_A, Point_in_B;

//build the solver to use for the subsequent queries
	GJK_EPA solver;
//build a logger to save the results (only for debugging purposes)
	Logger result_logger;

//build two poligons with a specified number of vertices
	size_t N_edge_A = 6;
	size_t N_edge_B = 3;
	get_3dpoligon(&cloud_A, N_edge_A);
	get_3dpoligon(&cloud_B, N_edge_B);

//set the poligons as shape A and B for the GJK_EPA solver
	solver.Set_shape_A(&cloud_A);
	Traslation_XYZ._x = 1.2f;
	solver.Set_shape_B_transformed(&cloud_B, Angles_XYZ, Traslation_XYZ); //only the orienation of shape B is changed

//get the penetration distance and the closest points
	float penetration = solver.Get_penetration(&Point_in_A, &Point_in_B);
//log results
	result_logger.Add_politope(cloud_A);
	trasform_points(&transformed, cloud_B, Angles_XYZ, Traslation_XYZ);   //this is just for the debuggin purpose
	result_logger.Add_politope(transformed);
	result_logger.Add_line(Point_in_A, Point_in_B);
	result_logger.Write_JSON("../Result_visualization/Sample_02_Log_01"); //you can use the python script in ../Result_visualization/Main.py to visualize the results



	result_logger.Clear();
//get two random rototraslations for both the shapes
	set_random_v3(&Angles_XYZ, 0.f, 0.5f*PI);
	set_random_v3(&Traslation_XYZ, 2.f, 3.5f);
	trasform_points(&transformed, cloud_A, Angles_XYZ, Traslation_XYZ);  //this is just for the debuggin purpose
	result_logger.Add_politope(transformed); 
	solver.Set_shape_A_transformed(&cloud_A, Angles_XYZ, Traslation_XYZ);

	set_random_v3(&Angles_XYZ, -0.5f*PI,0.f);
	set_random_v3(&Traslation_XYZ, -3.5f, -2.f);
	trasform_points(&transformed, cloud_B, Angles_XYZ, Traslation_XYZ);  //this is just for the debuggin purpose
	result_logger.Add_politope(transformed); 
	solver.Set_shape_B_transformed(&cloud_B, Angles_XYZ, Traslation_XYZ);

//get the closest points and the separating distance
	float distance = solver.Get_distance(&Point_in_A, &Point_in_B);
//log results
	trasform_points(&transformed, cloud_B, Angles_XYZ, Traslation_XYZ);
	result_logger.Add_politope(transformed);
	result_logger.Add_line(Point_in_A, Point_in_B);
	result_logger.Write_JSON("../Result_visualization/Sample_02_Log_02"); //you can use the python script in ../Result_visualization/Main.py to visualize the results
	
	return 0;
}