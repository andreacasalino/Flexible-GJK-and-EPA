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

int main() {

	list<v3> cloud_A, cloud_B;
	v3 Angles_XYZ, Traslation_XYZ; //default values are assumed (0,0,0)
	list<v3> transformed;
	v3 Point_in_A, Point_in_B;

//build the solver to use for the subsequent queries
	GJK_EPA solver;
//build a logger to save the results (only for debugging purposes)
	Logger result_logger;

// get random shapes with few points
	size_t Number_of_vertices = 50;
	sample_cloud(&cloud_A, Number_of_vertices);
	sample_cloud(&cloud_B, Number_of_vertices);

//set the poligons as shape A and B for the GJK_EPA solver
	solver.Set_shape_A(&cloud_A);
	solver.Set_shape_B(&cloud_B);

//get the penetration distance and the closest points
	float penetration = solver.Get_penetration(&Point_in_A, &Point_in_B);
//log results
	result_logger.Add_politope(cloud_A);
	result_logger.Add_politope(cloud_B);
	result_logger.Add_line(Point_in_A, Point_in_B);
	result_logger.Write_JSON("../Result_visualization/Sample_01_Log_01"); //you can use the python script in ../Result_visualization/Main.py to visualize the results

	result_logger.Clear();
//consider the same pair, but with a rototraslation for the points in shape B
	Traslation_XYZ._x = 3.f;
	Traslation_XYZ._y = 3.f;
	Traslation_XYZ._z = 3.f;
	//solver.Set_shape_A(&cloud_A); //redundant since shapeA was already set
	solver.Set_shape_B_transformed(&cloud_B, Angles_XYZ, Traslation_XYZ);
	
//get the closest points and the separating distance
	float distance = solver.Get_distance(&Point_in_A, &Point_in_B);
//log results
	result_logger.Add_politope(cloud_A);
	trasform_points(&transformed, cloud_B, Angles_XYZ, Traslation_XYZ);  //this is just for the debuggin purpose
	result_logger.Add_politope(transformed);
	result_logger.Add_line(Point_in_A, Point_in_B);
	result_logger.Write_JSON("../Result_visualization/Sample_01_Log_02"); //you can use the python script in ../Result_visualization/Main.py to visualize the results



// get re-sample the shapes, considering many points this time
	Number_of_vertices = 500;
	sample_cloud(&cloud_A, Number_of_vertices);
	sample_cloud(&cloud_B, Number_of_vertices);

//set the poligons as shape A and B for the GJK_EPA solver
	solver.Set_shape_A(&cloud_A);
	solver.Set_shape_B(&cloud_B);

//get the penetration distance of the new pair
	penetration = solver.Get_penetration(&Point_in_A, &Point_in_B);
//log results
	result_logger.Clear();
	result_logger.Add_politope(cloud_A);
	result_logger.Add_politope(cloud_B);
	result_logger.Add_line(Point_in_A, Point_in_B);
	result_logger.Write_JSON("../Result_visualization/Sample_01_Log_03"); //you can use the python script in ../Result_visualization/Main.py to visualize the results

//consider the same pair but with a rototrastlation
	solver.Set_shape_B_transformed(&cloud_B, Angles_XYZ, Traslation_XYZ);

	//get the closest points and the separating distance
	distance = solver.Get_distance(&Point_in_A, &Point_in_B);
//log results
	result_logger.Clear();
	result_logger.Add_politope(cloud_A);
	trasform_points(&transformed, cloud_B, Angles_XYZ, Traslation_XYZ);  //this is just for the debuggin purpose
	result_logger.Add_politope(transformed);
	result_logger.Add_line(Point_in_A, Point_in_B);
	result_logger.Write_JSON("../Result_visualization/Sample_01_Log_04"); //you can use the python script in ../Result_visualization/Main.py to visualize the results

	system("pause");
	return 0;
}