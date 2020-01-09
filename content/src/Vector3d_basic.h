/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef __VECTOR3D_BASIC_H__
#define __VECTOR3D_BASIC_H__

#include <list>
#include <cmath>

#define PI 3.1416f


//this is just an example of typename to use for representing 3d coordinates. You can feel free to define and use the one you prefer for representing the list of vertices of the convex shapes
struct v3 {
public:
	v3() : v3(0.f, 0.f, 0.f) {};
	v3(const float& X, const float& Y, const float& Z) { this->_x = X; this->_y = Y; this->_z = Z; };

	//these getters are used by the GJK_EPA solver
	const float& x() const { return this->_x; };
	const float& y() const { return this->_y; };
	const float& z()  const { return this->_z; };

	//these getters are used by the GJK_EPA solver, only when returning the results
	void x(const float& x) { this->_x = x; };
	void y(const float& y) { this->_y = y; };
	void z(const float& z) { this->_z = z; };

// data
	float		_x;
	float		_y;
	float		_z;
};

/** \brief Used for sampling within the uniform interval [low, upper]
*/
float rand_unif(const float& low, const float& upper) {

	if (upper < low) throw 0;

	return  low + (upper - low) *  (float)rand() / (float)RAND_MAX;

}

/** \brief Used for sampling N_points 3D coordinates, within a 
cube going from (-1,-1,-1) and (+1,+1,+1)
*/
void sample_cloud(std::list<v3>* Cloud, const size_t& N_points) {

	Cloud->clear();
	for (size_t k = 0; k < N_points; k++) {
		Cloud->push_back(v3(rand_unif(-1.f, 1.f), rand_unif(-1.f, 1.f), rand_unif(-1.f, 1.f)));
	}

}

/** \brief Used for computing the coordinates of a cloud after a rototraslation
\details The GJK_EPA solver doesn't need to explicitly perform this computations.
Indeed, you can use GJK_EPA::Set_shape_A_transformed or  GJK_EPA::Set_shape_B_transformed.
This function is mainly used for the aim of displaying the results.
*/
void trasform_points(std::list<v3>* Points_transformed , const std::list<v3>& Points ,const v3& rotation_XYZ, const v3& traslation_XYZ) {

	Points_transformed->clear();

	float C1 = cosf(rotation_XYZ._x), S1 = sinf(rotation_XYZ._x);
	float C2 = cosf(rotation_XYZ._y), S2 = sinf(rotation_XYZ._y);
	float C3 = cosf(rotation_XYZ._z), S3 = sinf(rotation_XYZ._z);

	float			Rotation[3][3];

	Rotation[0][0] = C2 * C3;
	Rotation[0][1] = -C2 * S3;
	Rotation[0][2] = S2;

	Rotation[1][0] = S1 * S2*C3 + C1 * S3;
	Rotation[1][1] = -S1 * S2*S3 + C1 * C3;
	Rotation[1][2] = -S1 * C2;

	Rotation[2][0] = -C1 * S2*C3 + S1 * S3;
	Rotation[2][1] = C1 * S2*S3 + S1 * C3;
	Rotation[2][2] = C1 * C2;

	for (auto it = Points.begin(); it != Points.end(); it++) {
		Points_transformed->push_back(v3());
		Points_transformed->back()._x = Rotation[0][0] * it->_x + Rotation[0][1] * it->_y + Rotation[0][2] * it->_z + traslation_XYZ._x;
		Points_transformed->back()._y = Rotation[1][0] * it->_x + Rotation[1][1] * it->_y + Rotation[1][2] * it->_z + traslation_XYZ._y;
		Points_transformed->back()._z = Rotation[2][0] * it->_x + Rotation[2][1] * it->_y + Rotation[2][2] * it->_z + traslation_XYZ._z;
	}

};

/** \brief Returns a poligonal set, considering the number of edges passed as input.
\details The poligon lies in the XY plane, centred at the origin. It goes form -0.2 to 0.2 alogn the z axis
*/
void get_3dpoligon(std::list<v3>* result, const size_t& N_edges) {

	result->clear();
	float delta_angle = 2.f * PI / (float)N_edges;
	float angle = 0.f;
	float temp[2];
	for (size_t k = 0; k < N_edges; k++) {
		temp[0] = cosf(angle);
		temp[1] = sinf(angle);
		result->push_back(v3(temp[0], temp[1], 0.2f));
		result->push_back(v3(temp[0], temp[1], -0.2f));
		angle += delta_angle;
	}

}

#endif 
