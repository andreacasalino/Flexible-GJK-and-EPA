/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "GJK_EPA.h"
using namespace std;


float GEOMETRIC_TOLLERANCE2 = GEOMETRIC_TOLLERANCE*GEOMETRIC_TOLLERANCE;
float GEOMETRIC_TOLLERANCE4 = GEOMETRIC_TOLLERANCE2 * GEOMETRIC_TOLLERANCE2;



void GJK_EPA::V::cross(const V& A, const V& B) {

	float new_x = A._y*B._z - A._z*B._y;
	float new_y = A._z*B._x - A._x*B._z;
	float new_z = A._x*B._y - A._y*B._x;

	this->_x = new_x;
	this->_y = new_y;
	this->_z = new_z;

}

void GJK_EPA::V::minus(const V& A, const V& B) {

	this->_x = A._x - B._x;
	this->_y = A._y - B._y;
	this->_z = A._z - B._z;

}

float GJK_EPA::V::squared_distance(const V& other) const {

	V temp;
	temp.minus(*this, other);
	return temp.dot(temp);

}

void GJK_EPA::V::normalize_in_place() {

	float n = this->dot(*this);
	if (n > GEOMETRIC_TOLLERANCE) {
		n = 1.f / sqrtf(n);
		this->_x *= n;
		this->_y *= n;
		this->_z *= n;
	}

}

void GJK_EPA::V::add(const float& coeff, const V& to_add) {

	this->_x += to_add._x *coeff;
	this->_y += to_add._y *coeff;
	this->_z += to_add._z *coeff;

}



bool GJK_EPA::Are_in_collision() {

	if ((this->State_Machine == 1) || (this->State_Machine == 2)) return true;
	if ((this->State_Machine == 3) || (this->State_Machine == 4)) return false;

	if ((this->Shape_A == NULL) || (this->Shape_B == NULL)) {
		throw 0; //one shape was not set
		return true;
	}

	if (this->Last_GJK == NULL) this->Last_GJK = new Plex(this);
	if(this->Last_GJK->is_collision_present()) {
#ifdef ENABLE_LOG
		if (this->bclose_log) {
			*this->log_stream << ",\"GJK_finish\":[0]\n";
			*this->log_stream << ",\"EPA\":[0]\n";
			this->close_log();
		}
#endif
		this->State_Machine = 1;
		return true;
	}
	else {
#ifdef ENABLE_LOG
		if (this->bclose_log) {
			*this->log_stream << ",\"GJK_finish\":[0]\n";
			*this->log_stream << ",\"EPA\":[0]\n";
			this->close_log();
		}
#endif
		this->State_Machine = 3;
		return false;
	}

}

void GJK_EPA::__get_Support_Minkowski_diff(V* result, const V& Direction, V* support_A, V* support_B) {

	V Direction_twin;
	if (this->Trasformation_A == NULL)  this->Shape_A->Get_Support(support_A, Direction);
	else {
		this->Trasformation_A->Per_Rtranspose(&Direction_twin, Direction);
		this->Shape_A->Get_Support(support_A, Direction_twin);
		this->Trasformation_A->Per_R_T(support_A, *support_A);
	}

	Direction_twin._x = -Direction._x;
	Direction_twin._y = -Direction._y;
	Direction_twin._z = -Direction._z;
	if (this->Trasformation_B == NULL)  this->Shape_B->Get_Support(support_B, Direction_twin);
	else {
		this->Trasformation_B->Per_Rtranspose(&Direction_twin, Direction_twin);
		this->Shape_B->Get_Support(support_B, Direction_twin);
		this->Trasformation_B->Per_R_T(support_B, *support_B);
	}

	result->_x = support_A->_x - support_B->_x;
	result->_y = support_A->_y - support_B->_y;
	result->_z = support_A->_z - support_B->_z;

}

#ifdef ENABLE_LOG
void GJK_EPA::close_log() {

	*this->log_stream << ",\"Closest\":{\n";
	*this->log_stream << "\"A\":[" << this->Points[0]._x << "," << this->Points[0]._y << "," << this->Points[0]._z << "]\n";
	*this->log_stream << ",\"B\":[" << this->Points[1]._x << "," << this->Points[1]._y << "," << this->Points[1]._z << "]\n";
	*this->log_stream << "}\n}";
	delete this->log_stream;
	this->log_stream = NULL;

}
#endif



void GJK_EPA::__trasformation::reset(const V& rotation_angles, const V& traslation) {

	float C1 = cosf(rotation_angles._x), S1 = sinf(rotation_angles._x);
	float C2 = cosf(rotation_angles._y), S2 = sinf(rotation_angles._y);
	float C3 = cosf(rotation_angles._z), S3 = sinf(rotation_angles._z);

	this->Rotation[0][0] = C2 * C3;
	this->Rotation[0][1] = -C2 * S3;
	this->Rotation[0][2] = S2;

	this->Rotation[1][0] = S1 * S2*C3 + C1 * S3;
	this->Rotation[1][1] = -S1 * S2*S3 + C1 * C3;
	this->Rotation[1][2] = -S1 * C2;

	this->Rotation[2][0] = -C1 * S2*C3 + S1 * S3;
	this->Rotation[2][1] = C1 * S2*S3 + S1 * C3;
	this->Rotation[2][2] = C1 * C2;

	this->Traslation[0] = traslation._x;
	this->Traslation[1] = traslation._y;
	this->Traslation[2] = traslation._z;

}

void GJK_EPA::__trasformation::Per_Rtranspose(V* result, const V& Dx) {

	float new_x = this->Rotation[0][0] * Dx._x + this->Rotation[1][0] * Dx._y + this->Rotation[2][0] * Dx._z;
	float new_y = this->Rotation[0][1] * Dx._x + this->Rotation[1][1] * Dx._y + this->Rotation[2][1] * Dx._z;
	float new_z = this->Rotation[0][2] * Dx._x + this->Rotation[1][2] * Dx._y + this->Rotation[2][2] * Dx._z;

	result->_x = new_x;
	result->_y = new_y;
	result->_z = new_z;

}

void GJK_EPA::__trasformation::Per_R_T(V* result, const V& Dx) {

	float new_x = this->Rotation[0][0] * Dx._x + this->Rotation[0][1] * Dx._y + this->Rotation[0][2] * Dx._z + this->Traslation[0];
	float new_y = this->Rotation[1][0] * Dx._x + this->Rotation[1][1] * Dx._y + this->Rotation[1][2] * Dx._z + this->Traslation[1];
	float new_z = this->Rotation[2][0] * Dx._x + this->Rotation[2][1] * Dx._y + this->Rotation[2][2] * Dx._z + this->Traslation[2];

	result->_x = new_x;
	result->_y = new_y;
	result->_z = new_z;

}



GJK_EPA::closest_element GJK_EPA::__get_closest_in_segment(const V& A, const V& B, float* miximg_coeff) {

	miximg_coeff[2] = 0.f;

	V B_A;
	B_A.minus(B, A);
	miximg_coeff[1] = -B_A.dot(A) / B_A.dot(B_A);

	if (miximg_coeff[1] <= 0.f) {
		miximg_coeff[0] = 1.f;
		miximg_coeff[1] = 0.f;
		return vertex_A;
	}
	else {
		miximg_coeff[0] = 1.f - miximg_coeff[1];
		return edge_AB;
	}

}

GJK_EPA::closest_element GJK_EPA::__get_closest_in_triangle(const V& A, const V& B, const V& C, float* miximg_coeff) {

	V B_A;
	B_A.minus(B, A);
	V C_A;
	C_A.minus(C, A);

	float m11 = B_A.dot(B_A);
	float m22 = C_A.dot(C_A);
	float m12 = B_A.dot(C_A);

	float c1 = -A.dot(B_A);
	float c2 = -A.dot(C_A);

	miximg_coeff[1] = (c1*m22 - m12 * c2) / (m11*m22 - m12 * m12);
	miximg_coeff[2] = (c2 - m12 * miximg_coeff[1]) / m22;

	bool temp[3];
	temp[0] = ((miximg_coeff[2] >= 0.f) && (miximg_coeff[2] <= 1.f));
	temp[1] = ((miximg_coeff[1] >= 0.f) && (miximg_coeff[1] <= 1.f));
	temp[2] = (miximg_coeff[1] + miximg_coeff[2] <= 1.f);

	if (temp[0] && temp[1] && temp[2] ) {
		miximg_coeff[0] = 1.f - miximg_coeff[1] - miximg_coeff[2];
		return face_ABC;
	}
	else {
		auto closest_AB = __get_closest_in_segment(A, B, miximg_coeff);
		V V_AB(A);
		V_AB.dot(miximg_coeff[0]);
		V_AB.add(miximg_coeff[1], B);
		float d_AB = V_AB.dot(V_AB);

		float coeff_AC[3];
		auto closest_AC = __get_closest_in_segment(A, C, &coeff_AC[0]);
		V V_AC(A);
		V_AC.dot(coeff_AC[0]);
		V_AC.add(coeff_AC[1], C);
		float d_AC = V_AC.dot(V_AC);

		if (d_AB < d_AC) return closest_AB;
		else {
			miximg_coeff[0] = coeff_AC[0];
			miximg_coeff[1] = coeff_AC[1];
			if (closest_AC == edge_AB) closest_AC = edge_AC;
			return closest_AC;
		}
	}

}

void GJK_EPA::__compute_outside_normal(V* N_result, const V& P1, const V& P2, const V& P3, const V& P_other) {

	V Delta1, Delta2;
	Delta1.minus(P2, P1);
	Delta2.minus(P3, P1);
	N_result->cross(Delta1, Delta2);

	Delta1.minus(P_other, P1);
	if (N_result->dot(Delta1) > GEOMETRIC_TOLLERANCE) N_result->invert();
	N_result->normalize_in_place();

}

#ifdef ENABLE_LOG
void GJK_EPA::log_points(const list<V>& L , ofstream* f) {

	auto it = L.begin();
	size_t S = L.size();
	*f << "[\n";
	for (size_t k = 1; k < S; k++) {
		*f << "[" << it->_x << "," << it->_y << "," << it->_z << "],\n";
		it++;
	}
	*f << "[" << it->_x << "," << it->_y << "," << it->_z << "]\n";
	*f << "]\n";

}
#endif

GJK_EPA::Plex::Plex(GJK_EPA* emitter) : Emitter(emitter) {

#ifdef ENABLE_LOG
	this->Emitter->__release(this->Emitter->log_stream);
	this->Emitter->log_stream = new ofstream(log_location);
	if (!this->Emitter->log_stream->is_open())
		throw 1;

	*this->Emitter->log_stream << "{\"Clouds\":{\n";
	list<V> points_temp;

	this->Emitter->Shape_A->Get_vertices(&points_temp , this->Emitter->Trasformation_A);
	*this->Emitter->log_stream << "\"A\":\n";
	log_points(points_temp, this->Emitter->log_stream);

	this->Emitter->Shape_B->Get_vertices(&points_temp, this->Emitter->Trasformation_B);
	*this->Emitter->log_stream << ",\"B\":\n";
	log_points(points_temp, this->Emitter->log_stream);

	*this->Emitter->log_stream << "}\n";

	*this->Emitter->log_stream << ",\"GJK\":[\n";
#endif

	for (size_t k = 0; k < 4; k++) this->plex[k] = new V3();

	this->plex_dim = 1;
	this->Emitter->__get_Support_Minkowski_diff(&this->plex[1]->P_in_MikDiff, V(1.f, 0.f, 0.f), &this->plex[1]->P_in_shapeA, &this->plex[1]->P_in_shapeB);
	if (this->plex[1]->P_in_MikDiff.dot(this->plex[1]->P_in_MikDiff) <= GEOMETRIC_TOLLERANCE2) {
		this->collision_present = true;
#ifdef ENABLE_LOG
		*this->Emitter->log_stream << "{\"d\":0}]\n";
#endif
		return;
	}
	this->Direction = this->plex[1]->P_in_MikDiff;
	this->Direction.invert();
	this->Direction.normalize_in_place();

	V temp;
	while (true) {
		this->Emitter->__get_Support_Minkowski_diff(&this->plex[0]->P_in_MikDiff, this->Direction, &this->plex[0]->P_in_shapeA, &this->plex[0]->P_in_shapeB);
		if (this->plex[0]->P_in_MikDiff.dot(this->Direction) <= GEOMETRIC_TOLLERANCE) {
			this->collision_present = false;
#ifdef ENABLE_LOG
			*this->Emitter->log_stream << "{\"d\":0}]\n";
#endif
			return;
		}
		this->plex_dim++;

		//check if plex contains origin
		if (this->plex_dim == 4) {
			this->__update_Origin_is_visible();
			if (!(this->Origin_is_visible[0] && this->Origin_is_visible[1] && this->Origin_is_visible[2])) {
				this->collision_present = true;
				this->plex_dim--;
#ifdef ENABLE_LOG
				*this->Emitter->log_stream << "{\"d\":0}]\n";
#endif
				return;
			}
		}
		else if (this->plex_dim == 3) {
			float coeff[3];
			this->Check_triangle_origin = __get_closest_in_triangle(this->plex[0]->P_in_MikDiff, this->plex[1]->P_in_MikDiff, this->plex[2]->P_in_MikDiff, &coeff[0]);
			temp = this->plex[0]->P_in_MikDiff;
			temp.dot(coeff[0]);
			temp.add(coeff[1], this->plex[1]->P_in_MikDiff);
			temp.add(coeff[2], this->plex[2]->P_in_MikDiff);
			if (temp.dot(temp) <= GEOMETRIC_TOLLERANCE2) {
				this->collision_present = true;
				this->plex_dim--;
#ifdef ENABLE_LOG
				*this->Emitter->log_stream << "{\"d\":0}]\n";
#endif
				return;
			}
		}
		else if (this->plex_dim == 2) {
			temp.cross(this->plex[0]->P_in_MikDiff , this->plex[1]->P_in_MikDiff);
			if (temp.squared_norm() <= GEOMETRIC_TOLLERANCE4) {
				this->collision_present = true;
				this->plex_dim--;
#ifdef ENABLE_LOG
				*this->Emitter->log_stream << "{\"d\":0}]\n";
#endif
				return;
			}
		}

		this->__update();
	}

}

void GJK_EPA::Plex::finishing_loop() {

#ifdef ENABLE_LOG
	*this->Emitter->log_stream << ",\"GJK_finish\":[\n";
#endif

	V delta;
	this->Check_triangle_origin = 4;

	delta.minus(this->plex[0]->P_in_MikDiff, this->plex[1]->P_in_MikDiff);
	if (this->Direction.dot(delta) <= GEOMETRIC_TOLLERANCE) {
		this->plex_dim--;
	}
	else {
		if (this->plex_dim == 4)
			this->__update_Origin_is_visible();
		this->__update();

		while (true) {
			this->Emitter->__get_Support_Minkowski_diff(&this->plex[0]->P_in_MikDiff, this->Direction, &this->plex[0]->P_in_shapeA, &this->plex[0]->P_in_shapeB);
			this->plex_dim++;

			delta.minus(this->plex[0]->P_in_MikDiff, this->plex[1]->P_in_MikDiff);
			if (this->Direction.dot(delta) <= GEOMETRIC_TOLLERANCE) {
				this->plex_dim--;
				break;
			}

			if (this->plex_dim == 4)
				this->__update_Origin_is_visible();
			this->__update();
		}
	}

	if (this->plex_dim == 1) {
		this->Emitter->Points[0] = this->plex[1]->P_in_shapeA;
		this->Emitter->Points[1] = this->plex[1]->P_in_shapeB;
	}
	else if(this->plex_dim == 2) {
		float coeff[3];
		__get_closest_in_segment(this->plex[1]->P_in_MikDiff, this->plex[2]->P_in_MikDiff, &coeff[0]);

		this->Emitter->Points[0] = this->plex[1]->P_in_shapeA;
		this->Emitter->Points[0].dot(coeff[0]);
		this->Emitter->Points[0].add(coeff[1], this->plex[2]->P_in_shapeA);

		this->Emitter->Points[1] = this->plex[1]->P_in_shapeB;
		this->Emitter->Points[1].dot(coeff[0]);
		this->Emitter->Points[1].add(coeff[1], this->plex[2]->P_in_shapeB);
	}
	else if (this->plex_dim == 3) {
		float coeff[3];
		__get_closest_in_triangle(this->plex[1]->P_in_MikDiff, this->plex[2]->P_in_MikDiff, this->plex[3]->P_in_MikDiff, &coeff[0]);

		this->Emitter->Points[0] = this->plex[1]->P_in_shapeA;
		this->Emitter->Points[0].dot(coeff[0]);
		this->Emitter->Points[0].add(coeff[1], this->plex[2]->P_in_shapeA);
		this->Emitter->Points[0].add(coeff[2], this->plex[3]->P_in_shapeA);

		this->Emitter->Points[1] = this->plex[1]->P_in_shapeB;
		this->Emitter->Points[1].dot(coeff[0]);
		this->Emitter->Points[1].add(coeff[1], this->plex[2]->P_in_shapeB);
		this->Emitter->Points[1].add(coeff[2], this->plex[3]->P_in_shapeB);
	}

#ifdef ENABLE_LOG
	*this->Emitter->log_stream << "{\"d\":0}]\n";
#endif

}

void GJK_EPA::Plex::__update_Origin_is_visible() {

	__compute_outside_normal(&this->Normals[0], this->plex[0]->P_in_MikDiff, this->plex[1]->P_in_MikDiff, this->plex[2]->P_in_MikDiff, this->plex[3]->P_in_MikDiff);
	if (this->Normals[0].dot(this->plex[0]->P_in_MikDiff) <= GEOMETRIC_TOLLERANCE) this->Origin_is_visible[0] = true;
	else this->Origin_is_visible[0] = false;

	__compute_outside_normal(&this->Normals[1], this->plex[0]->P_in_MikDiff, this->plex[1]->P_in_MikDiff, this->plex[3]->P_in_MikDiff, this->plex[2]->P_in_MikDiff);
	if (this->Normals[1].dot(this->plex[0]->P_in_MikDiff) <= GEOMETRIC_TOLLERANCE) this->Origin_is_visible[1] = true;
	else this->Origin_is_visible[1] = false;

	__compute_outside_normal(&this->Normals[2], this->plex[0]->P_in_MikDiff, this->plex[2]->P_in_MikDiff, this->plex[3]->P_in_MikDiff, this->plex[1]->P_in_MikDiff);
	if (this->Normals[2].dot(this->plex[0]->P_in_MikDiff) <= GEOMETRIC_TOLLERANCE) this->Origin_is_visible[2] = true;
	else this->Origin_is_visible[2] = false;

}

struct __Facet_Inc {
	struct __facet_inc {
		size_t pos[3];
	};

	__Facet_Inc()  {
		Inc[0].pos[0] = 0;
		Inc[0].pos[1] = 1;
		Inc[0].pos[2] = 2;

		Inc[1].pos[0] = 0;
		Inc[1].pos[1] = 1;
		Inc[1].pos[2] = 3;

		Inc[2].pos[0] = 0;
		Inc[2].pos[1] = 2;
		Inc[2].pos[2] = 3;
	}

	__facet_inc Inc[3];
} Incidences;
void GJK_EPA::Plex::__update() {

#ifdef ENABLE_LOG
	*this->Emitter->log_stream << "     {\n";
	*this->Emitter->log_stream << "     \"Plex\":\n";
	list<V> plex_log;
	for (size_t k = 0; k < this->plex_dim; k++) 
		plex_log.push_back(V(this->plex[k]->P_in_MikDiff));
	log_points(plex_log , this->Emitter->log_stream);
	*this->Emitter->log_stream << "     ,\"D_old\":["<< this->Direction._x << "," << this->Direction._y << "," << this->Direction._z  << "]\n";
#endif

	float coeff[3];
	if (this->plex_dim == 4) {
		closest_element regions[3];
		float distances[3];
		V temp;

#ifdef ENABLE_LOG
		V temp_closest[3];
#endif

		for (size_t k = 0; k < 3; k++) {
			if (this->Origin_is_visible[k]) {
				regions[k] = __get_closest_in_triangle(this->plex[Incidences.Inc[k].pos[0]]->P_in_MikDiff, this->plex[Incidences.Inc[k].pos[1]]->P_in_MikDiff, this->plex[Incidences.Inc[k].pos[2]]->P_in_MikDiff, &coeff[0]);
				temp = this->plex[Incidences.Inc[k].pos[0]]->P_in_MikDiff;
				temp.dot(coeff[0]);
				temp.add(coeff[1], this->plex[Incidences.Inc[k].pos[1]]->P_in_MikDiff);
				temp.add(coeff[2], this->plex[Incidences.Inc[k].pos[2]]->P_in_MikDiff);
				distances[k] = temp.dot(temp);
#ifdef ENABLE_LOG
				temp_closest[k] = temp;
#endif
			}
			else distances[k] = FLT_MAX;
		}

		size_t closest = 0;
		if (distances[1] < distances[closest]) closest = 1;
		if (distances[2] < distances[closest]) closest = 2;

#ifdef ENABLE_LOG
		*this->Emitter->log_stream << "     ,\"Closest\":[" << temp_closest[closest]._x << "," << temp_closest[closest]._y << "," << temp_closest[closest]._z << "]\n";
#endif

		if (regions[closest] == face_ABC)
			this->__set_plex_to_facet(closest);
		else  if (regions[closest] == vertex_A)
			this->__set_plex_to_vertex();
		else {
			switch (closest) {
			case 0:
				if (regions[0] == edge_AB)
					this->__set_plex_to_segment(0);
				else
					this->__set_plex_to_segment(1);
				break;
			case 1:
				if (regions[1] == edge_AB)
					this->__set_plex_to_segment(0);
				else
					this->__set_plex_to_segment(2);
				break;
			case 2:
				if (regions[2] == edge_AB)
					this->__set_plex_to_segment(1);
				else
					this->__set_plex_to_segment(2);
				break;
			}
		}
	}
	else if (this->plex_dim == 3) {
		closest_element region;
		if (this->Check_triangle_origin == 4)
			region = __get_closest_in_triangle(this->plex[0]->P_in_MikDiff, this->plex[1]->P_in_MikDiff, this->plex[2]->P_in_MikDiff, &coeff[0]);
		else
			region = (closest_element)this->Check_triangle_origin;

#ifdef ENABLE_LOG
		float debug_coeff[3];
		__get_closest_in_triangle(this->plex[0]->P_in_MikDiff, this->plex[1]->P_in_MikDiff, this->plex[2]->P_in_MikDiff, &debug_coeff[0]);
		V debug_closest = this->plex[0]->P_in_MikDiff;
		debug_closest.dot(debug_coeff[0]);
		debug_closest.add(debug_coeff[1], this->plex[1]->P_in_MikDiff);
		debug_closest.add(debug_coeff[2], this->plex[2]->P_in_MikDiff);
		*this->Emitter->log_stream << "     ,\"Closest\":[" << debug_closest._x << "," << debug_closest._y << "," << debug_closest._z << "]\n";
#endif

		if (region == face_ABC) {
			this->__set_plex_to_facet(0);
		}
		else if (region == vertex_A)
			this->__set_plex_to_vertex();
		else if (region == edge_AB)
			this->__set_plex_to_segment(0);
		else
			this->__set_plex_to_segment(1);
	}
	else  { //dim_plex == 2
		auto region = __get_closest_in_segment(this->plex[0]->P_in_MikDiff, this->plex[1]->P_in_MikDiff, &coeff[0]);
#ifdef ENABLE_LOG
		V debug_closest = this->plex[0]->P_in_MikDiff;
		debug_closest.dot(coeff[0]);
		debug_closest.add(coeff[1] , this->plex[1]->P_in_MikDiff);
		*this->Emitter->log_stream << "     ,\"Closest\":[" << debug_closest._x << "," << debug_closest._y << "," << debug_closest._z << "]\n";
#endif
		if (region == vertex_A)
			this->__set_plex_to_vertex();
		else
			this->__set_plex_to_segment(0);

	}

#ifdef ENABLE_LOG
	*this->Emitter->log_stream << "     ,\"D\":[" << this->Direction._x << "," << this->Direction._y << "," << this->Direction._z << "]\n";
	*this->Emitter->log_stream << "     },\n";
#endif

}

void GJK_EPA::Plex::__set_plex_to_vertex() {

	this->Direction = this->plex[0]->P_in_MikDiff;
	this->Direction.invert();
	this->Direction.normalize_in_place();
	this->plex_dim = 1;
	auto temp = this->plex[1];
	this->plex[1] = this->plex[0];
	this->plex[0] = temp;

}

void GJK_EPA::Plex::__set_plex_to_segment(const size_t& id) {

	V* A = &this->plex[0]->P_in_MikDiff, *B;
	V3* temp = NULL;
	B = NULL;
	switch (id) {
	case 0:
		B = &this->plex[1]->P_in_MikDiff;
		temp = this->plex[2];
		this->plex[2] = this->plex[1];
		this->plex[1] = this->plex[0];
		this->plex[0] = temp;
		break;
	case 1:
		B = &this->plex[2]->P_in_MikDiff;
		temp = this->plex[1];
		this->plex[1] = this->plex[0];
		this->plex[0] = temp;
		break;
	case 2:
		B = &this->plex[3]->P_in_MikDiff;
		temp = this->plex[1];
		this->plex[1] = this->plex[0];
		this->plex[0] = temp;
		temp = this->plex[3];
		this->plex[3] = this->plex[2];
		this->plex[2] = temp;
		break;
	}

	this->Direction.cross(*A, *B);
	V B_A;
	B_A.minus(*B, *A);
	this->Direction.cross(this->Direction, B_A);
	this->Direction.normalize_in_place();
	this->plex_dim = 2;

}

void GJK_EPA::Plex::__set_plex_to_facet(const size_t& id) {

	V* A = &this->plex[0]->P_in_MikDiff, *B = NULL, *C = NULL;
	V3* temp = NULL;
	switch (id)
	{
	case 0:
		B = &this->plex[1]->P_in_MikDiff;
		C = &this->plex[2]->P_in_MikDiff;
		temp = this->plex[3];
		this->plex[3] = this->plex[2];
		this->plex[2] = this->plex[1];
		this->plex[1] = this->plex[0];
		this->plex[0] = temp;
		break;
	case 1:
		B = &this->plex[1]->P_in_MikDiff;
		C = &this->plex[3]->P_in_MikDiff;
		temp = this->plex[2];
		this->plex[2] = this->plex[1];
		this->plex[1] = this->plex[0];
		this->plex[0] = temp;
		break;
	case 2:
		B = &this->plex[2]->P_in_MikDiff;
		C = &this->plex[3]->P_in_MikDiff;
		temp = this->plex[1];
		this->plex[1] = this->plex[0];
		this->plex[0] = temp;
		break;
	}

	if (this->plex_dim == 3) {
		__compute_outside_normal(&this->Direction, *A, *B, *C, V(0.f, 0.f, 0.f));
		this->Direction.invert();
	}
	else 
		this->Direction = this->Normals[id];
	this->plex_dim = 3;

}

void GJK_EPA::Plex::copy_plex_4EPA(std::list<V3>* plex) {

	plex->clear();
	for (size_t k = 0; k < this->plex_dim; k++) 
plex->push_back(*this->plex[k + 1]);

}



void GJK_EPA::EPA() {

#ifdef ENABLE_LOG
	*this->log_stream << ",\"EPA\":[\n";
#endif

	list<V3> vertices_register;
	this->Last_GJK->copy_plex_4EPA(&vertices_register);

	//build initial thetraedron
	V D;
	size_t Size;
	while (vertices_register.size() < 4) {
		Size = vertices_register.size();
		if (Size == 1) {
			D = vertices_register.front().P_in_MikDiff;
			if (D.dot(D) <= GEOMETRIC_TOLLERANCE2) {
				D._x = 1.f;
				D._y = 0.f;
				D._z = 0.f;
			}
			else  D.invert();
			vertices_register.push_back(V3());
			this->__get_Support_Minkowski_diff(&vertices_register.back().P_in_MikDiff, D, &vertices_register.back().P_in_shapeA, &vertices_register.back().P_in_shapeB);
			if (vertices_register.back().P_in_MikDiff.squared_distance(vertices_register.front().P_in_MikDiff) <= GEOMETRIC_TOLLERANCE2) {
				D.invert();
				this->__get_Support_Minkowski_diff(&vertices_register.back().P_in_MikDiff, D, &vertices_register.back().P_in_shapeA, &vertices_register.back().P_in_shapeB);
			}
		}
		else if (Size == 2) {
			V D2(1.f, 0.f, 0.f);
			D.minus(vertices_register.front().P_in_MikDiff, vertices_register.back().P_in_MikDiff);
			D2.cross(D, D2);
			if (D2.squared_norm() <= GEOMETRIC_TOLLERANCE2) {
				D2._x = 0.f;
				D2._y = 1.f;
				D2._z = 0.f;
				D2.cross(D, D2);
			}
			V3 tempV3;
			this->__get_Support_Minkowski_diff(&tempV3.P_in_MikDiff, D2, &tempV3.P_in_shapeA, &tempV3.P_in_shapeB);
			bool tempB[2];
			tempB[0] = (tempV3.P_in_MikDiff.squared_distance(vertices_register.front().P_in_MikDiff) <= GEOMETRIC_TOLLERANCE2);
			tempB[1] = (tempV3.P_in_MikDiff.squared_distance(vertices_register.back().P_in_MikDiff) <= GEOMETRIC_TOLLERANCE2);
			if (tempB[0] || tempB[1]) {
				D2.invert();
				this->__get_Support_Minkowski_diff(&tempV3.P_in_MikDiff, D2, &tempV3.P_in_shapeA, &tempV3.P_in_shapeB);
			}
			vertices_register.push_back(tempV3);
		}
		else {
			V* A, *B, *C;
			auto it_reg = vertices_register.begin();
			A = &(it_reg->P_in_MikDiff); it_reg++;
			B = &(it_reg->P_in_MikDiff); it_reg++;
			C = &(it_reg->P_in_MikDiff);

			__compute_outside_normal(&D, *A, *B, *C, V(0.f, 0.f, 0.f));
			D.invert();

			vertices_register.push_back(V3());
			this->__get_Support_Minkowski_diff(&vertices_register.back().P_in_MikDiff, D, &vertices_register.back().P_in_shapeA, &vertices_register.back().P_in_shapeB);
			V delta;
			delta.minus(vertices_register.back().P_in_MikDiff, vertices_register.front().P_in_MikDiff);
			if (delta.dot(D) <= GEOMETRIC_TOLLERANCE) {
				D.invert();
				this->__get_Support_Minkowski_diff(&vertices_register.back().P_in_MikDiff, D, &vertices_register.back().P_in_shapeA, &vertices_register.back().P_in_shapeB);
			}
		}
	}

	struct Facet {
		bool					bVisible; //only for update Hull
		const V3*			A; //pointer of the vertex A
		const V3*			B; //pointer of the vertex B
		const V3*			C; //pointer of the vertex C
		size_t					id_A;// identifief of vertex A(if not passed 0 for all points is assumed)
		size_t					id_B;// identifief of vertex A(if not passed 0 for all points is assumed)
		size_t					id_C;// identifief of vertex A(if not passed 0 for all points is assumed)
		float						N[3]; //outer normal 
		Facet*				Neighbour[3]; // AB, BC, CA

		float						distance_to_origin;
	};

	class Convex_Set : public Hull<Facet, V3> {
	public:
		Convex_Set(const list<V3>& initial_plex) : Hull<Facet, V3>(
			get_point(initial_plex, 0), get_point(initial_plex, 1), get_point(initial_plex, 2), get_point(initial_plex, 3)) { };
		void Update(const V3* vertex_of_new_cone, std::list<Facet*>* created_cone) { this->Hull<Facet, V3>::__Update_Hull(vertex_of_new_cone, created_cone); };
		std::list<Facet>* get_facets() { return &this->Facets; };
#ifdef ENABLE_LOG
		void Log_facets(ofstream* f) {
			*f << "\"Facets\":[\n";
			for (auto it = this->Facets.begin(); it != this->Facets.end(); it++) {
				*f << "{\n";
				*f << "\"A\":[" << it->A->x() << "," << it->A->y() << "," << it->A->z() << "]\n";
				*f << ",\"B\":[" << it->B->x() << "," << it->B->y() << "," << it->B->z() << "]\n";
				*f << ",\"C\":[" << it->C->x() << "," << it->C->y() << "," << it->C->z() << "]\n";
				*f << ",\"N\":[" << it->N[0] << "," << it->N[1] << "," << it->N[2] << "]\n";
				*f << "},\n";
			}
			*f << "{}\n";
			*f << "]\n";
		};
#endif
	private:
		static const V3* get_point(const list<V3>& initial_plex, const size_t& pos) {
			auto it = initial_plex.begin();
			advance(it, pos);
			return &(*it);
		}
	};

	Convex_Set Minkowski_diff(vertices_register);
	for (auto it = Minkowski_diff.get_facets()->begin(); it != Minkowski_diff.get_facets()->end(); it++)
		it->distance_to_origin = abs(it->N[0] * it->A->x() + it->N[1] * it->A->y() + it->N[2] * it->A->z());

	Facet* closest_to_origin = NULL;
	auto Facets = Minkowski_diff.get_facets();
	auto it_F = Facets->begin();
	auto it_F_end = it_F;
	float temp;
	std::list<Facet*> cone;
	auto it_cone = cone.begin();
	V V_temp;
	while (true) {
#ifdef ENABLE_LOG
		*this->log_stream << "{\n";
		Minkowski_diff.Log_facets(this->log_stream);
#endif

		it_F_end = Facets->end();
		it_F = Facets->begin();
		closest_to_origin = &(*it_F);
		it_F++;
		for (it_F; it_F != it_F_end; it_F++) {
			if (it_F->distance_to_origin < closest_to_origin->distance_to_origin)
				closest_to_origin = &(*it_F);
		}

#ifdef ENABLE_LOG
		*this->log_stream << ",\"Closest\":{\n";
		*this->log_stream << "\"A\":[" << closest_to_origin->A->x() << "," << closest_to_origin->A->y() << "," << closest_to_origin->A->z() << "]\n";
		*this->log_stream << ",\"B\":[" << closest_to_origin->B->x() << "," << closest_to_origin->B->y() << "," << closest_to_origin->B->z() << "]\n";
		*this->log_stream << ",\"C\":[" << closest_to_origin->C->x() << "," << closest_to_origin->C->y() << "," << closest_to_origin->C->z() << "]\n";
		*this->log_stream << "}\n";
		*this->log_stream << "},\n";
#endif

		vertices_register.push_back(V3());
		V_temp._x = closest_to_origin->N[0];
		V_temp._y = closest_to_origin->N[1];
		V_temp._z = closest_to_origin->N[2];
		this->__get_Support_Minkowski_diff(&vertices_register.back().P_in_MikDiff , V_temp, &vertices_register.back().P_in_shapeA, &vertices_register.back().P_in_shapeB);

		V_temp.minus(vertices_register.back().P_in_MikDiff, closest_to_origin->A->P_in_MikDiff);
		temp = closest_to_origin->N[0] * V_temp._x + closest_to_origin->N[1] * V_temp._y + closest_to_origin->N[2] * V_temp._z ;
		if (temp < GEOMETRIC_TOLLERANCE)
			break;

		Minkowski_diff.Update(&vertices_register.back(), &cone);
		for (it_cone = cone.begin(); it_cone != cone.end(); it_cone++)
			(*it_cone)->distance_to_origin = abs((*it_cone)->N[0] * (*it_cone)->A->x() + (*it_cone)->N[1] * (*it_cone)->A->y() + (*it_cone)->N[2] * (*it_cone)->A->z());

	}

#ifdef ENABLE_LOG
	*this->log_stream << "{\"d\":0}]\n";
#endif

	float coeff[3];
	__get_closest_in_triangle(closest_to_origin->A->P_in_MikDiff, closest_to_origin->B->P_in_MikDiff, closest_to_origin->C->P_in_MikDiff, &coeff[0]);

	this->Points[0] = closest_to_origin->A->P_in_shapeA;
	this->Points[0].dot(coeff[0]);
	this->Points[0].add(coeff[1], closest_to_origin->B->P_in_shapeA);
	this->Points[0].add(coeff[2], closest_to_origin->C->P_in_shapeA);

	this->Points[1] = closest_to_origin->A->P_in_shapeB;
	this->Points[1].dot(coeff[0]);
	this->Points[1].add(coeff[1], closest_to_origin->B->P_in_shapeB);
	this->Points[1].add(coeff[2], closest_to_origin->C->P_in_shapeB);

}