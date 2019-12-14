/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/


#pragma once
#ifndef __GJK_EPA_H__
#define __GJK_EPA_H__


#define GEOMETRIC_TOLLERANCE float(HULL_GEOMETRIC_TOLLERANCE)

//#define ENABLE_LOG //when enabled, every time a proximity query is invoked (calling GJK_EPA::Are_in_collision, GJK_EPA::Get_penetration or GJK_EPA::Get_distance ) a JSON containing all the steps done by the solver is produced, see also the documentation. It should be used only for debugging purpose  
#ifdef ENABLE_LOG
#include <fstream>
#define log_location std::string("../Result_visualization/Log_JSON") //When ENABLE_LOG is defined, the debugging JSON (see above) is produced at the location specified in this string. Change the location to fit your need.
#endif
#include <list>
#include "Hull.h"



 /** \brief The solver to use for computing proximity queries on convex sets. 
* \details The pair oh shapes to consider, shape A and shape B, can be changed during time.
The proximity queries (distance or penetration computation) are computed considering the pair <shape A,shape B>.
In all the template methods V3d is a type representing a 3D Cartesian coordinate and must have at least:
 -> Three const getters returning each coordinate, defined in this way (with the obvious meaning of notation):
		 -> const float& V::x() const;
		 -> const float& V::y() const;
		 -> const float& V::z() const;

-> Three setters for the three coordinates:
		 -> void x(const float& x);
		 -> void y(const float& y);
		 -> void z(const float& z);
 */
class GJK_EPA {
public:
	GJK_EPA() :  Trasformation_A(NULL), Shape_A(NULL), Trasformation_B(NULL), Shape_B(NULL), State_Machine(0), Last_GJK(NULL) {  };
	~GJK_EPA() { this->__release(this->Shape_A); this->__release(this->Shape_B); this->__release(this->Trasformation_A); this->__release(this->Trasformation_B); this->__release(this->Last_GJK); };

	/** \brief Returns true if the last set pair of shapes is in collision, otherwise returns false.
	* \details The shapes can be set using GJK_EPA::Set_shape_A; GJK_EPA::Set_shape_B;
	GJK_EPA::Set_shape_A_transformed or GJK_EPA::Set_shape_B_transformed.
	In case at least one of the shape was not set, an exception is raised.
	*/
	bool Are_in_collision();

	/** \brief Returns the penetration distance of the last set pair of shapes.
	* \details  The shapes can be set using GJK_EPA::Set_shape_A; GJK_EPA::Set_shape_B;
	GJK_EPA::Set_shape_A_transformed or GJK_EPA::Set_shape_B_transformed.
	In case the set of pairs are not in collision, 0 is returned and the inputs are ignored (closest points)
	@param[out] return the penetration distance
	@param[out] closest_in_A When passed different from NULL, the closest point (see doc) in set A is returned
	@param[out] closest_in_B When passed different from NULL, the closest point (see doc) in set B is returned
	* /*/
	template<typename V3d>
	float Get_penetration(V3d* closest_in_A = NULL, V3d* closest_in_B = NULL);

	/** \brief Returns the distance between the last set pair of shapes.
	* \details  The shapes can be set using GJK_EPA::Set_shape_A; GJK_EPA::Set_shape_B;
	GJK_EPA::Set_shape_A_transformed or GJK_EPA::Set_shape_B_transformed.
	In case the set of pairs are in collision, 0 is returned and the inputs are ignored (closest points)
	@param[out] return the distance between the convex sets
	@param[out] closest_in_A When passed different from NULL, the closest point (see doc) in set A is returned
	@param[out] closest_in_B When passed different from NULL, the closest point (see doc) in set B is returned
	*/
	template<typename V3d>
	float Get_distance(V3d* closest_in_A = NULL, V3d* closest_in_B = NULL);

	/** \brief Set the shape A to consider.
	* \details  V3d is a template typename describing a 3d coordinate (see the comments above)
	@param[in] vertices Is the list of vertex characterizing the convex set to consider. The vertices are not internally copied: the passed list must be not deleted
	*/
	template<typename V3d>
	void Set_shape_A(const std::list<V3d>* vertices) { const V3d* temp = NULL; this->__Set_shape(vertices, temp, temp, &this->Shape_A, &this->Trasformation_A);  };

	/** \brief Set the shape B to consider.
	* \details  V3d is a template typename describing a 3d coordinate (see the comments above)
	@param[in] vertices Is the list of vertex characterizing the convex set to consider
	*/
	template<typename V3d>
	void Set_shape_B(const std::list<V3d>* vertices) { const V3d* temp = NULL; this->__Set_shape(vertices, temp, temp, &this->Shape_B, &this->Trasformation_B); };

	/** \brief Set the shape A to consider.
	* \details  An additional rototraslation is applied to the shape.
	 V3d is a template typename describing a 3d coordinate (see the comments above).
	@param[in] vertices Is the list of vertex characterizing the convex set to consider
	@param[in] rotation_angles Describes the rototation to consider for the shape. The order is as follows: 
	x:rotation along x axis -> y:rotation along y axis -> z:rotation along z axis
	@param[in] traslation Describes the traslation to consider for the shape along the x,y,z axis.
	*/
	template<typename V3d>
	void Set_shape_A_transformed(const std::list<V3d>* vertices, const V3d& rotation_angles, const V3d& traslation) { this->__Set_shape(vertices, &rotation_angles, &traslation, &this->Shape_A, &this->Trasformation_A); };

	/** \brief Set the shape B to consider.
	* \details  An additional rototraslation is applied to the shape.
	 V3d is a template typename describing a 3d coordinate (see the comments above).
	@param[in] vertices Is the list of vertex characterizing the convex set to consider
	@param[in] rotation_angles Describes the rototation to consider for the shape. The order is as follows:
	x:rotation along x axis -> y:rotation along y axis -> z:rotation along z axis
	@param[in] traslation Describes the traslation to consider for the shape along the x,y,z axis.
	*/
	template<typename V3d>
	void Set_shape_B_transformed(const std::list<V3d>* vertices, const V3d& rotation_angles, const V3d& traslation) { this->__Set_shape(vertices, &rotation_angles, &traslation, &this->Shape_B, &this->Trasformation_B); };

private:

	struct V {
	public:
		V() : V(0, 0, 0) {};
		V(const float& x, const float& y, const float& z) { this->_x = x; this->_y = y; this->_z = z; };

		float dot(const V& other) const { return this->_x*other._x + this->_y*other._y + this->_z*other._z; };
		void cross(const V& A, const V& B); //call on result variable
		void minus(const V& A, const V& B); //call on result variable
		float squared_norm() const { return this->dot(*this); };
		float squared_distance(const V& other) const;
		void normalize_in_place();
		void add(const float& coeff, const V& to_add); //call on result variable to obtain += coeff  *to_add
		void dot(const float& coeff) { this->_x *= coeff; this->_y *= coeff; this->_z *= coeff; };
		void invert() { this->_x = -this->_x; this->_y = -this->_y; this->_z = -this->_z; };

		// data
		float		_x;
		float		_y;
		float		_z;
	};

	struct __trasformation {
		__trasformation(const V& rotation_angles, const V& traslation) { this->reset(rotation_angles, traslation); };
		void reset(const V& rotation_angles, const V& traslation);
		void Per_Rtranspose(V* result, const V& Dx);
		void Per_R_T(V* result, const V& Dx);
		// data
		float			Rotation[3][3];
		float			Traslation[3];
	};

	class I_Convex_shape {
	public:
		virtual void Get_Support(V* result, const V& Direction) = 0;
		virtual ~I_Convex_shape() {};
#ifdef ENABLE_LOG
		virtual void Get_vertices(std::list<V>* points, __trasformation* trsf) = 0;
#endif 
	};

	template<typename V3d>
	class Convex_shape : public I_Convex_shape {
	public:
		Convex_shape(const std::list<V3d>* vertices, const float& radius = 0.f) : Vertices(vertices), Radius(radius) { if (this->Radius < GEOMETRIC_TOLLERANCE) this->Radius = 0.f; };
		void Get_Support(V* result, const V& Direction);
#ifdef ENABLE_LOG
		void Get_vertices(std::list<V>* points, __trasformation* trsf);
#endif 
	private:
	// data
		const std::list<V3d>* Vertices;
		float									Radius;
	};

	struct V3 {
		const float& x() const { return this->P_in_MikDiff._x; }; //used by the EPA algorithm
		const float& y() const { return this->P_in_MikDiff._y; }; //used by the EPA algorithm
		const float& z() const { return this->P_in_MikDiff._z; }; //used by the EPA algorithm

		V			P_in_MikDiff;
		V			P_in_shapeA;
		V			P_in_shapeB;
	};

	class Plex {
	public:
		Plex(GJK_EPA* emitter); //in the constructor the primal part is done
		~Plex() { for (size_t k = 0; k < 4; k++) delete this->plex[k]; };

		void finishing_loop(); //evolve till finding the closest plex to origin

		const bool& is_collision_present() { return this->collision_present; };
		void copy_plex_4EPA(std::list<V3>* plex);
	private:
		void __update(); 
		void __update_Origin_is_visible();

		void __set_plex_to_vertex();
		void __set_plex_to_segment(const size_t& id); //AB, AC, AE
		void __set_plex_to_facet(const size_t& id);  //ABC, ABE, ACE

// data
		GJK_EPA*					Emitter;
		V3*								plex[4];
		size_t							plex_dim;
		V									Direction;
		bool							collision_present;
// cache
		V									Normals[3];  //ABC, ABE, ACE
		bool							Origin_is_visible[3];  //ABC, ABE, ACE

		size_t							Check_triangle_origin; //4 indicates that was not updated
	};

	void __get_Support_Minkowski_diff(V* result, const V& Direction, V* support_A, V* support_B);
	template<typename R>
	void __release(R* resource) { if (resource != NULL) delete resource; };
	void EPA();
	template<typename V3d>
	void __Set_shape(const std::list<V3d>* vertices, const V3d* rotation_angles, const V3d* traslation, I_Convex_shape** shape, __trasformation** trsf);

	enum closest_element { vertex_A, edge_AB, edge_AC, face_ABC };
	static closest_element __get_closest_in_segment(const V& A, const V& B, float* miximg_coeff);
	static closest_element __get_closest_in_triangle(const V& A, const V& B, const V& C, float* miximg_coeff);
	static void __compute_outside_normal(V* N_result, const V& P1, const V& P2, const V& P3, const V& P_other);


// data
	I_Convex_shape*								Shape_A;
	__trasformation*							Trasformation_A;

	I_Convex_shape*								Shape_B;
	__trasformation*							Trasformation_B;

	size_t												State_Machine;
	V														Points[2];
	Plex*												Last_GJK;

#ifdef ENABLE_LOG
	std::ofstream*								log_stream;
	bool												bclose_log = true;

	static void log_points(const std::list<V>& L, std::ofstream* f);
	void close_log();
#endif
};

template<typename V3d>
void GJK_EPA::Convex_shape<V3d>::Get_Support(GJK_EPA::V* result, const GJK_EPA::V& Direction) {

	auto it = this->Vertices->begin();
	auto it_max = it;
	float dist, dist_max;
	dist_max = it->x()*Direction._x + it->y()*Direction._y + it->z()*Direction._z;
	it++;
	auto it_end = this->Vertices->end();
	for (it; it != it_end; it++) {
		dist = it->x()*Direction._x + it->y()*Direction._y + it->z()*Direction._z;
		if (dist > dist_max) {
			dist_max = dist;
			it_max = it;
		}
	}

	result->_x = it_max->x();
	result->_y= it_max->y();
	result->_z = it_max->z();

	if (this->Radius != 0.f) {
		dist = Direction.dot(Direction);
		dist = this->Radius / sqrtf(dist);

		result->_x += dist * Direction._x;
		result->_y += dist * Direction._y;
		result->_z += dist * Direction._z;
	}

}

template<typename V3d>
void GJK_EPA::__Set_shape(const std::list<V3d>* vertices, const V3d* rotation_angles, const V3d* traslation, I_Convex_shape** shape, __trasformation** trsf) {

	if (rotation_angles == NULL) {
		this->__release(*trsf);
		*trsf = NULL;
	}
	else {
		if (*trsf == NULL) *trsf = new __trasformation(V(rotation_angles->x(), rotation_angles->y(), rotation_angles->z()), V(traslation->x(), traslation->y(), traslation->z()));
		else (*trsf)->reset(V(rotation_angles->x(), rotation_angles->y(), rotation_angles->z()), V(traslation->x(), traslation->y(), traslation->z()));
	}

	this->__release(*shape);
	*shape = new Convex_shape<V3d>(vertices);

	this->State_Machine = 0;
	this->__release(this->Last_GJK);
	this->Last_GJK = NULL;

}

template<typename V3d>
float  GJK_EPA::Get_penetration(V3d* closest_in_A, V3d* closest_in_B) {

	if ((this->State_Machine == 3) || (this->State_Machine == 4)) return 0.f;

	if (this->State_Machine == 2) {
		if (closest_in_A != NULL) {
			closest_in_A->x(this->Points[0]._x);
			closest_in_A->y(this->Points[0]._y);
			closest_in_A->z(this->Points[0]._z);
		}
		if (closest_in_B != NULL) {
			closest_in_B->x(this->Points[1]._x);
			closest_in_B->y(this->Points[1]._y);
			closest_in_B->z(this->Points[1]._z);
		}
		return sqrtf(this->Points[0].squared_distance(this->Points[1]));
	}
	else if (this->State_Machine == 1) {
		this->EPA();
#ifdef ENABLE_LOG
		*this->log_stream << ",\"GJK_finish\":[0]\n";
		this->close_log();
#endif
		this->State_Machine = 2;
		return this->Get_penetration(closest_in_A, closest_in_B);
	}

#ifdef ENABLE_LOG
	this->bclose_log = false;
#endif
	this->Are_in_collision();
#ifdef ENABLE_LOG
	this->bclose_log = true;
#endif
	return this->Get_penetration(closest_in_A, closest_in_B);

}

template<typename V3d>
float GJK_EPA::Get_distance(V3d* closest_in_A, V3d* closest_in_B) {

	if ((this->State_Machine == 1) || (this->State_Machine == 2)) return 0.f;

	if (this->State_Machine == 4) {
		if (closest_in_A != NULL) {
			closest_in_A->x(this->Points[0]._x);
			closest_in_A->y(this->Points[0]._y);
			closest_in_A->z(this->Points[0]._z);
		}
		if (closest_in_B != NULL) {
			closest_in_B->x(this->Points[1]._x);
			closest_in_B->y(this->Points[1]._y);
			closest_in_B->z(this->Points[1]._z);
		}
		return sqrtf(this->Points[0].squared_distance(this->Points[1]));
	}
	else if (this->State_Machine == 3) {
		this->Last_GJK->finishing_loop();
#ifdef ENABLE_LOG
		*this->log_stream << ",\"EPA\":[0]\n";
		this->close_log();
#endif
		this->State_Machine = 4;
		return this->Get_distance(closest_in_A, closest_in_B);
	}

#ifdef ENABLE_LOG
	this->bclose_log = false;
#endif
	this->Are_in_collision();
#ifdef ENABLE_LOG
	this->bclose_log = true;
#endif
	return this->Get_distance(closest_in_A, closest_in_B);

}

#ifdef  ENABLE_LOG
template<typename V3d>
void  GJK_EPA::Convex_shape<V3d>::Get_vertices(std::list<V>* points, __trasformation* trsf) {

	points->clear();
	for (auto it= this->Vertices->begin(); it != this->Vertices->end(); it++)
		points->push_back(V(it->x(), it->y(), it->z()));
	if (trsf != NULL) {
		for (auto it2 = points->begin(); it2 != points->end(); it2++)
			trsf->Per_R_T(&(*it2), *it2);
	}

};
#endif 

#endif