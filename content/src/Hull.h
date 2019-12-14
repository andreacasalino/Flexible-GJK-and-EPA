/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef __HULL_H__
#define __HULL_H__

#include <list>

#define HULL_GEOMETRIC_TOLLERANCE (float)1e-3

//Facet must contain the info for a facet, i.e. at least:
// {
//	bool					bVisible; //only for update Hull
//
//	const V*				A; //pointer of the vertex A
//	const V*				B; //pointer of the vertex B
//	const V*				C; //pointer of the vertex C
//
//size_t					id_A;// identifier of vertex A(if not passed 0 for all points is assumed)
//size_t					id_B;// identifier of vertex B(if not passed 0 for all points is assumed)
//size_t					id_C;// identifier of vertex C(if not passed 0 for all points is assumed)
//
//	float						N[3]; //outer normal 
//
//	Facet*				Neighbour[3]; // AB, BC, CA
//};

//V should be a description of a 3d vector containing at least 3 const getters of the coordinates
//V{
// const float& x() const;
// const float& y() const;
// const float& z() const;
//}
template<typename Facet, typename V>
class Hull {
public:
	Hull(const V* A, const V* B, const  V* C, const V* D, const size_t& A_id = 0, const size_t& B_id = 0, const size_t& C_id = 0, const size_t& D_id = 0 ) { this->__init_thetraedron(A, B, C, D, A_id, B_id, C_id, D_id); };

protected:

	void __Update_Hull(const V* vertex_of_new_cone, Facet* starting_facet_for_expansion,  const size_t& vertex_id = 0, std::list<Facet*>* created_cone = NULL);
	void __Update_Hull(const V* vertex_of_new_cone, std::list<Facet*>* created_cone = NULL);

// data
	std::list<Facet>					Facets;
private:
	static void __Replace(Facet* involved_facet, Facet* old_neigh, Facet* new_neigh);
	void __init_thetraedron(const V* A, const V* B, const  V* C, const  V* D, const size_t& A_id , const size_t& B_id , const size_t& C_id , const size_t& D_id);
	void __Append_facet(const V* vertexA, const V* vertexB, const V* vertexC, const size_t& A_id, const size_t& B_id, const size_t& C_id);
	void __Recompute_Normal(Facet* facet);
// data
	float											Mid_point[3];
};


///////////////////////////////////////////////////////
//							Implementations									 //
///////////////////////////////////////////////////////


template<typename Facet, typename V>
void Hull<Facet, V>::__init_thetraedron(const V* A, const  V* B, const V* C, const  V* D, const size_t& A_id, const size_t& B_id, const size_t& C_id, const size_t& D_id) {

	//check the thetraedron has a non zero volume
	float delta_1[3];
	delta_1[0] = D->x() - A->x();
	delta_1[1] = D->y() - A->y();
	delta_1[2] = D->z() - A->z();
	float delta_2[3];
	delta_2[0] = D->x() - B->x();
	delta_2[1] = D->y() - B->y();
	delta_2[2] = D->z() - B->z();
	float delta_3[3];
	delta_3[0] = D->x() - C->x();
	delta_3[1] = D->y() - C->y();
	delta_3[2] = D->z() - C->z();

	float cross_1_2[3];
	cross_1_2[0] = delta_1[1] * delta_2[2] - delta_1[2] * delta_2[1];
	cross_1_2[1] = delta_1[2] * delta_2[0] - delta_1[0] * delta_2[2];
	cross_1_2[2] = delta_1[0] * delta_2[1] - delta_1[1] * delta_2[0];
	float volume = cross_1_2[0] * delta_3[0] + cross_1_2[1] * delta_3[1] + cross_1_2[2] * delta_3[2];
	if (abs(volume) < HULL_GEOMETRIC_TOLLERANCE) 
		throw 0;

	//computation of the midpoint of the thetraedron
	this->Mid_point[0] = 0.25f * (A->x() + B->x() + C->x() + D->x());
	this->Mid_point[1] = 0.25f * (A->y() + B->y() + C->y() + D->y());
	this->Mid_point[2] = 0.25f * (A->z() + B->z() + C->z() + D->z());

	//build the tethraedron
	//ABC->0; ABD->1; ACD->2; BCD->3
	this->__Append_facet(A, B, C, A_id, B_id, C_id);
	this->__Append_facet(A, B, D, A_id, B_id, D_id);
	this->__Append_facet(A, C, D, A_id, C_id, D_id);
	this->__Append_facet(B, C, D, B_id, C_id, D_id);

	auto itFace_ABC = this->Facets.begin();
	auto itFace_ABD = itFace_ABC; itFace_ABD++;
	auto itFace_ACD = itFace_ABD; itFace_ACD++;
	auto itFace_BCD = itFace_ACD; itFace_BCD++;

	//ABC
	itFace_ABC->Neighbour[0] = &(*itFace_ABD);
	itFace_ABC->Neighbour[1] = &(*itFace_BCD);
	itFace_ABC->Neighbour[2] = &(*itFace_ACD);

	//ABD
	itFace_ABD->Neighbour[0] = &(*itFace_ABC);
	itFace_ABD->Neighbour[1] = &(*itFace_BCD);
	itFace_ABD->Neighbour[2] = &(*itFace_ACD);

	//ACD
	itFace_ACD->Neighbour[0] = &(*itFace_ABC);
	itFace_ACD->Neighbour[1] = &(*itFace_BCD);
	itFace_ACD->Neighbour[2] = &(*itFace_ABD);

	//BCD
	itFace_BCD->Neighbour[0] = &(*itFace_ABC);
	itFace_BCD->Neighbour[1] = &(*itFace_ACD);
	itFace_BCD->Neighbour[2] = &(*itFace_ABD);

}

template<typename Facet, typename V>
void Hull<Facet, V>::__Recompute_Normal(Facet* facet) {

	float delta1[3], delta2[3];
	delta1[0] = facet->A->x() - facet->C->x();
	delta1[1] = facet->A->y() - facet->C->y();
	delta1[2] = facet->A->z() - facet->C->z();

	delta2[0] = facet->B->x() - facet->C->x();
	delta2[1] = facet->B->y() - facet->C->y();
	delta2[2] = facet->B->z() - facet->C->z();

	facet->N[0] = delta1[1] * delta2[2] - delta1[2] * delta2[1];
	facet->N[1] = delta1[2] * delta2[0] - delta1[0] * delta2[2];
	facet->N[2] = delta1[0] * delta2[1] - delta1[1] * delta2[0];

	float dot_normal;
	delta1[0] = this->Mid_point[0] - facet->A->x();
	delta1[1] = this->Mid_point[1] - facet->A->y();
	delta1[2] = this->Mid_point[2] - facet->A->z();
	dot_normal = delta1[0] * facet->N[0] + delta1[1] * facet->N[1] + delta1[2] * facet->N[2];
	if (dot_normal >= 0.f) {
		facet->N[0] = -facet->N[0];
		facet->N[1] = -facet->N[1];
		facet->N[2] = -facet->N[2];
	}

	//normalize
	dot_normal = sqrtf(facet->N[0] * facet->N[0] + facet->N[1] * facet->N[1] + facet->N[2] * facet->N[2]);
	if (dot_normal < 1e-7) {
		facet->N[0] = (float)1e6 * facet->N[0];
		facet->N[1] = (float)1e6  * facet->N[1];
		facet->N[2] = (float)1e6  * facet->N[2];
	}
	else {
		dot_normal = 1.f / dot_normal;
		facet->N[0] = dot_normal * facet->N[0];
		facet->N[1] = dot_normal * facet->N[1];
		facet->N[2] = dot_normal * facet->N[2];
	}

}

template<typename Facet, typename V>
void Hull<Facet, V>::__Append_facet(const V* vertexA, const V* vertexB, const V* vertexC, const size_t& A_id, const size_t& B_id, const size_t& C_id) {

	Facet new_face;
	new_face.bVisible = false;
	new_face.A = vertexA;
	new_face.B = vertexB;
	new_face.C = vertexC;

	new_face.id_A = A_id;
	new_face.id_B = B_id;
	new_face.id_C = C_id;

	//normal computation
	this->__Recompute_Normal(&new_face);

	this->Facets.push_back(new_face);

}

template<typename Facet, typename V>
void Hull<Facet, V>::__Replace(Facet* involved_facet, Facet* old_neigh, Facet* new_neigh) {
	if (old_neigh == involved_facet->Neighbour[0]) involved_facet->Neighbour[0] = new_neigh;
	else {
		if (old_neigh == involved_facet->Neighbour[1]) involved_facet->Neighbour[1] = new_neigh;
		else involved_facet->Neighbour[2] = new_neigh;
	}
};

template<typename Facet, typename V>
void Hull<Facet, V>::__Update_Hull(const V* vertex_of_new_cone, Facet* starting_facet_for_expansion, const size_t& vertex_id, std::list<Facet*>* created_cone) {

	starting_facet_for_expansion->bVisible = true;
	const V* Point = vertex_of_new_cone;

	//find the group of visible faces
	std::list<Facet*> Visible_group;
	Visible_group.push_back(starting_facet_for_expansion);
	auto itN = Visible_group.begin();
	size_t kNeigh = 0, k;
	float distance;
	while (kNeigh < Visible_group.size()) {
		for (k = 0; k < 3; k++) {
			if (!(*itN)->Neighbour[k]->bVisible) { //this neighbour facet is not already present in Visible_Group
				distance = (*itN)->Neighbour[k]->N[0] * (Point->x() - (*itN)->Neighbour[k]->A->x());
				distance += (*itN)->Neighbour[k]->N[1] * (Point->y() - (*itN)->Neighbour[k]->A->y());
				distance += (*itN)->Neighbour[k]->N[2] * (Point->z() - (*itN)->Neighbour[k]->A->z());
				if (distance > HULL_GEOMETRIC_TOLLERANCE) {
					(*itN)->Neighbour[k]->bVisible = true;
					Visible_group.push_back((*itN)->Neighbour[k]);
				}
			}
		}
		kNeigh++;
		itN++;
	}
	size_t dim_visible = Visible_group.size();

	//Update Hull. Build the cone of new facets
	const V* A, *B, *C;
	Facet* AB, *BC, *CA;
	size_t pos_vertex_old[3];
	itN = Visible_group.begin();
	size_t kboard;
	bool delete_face = false;
	for (k = 0; k < dim_visible; k++) {
		kboard = 0;
		A = (*itN)->A; B = (*itN)->B; C = (*itN)->C;
		AB = (*itN)->Neighbour[0]; BC = (*itN)->Neighbour[1]; CA = (*itN)->Neighbour[2];
		pos_vertex_old[0] = (*itN)->id_A; pos_vertex_old[1] = (*itN)->id_B; pos_vertex_old[2] = (*itN)->id_C;

		//AB
		if (!AB->bVisible) { //edge AB is part of the board of the cone
			(*itN)->C = Point;
			(*itN)->id_C = vertex_id;
			this->__Recompute_Normal(*itN);
			kboard++;
		}
		//BC
		if (!BC->bVisible) { //edge BC is part of the board of the cone
			if (kboard == 0) {
				(*itN)->A = B;
				(*itN)->B = C;
				(*itN)->C = Point;
				(*itN)->id_A = pos_vertex_old[1];
				(*itN)->id_B = pos_vertex_old[2];
				(*itN)->id_C = vertex_id;
				this->__Recompute_Normal(*itN);
				(*itN)->Neighbour[0] = BC;
			}
			else {
				this->__Append_facet(B, C, Point, pos_vertex_old[1], pos_vertex_old[2], vertex_id);
				this->Facets.back().bVisible = true;
				Visible_group.push_back(&this->Facets.back());
				Visible_group.back()->Neighbour[0] = BC;
				this->__Replace(BC, *itN, Visible_group.back());
			}
			kboard++;
		}
		//CA
		if (!CA->bVisible) { //edge CA is part of the board of the cone
			if (kboard == 0) {
				(*itN)->A = A;
				(*itN)->B = C;
				(*itN)->C = Point;
				(*itN)->id_A = pos_vertex_old[0];
				(*itN)->id_B = pos_vertex_old[2];
				(*itN)->id_C = vertex_id;
				this->__Recompute_Normal(*itN);
				(*itN)->Neighbour[0] = CA;
			}
			else {
				this->__Append_facet(A, C, Point, pos_vertex_old[0], pos_vertex_old[2], vertex_id);
				this->Facets.back().bVisible = true;
				Visible_group.push_back(&this->Facets.back());
				Visible_group.back()->Neighbour[0] = CA;
				this->__Replace(CA, *itN, Visible_group.back());
			}
			kboard++;
		}

		if (kboard == 0) {
			delete_face = true;
			(*itN)->Neighbour[0] = NULL;
			itN = Visible_group.erase(itN);
		}
		else itN++;
	}

	if (delete_face) { //remove facet for which Neighbour[0]=NULL
		auto itF = this->Facets.begin();
		while (itF != this->Facets.end()) {
			if (itF->Neighbour[0] == NULL)
				itF = this->Facets.erase(itF);
			else
				itF++;
		}

	}


	//delete old neighbour info for the visible group
	auto Vis_end = Visible_group.end();
	for (itN = Visible_group.begin(); itN != Vis_end; itN++) {
		(*itN)->Neighbour[1] = NULL; (*itN)->Neighbour[2] = NULL;
	}

	//update neighbour info for the visible group 
	auto itN2 = itN;
	for (itN = Visible_group.begin(); itN != Vis_end; itN++) {
		if ((*itN)->Neighbour[1] == NULL) { //find neighbour of edge BC
			for (itN2 = Visible_group.begin(); itN2 != Vis_end; itN2++) {
				if (*itN2 != *itN) {
					if ((*itN2)->A == (*itN)->B) {
						(*itN)->Neighbour[1] = *itN2;
						(*itN2)->Neighbour[2] = *itN;
						break;
					}

					if ((*itN2)->B == (*itN)->B) {
						(*itN)->Neighbour[1] = *itN2;
						(*itN2)->Neighbour[1] = *itN;
						break;
					}
				}
			}
		}

		if ((*itN)->Neighbour[2] == NULL) { //find neighbour of edge CA
			for (itN2 = Visible_group.begin(); itN2 != Vis_end; itN2++) {
				if (*itN2 != *itN) {
					if ((*itN2)->A == (*itN)->A) {
						(*itN)->Neighbour[2] = *itN2;
						(*itN2)->Neighbour[2] = *itN;
						break;
					}

					if ((*itN2)->B == (*itN)->A) {
						(*itN)->Neighbour[2] = *itN2;
						(*itN2)->Neighbour[1] = *itN;
						break;
					}
				}
			}
		}
	}

	if (created_cone != NULL)
		*created_cone = Visible_group;

	for (itN = Visible_group.begin(); itN != Vis_end; itN++)
		(*itN)->bVisible = false;

}

template<typename Facet, typename V>
void Hull<Facet, V>::__Update_Hull(const V* Point, std::list<Facet*>* created_cone) {

	auto it = this->Facets.begin();
	auto it_end = this->Facets.end();
	float distance;
	for (it; it != it_end; it++) {
		distance = it->N[0] * (Point->x() - it->A->x());
		distance += it->N[1] * (Point->y() - it->A->y());
		distance += it->N[2] * (Point->z() - it->A->z());
		if (distance > HULL_GEOMETRIC_TOLLERANCE) {
			this->__Update_Hull(Point, &(*it), 0, created_cone);
			break;
		}
	}

}

#endif