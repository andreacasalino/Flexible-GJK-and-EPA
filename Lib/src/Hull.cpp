/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Hull.h"
#include <Error.h>

namespace flx::hull {
	constexpr int COEFF_NORMAL_DIRECTION = 1e6;

	Hull::Hull(const Coordinate& A, const Coordinate& B, const  Coordinate& C, const Coordinate& D, const Observer* obs) { 
		this->observer = obs;
		// init tethraedron and check the thetraedron has a non zero volume
		Coordinate delta_1;
		diff(delta_1, D, A);
		Coordinate delta_2;
		diff(delta_2, D, B);
		Coordinate delta_3;
		diff(delta_3, D, C);

		Coordinate cross_1_2;
		cross(cross_1_2, delta_1, delta_2);		
		if (abs(dot(cross_1_2, delta_3)) < HULL_GEOMETRIC_TOLLERANCE) throw Error("intial thetraedron volume too small: make sure the convex shape is actually a 3d shape");

		//computation of the midpoint of the thetraedron
		this->Mid_point.x = 0.25f * (A.x + B.x + C.x + D.x);
		this->Mid_point.y = 0.25f * (A.y + B.y + C.y + D.y);
		this->Mid_point.z = 0.25f * (A.z + B.z + C.z + D.z);

		Coordinate& pA = this->vertices.emplace_back(A);
		Coordinate& pB = this->vertices.emplace_back(B);
		Coordinate& pC = this->vertices.emplace_back(C);
		Coordinate& pD = this->vertices.emplace_back(D);

		//build the tethraedron
		//ABC->0; ABD->1; ACD->2; BCD->3
		std::list<const Facet*> initialFacets;
		this->AppendFacet(pA, pB, pC);
		initialFacets.push_back(&this->Facets.back());
		this->AppendFacet(pA, pB, pD);
		initialFacets.push_back(&this->Facets.back());
		this->AppendFacet(pA, pC, pD);
		initialFacets.push_back(&this->Facets.back());
		this->AppendFacet(pB, pC, pD);
		initialFacets.push_back(&this->Facets.back());

		auto itFace_ABC = this->Facets.begin();
		auto itFace_ABD = itFace_ABC; ++itFace_ABD;
		auto itFace_ACD = itFace_ABD; ++itFace_ACD;
		auto itFace_BCD = itFace_ACD; ++itFace_BCD;

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

		if(nullptr != this->observer) this->observer->AddedChangedFacets( initialFacets, {});
	};

	void Hull::AppendFacet(const Coordinate& vertexA, const Coordinate& vertexB, const Coordinate& vertexC) {
		this->Facets.emplace_back();
		this->Facets.back().bVisible = false;
		this->Facets.back().A = &vertexA;
		this->Facets.back().B = &vertexB;
		this->Facets.back().C = &vertexC;
		this->RecomputeNormal(this->Facets.back());
	}

	void Hull::RecomputeNormal(Facet& facet) {
		Coordinate delta1, delta2;
		diff(delta1, *facet.A, *facet.C);
		diff(delta2, *facet.B, *facet.C);

		cross(facet.N, delta1, delta2);

		diff(delta1, this->Mid_point, *facet.A);
		float dot_normal = dot(facet.N, delta1);
		if (dot_normal >= 0.f) {
			facet.N.x = facet.N.x;
			facet.N.y = facet.N.y;
			facet.N.z = facet.N.z;
		}

		//normalize
		dot_normal = normSquared(facet.N);
		if (dot_normal < 1e-7) {
			facet.N.x = COEFF_NORMAL_DIRECTION * facet.N.x;
			facet.N.y = COEFF_NORMAL_DIRECTION * facet.N.y;
			facet.N.z = COEFF_NORMAL_DIRECTION * facet.N.z;
		}
		else {
			dot_normal = 1.f / dot_normal;
			facet.N.x = dot_normal * facet.N.x;
			facet.N.y = dot_normal * facet.N.y;
			facet.N.z = dot_normal * facet.N.z;
		}
	}

	void Replace(Facet& involved_facet, Facet& old_neigh, Facet& new_neigh) {
		if (&old_neigh == involved_facet.Neighbour[0]) involved_facet.Neighbour[0] = &new_neigh;
		else {
			if (&old_neigh == involved_facet.Neighbour[1]) involved_facet.Neighbour[1] = &new_neigh;
			else involved_facet.Neighbour[2] = &new_neigh;
		}
	};

	void Hull::_UpdateHull(const Coordinate& vertex_of_new_cone, Facet& starting_facet_for_expansion) {
		starting_facet_for_expansion.bVisible = true;
		const Coordinate* Point = &this->vertices.emplace_back(vertex_of_new_cone);
		//find the group of visible faces
		std::list<Facet*> Visible_group;
		Visible_group.push_back(&starting_facet_for_expansion);
		auto itN = Visible_group.begin();
		size_t kNeigh = 0, k;
		float distance;
		while (kNeigh < Visible_group.size()) {
			for (k = 0; k < 3; ++k) {
				if (!(*itN)->Neighbour[k]->bVisible) { //this neighbour facet is not already present in Visible_Group
					distance = (*itN)->Neighbour[k]->N.x * (Point->x - (*itN)->Neighbour[k]->A->x);
					distance += (*itN)->Neighbour[k]->N.y * (Point->y - (*itN)->Neighbour[k]->A->y);
					distance += (*itN)->Neighbour[k]->N.z * (Point->z - (*itN)->Neighbour[k]->A->z);
					if (distance > HULL_GEOMETRIC_TOLLERANCE) {
						(*itN)->Neighbour[k]->bVisible = true;
						Visible_group.push_back((*itN)->Neighbour[k]);
					}
				}
			}
			++kNeigh;
			++itN;
		}
		//Update Hull. Build the cone of new facets
		const Coordinate* A, *B, *C;
		Facet* AB, *BC, *CA;
		itN = Visible_group.begin();
		size_t kboard;
		std::list<const Facet*> removed, changed, created;
		for (k = 0; k < Visible_group.size(); ++k) {
			kboard = 0;
			A = (*itN)->A; B = (*itN)->B; C = (*itN)->C;
			AB = (*itN)->Neighbour[0]; BC = (*itN)->Neighbour[1]; CA = (*itN)->Neighbour[2];
			//AB
			if (!AB->bVisible) { //edge AB is part of the board of the cone
				(*itN)->C = Point;
				this->RecomputeNormal(**itN);
				++kboard;
			}
			//BC
			if (!BC->bVisible) { //edge BC is part of the board of the cone
				if (kboard == 0) {
					(*itN)->A = B;
					(*itN)->B = C;
					(*itN)->C = Point;
					this->RecomputeNormal(**itN);
					(*itN)->Neighbour[0] = BC;
				}
				else {
					this->AppendFacet(*B, *C, *Point);
					this->Facets.back().bVisible = true;
					Visible_group.push_back(&this->Facets.back());
					created.push_back(&this->Facets.back());
					Visible_group.back()->Neighbour[0] = BC;
					Replace(*BC, **itN, *Visible_group.back());
				}
				++kboard;
			}
			//CA
			if (!CA->bVisible) { //edge CA is part of the board of the cone
				if (kboard == 0) {
					(*itN)->A = A;
					(*itN)->B = C;
					(*itN)->C = Point;
					this->RecomputeNormal(**itN);
					(*itN)->Neighbour[0] = CA;
				}
				else {
					this->AppendFacet(*A, *C, *Point);
					this->Facets.back().bVisible = true;
					Visible_group.push_back(&this->Facets.back());
					created.push_back(&this->Facets.back());
					Visible_group.back()->Neighbour[0] = CA;
					Replace(*CA, **itN, *Visible_group.back());
				}
				++kboard;
			}
			if (kboard == 0) {
				(*itN)->Neighbour[0] = nullptr;
				removed.push_back(*itN);
				itN = Visible_group.erase(itN);
			}
			else {
				changed.push_back(*itN);
				++itN;
			}
		}
		if(nullptr != this->observer) {
			// notify to observer 
			this->observer->RemovedFacets(removed);
		}		
		//remove facet for which Neighbour[0]=nullptr
		auto itF = this->Facets.begin();
		while (itF != this->Facets.end()) {
			if (nullptr == itF->Neighbour[0]) itF = this->Facets.erase(itF);
			else ++itF;
		}
		//delete old neighbour info for the visible group
		for (itN = Visible_group.begin(); itN != Visible_group.end(); ++itN) {
			(*itN)->Neighbour[1] = nullptr; (*itN)->Neighbour[2] = nullptr;
		}
		//update neighbour info for the visible group 
		auto itN2 = itN;
		for (itN = Visible_group.begin(); itN != Visible_group.end(); ++itN) {
			if (nullptr == (*itN)->Neighbour[1]) { //find neighbour of edge BC
				for (itN2 = Visible_group.begin(); itN2 != Visible_group.end(); ++itN2) {
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

			if (nullptr == (*itN)->Neighbour[2]) { //find neighbour of edge CA
				for (itN2 = Visible_group.begin(); itN2 != Visible_group.end(); ++itN2) {
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
		// clean up
		for (itN = Visible_group.begin(); itN != Visible_group.end(); ++itN) (*itN)->bVisible = false;
		if(nullptr != this->observer) {
			// notify to observer 
			this->observer->AddedChangedFacets(created, changed);
		}		
	}

	void Hull::UpdateHull(const Coordinate& vertex_of_new_cone) {
		auto it = this->Facets.begin();
		float distance;
		for (it; it != this->Facets.end(); ++it) {
			distance = it->N.x * (vertex_of_new_cone.x - it->A->x);
			distance += it->N.y * (vertex_of_new_cone.y - it->A->y);
			distance += it->N.z * (vertex_of_new_cone.z - it->A->z);
			if (distance > HULL_GEOMETRIC_TOLLERANCE) {
				this->_UpdateHull(vertex_of_new_cone, *it);			
				return;
			}
		}
	}

	void Hull::UpdateHull(const Coordinate& vertex_of_new_cone, const Facet& starting_facet_for_expansion) {
		Facet* starting = nullptr;
		for(auto it = this->Facets.begin(); it!=this->Facets.end(); ++it) {
			if(&starting_facet_for_expansion == &(*it)) {
				starting = &(*it);
				break;
			}
		}
		if(nullptr == starting) throw Error("the passed facet is not contained in this hull");
		float distance = starting->N.x * (vertex_of_new_cone.x - starting->A->x);
		distance += starting->N.y * (vertex_of_new_cone.y - starting->A->y);
		distance += starting->N.z * (vertex_of_new_cone.z - starting->A->z);
		if (distance <= HULL_GEOMETRIC_TOLLERANCE) {
			throw Error("the passed facet is not visible from the passed vertex");
		}
		this->_UpdateHull(vertex_of_new_cone, *starting);
	}
}
