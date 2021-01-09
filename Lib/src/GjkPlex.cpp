/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "GjkPlex.h"
#include <float.h>

namespace flx {
	GjkEpa::Plex::Plex(GjkEpa& user, const ShapePair& pair)
		: pair(pair)
		, user(user) {
		for (size_t k = 0; k < 4; ++k) this->vertices[k] = std::make_unique<MinkowskiCoordinate>();
		this->user.searchDirection = {1.f, 0.f, 0.f};
		this->user.getSupportMinkowskiDiff(this->pair, *this->vertices[1]);
		if(normSquared(this->vertices[1]->vertexDiff) <= GEOMETRIC_TOLLERANCE2) {
			this->collision_present = true;
			return;
		}
		this->user.searchDirection = this->vertices[0]->vertexDiff;
		invert(this->user.searchDirection);
		normalizeInPlace(this->user.searchDirection);
		Coordinate temp;
		while (true) {
			this->user.getSupportMinkowskiDiff(this->pair, *this->vertices[0]);
			if(dot(this->vertices[0]->vertexDiff, this->user.searchDirection) <= GEOMETRIC_TOLLERANCE) {
				this->collision_present = false;
				return;
			}
			++this->plex_dim;
			//check if plex contains origin
			switch (this->plex_dim) {
			case 4:
				this->updateOriginVisibilityFlags();
				if (!(this->Origin_is_visible[0] && this->Origin_is_visible[1] && this->Origin_is_visible[2])) {
					this->collision_present = true;
					--this->plex_dim;
					return;
				}
				break;
			case 3:
				float coeff[3];
				this->Check_triangle_origin = getClosestInTriangle(this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff, this->vertices[2]->vertexDiff, &coeff[0]);
				mix3(temp, this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff, this->vertices[2]->vertexDiff, coeff);
				if (normSquared(temp) <= GEOMETRIC_TOLLERANCE2) {
					this->collision_present = true;
					--this->plex_dim;
					return;
				}
				break;
			case 2:
				cross(temp, this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff);
				if (normSquared(temp) <= GEOMETRIC_TOLLERANCE4) {
					this->collision_present = true;
					--this->plex_dim;
					return;
				}
				break;
			default:
				break;
			}
			this->update();
		}
	}

	void GjkEpa::Plex::finishingLoop(CoordinatePair& closestPoints) {
		Coordinate delta;
		this->Check_triangle_origin = 4;
		diff(delta, this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff);
		if (dot(this->user.searchDirection, delta) <= GEOMETRIC_TOLLERANCE) {
			--this->plex_dim;
		}
		else {
			if (4 == this->plex_dim) this->updateOriginVisibilityFlags();
			this->update();
			while (true) {
				this->user.getSupportMinkowskiDiff(this->pair, *this->vertices[0]);
				++this->plex_dim;
				diff(delta, this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff);
				if (dot(this->user.searchDirection, delta) <= GEOMETRIC_TOLLERANCE) {
					--this->plex_dim;
					break;
				}
				if (4 == this->plex_dim == 4) this->updateOriginVisibilityFlags();
				this->update();
			}
		}
		if ((1 == this->plex_dim) || (0 == this->plex_dim)) {
			closestPoints.pointA = this->vertices[0]->vertexA;
			closestPoints.pointB = this->vertices[0]->vertexB;
			return;
		}
		if(2 == this->plex_dim) {
			float coeff[3];
			getClosestInSegment(this->vertices[1]->vertexDiff, this->vertices[2]->vertexDiff, coeff);
			mix2(closestPoints.pointA, this->vertices[1]->vertexA, this->vertices[2]->vertexA, coeff);
			mix2(closestPoints.pointB, this->vertices[1]->vertexB, this->vertices[2]->vertexB, coeff);
			return;
		}
		float coeff[3];
		getClosestInTriangle(this->vertices[1]->vertexDiff, this->vertices[2]->vertexDiff, this->vertices[3]->vertexDiff, coeff);
		mix3(closestPoints.pointA, this->vertices[1]->vertexA, this->vertices[2]->vertexA, this->vertices[3]->vertexA, coeff);
		mix3(closestPoints.pointB, this->vertices[1]->vertexB, this->vertices[2]->vertexB, this->vertices[3]->vertexB, coeff);
	}

	void GjkEpa::Plex::updateOriginVisibilityFlags() {
		computeOutsideNormal(this->Normals[0], this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff, this->vertices[2]->vertexDiff, this->vertices[3]->vertexDiff);
		if (dot(this->Normals[0], this->vertices[0]->vertexDiff) <= GEOMETRIC_TOLLERANCE) this->Origin_is_visible[0] = true;
		else this->Origin_is_visible[0] = false;

		computeOutsideNormal(this->Normals[1], this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff, this->vertices[3]->vertexDiff, this->vertices[2]->vertexDiff);
		if (dot(this->Normals[1], this->vertices[0]->vertexDiff) <= GEOMETRIC_TOLLERANCE) this->Origin_is_visible[1] = true;
		else this->Origin_is_visible[1] = false;

		computeOutsideNormal(this->Normals[2], this->vertices[0]->vertexDiff, this->vertices[2]->vertexDiff, this->vertices[3]->vertexDiff, this->vertices[1]->vertexDiff);
		if (dot(this->Normals[2], this->vertices[0]->vertexDiff) <= GEOMETRIC_TOLLERANCE) this->Origin_is_visible[2] = true;
		else this->Origin_is_visible[2] = false;
	}

	struct FacetIncidences {
		struct __facet_inc {
			std::uint8_t pos[3];
		};

		FacetIncidences()  {
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
	};
	static const FacetIncidences INCIDENCES;
	void GjkEpa::Plex::update() {
		float coeff[3];
		if(4 == this->plex_dim) {
			ClosestElement regions[3];
			float distances[3];
			Coordinate temp;
			for (std::uint8_t k = 0; k < 3; ++k) {
				if (this->Origin_is_visible[k]) {
					regions[k] = getClosestInTriangle(this->vertices[INCIDENCES.Inc[k].pos[0]]->vertexDiff,
													  this->vertices[INCIDENCES.Inc[k].pos[1]]->vertexDiff,
													  this->vertices[INCIDENCES.Inc[k].pos[2]]->vertexDiff, coeff);
					mix3(temp, this->vertices[INCIDENCES.Inc[k].pos[0]]->vertexDiff, 
							   this->vertices[INCIDENCES.Inc[k].pos[1]]->vertexDiff, 
							   this->vertices[INCIDENCES.Inc[k].pos[2]]->vertexDiff, coeff);
					distances[k] = normSquared(temp);

				}
				else distances[k] = FLT_MAX;
			}
			std::uint8_t closest = 0;
			if (distances[1] < distances[closest]) closest = 1;
			if (distances[2] < distances[closest]) closest = 2;

			if (face_ABC == regions[closest])
				this->setToFacet(closest);
			else  if (vertex_A == regions[closest])
				this->setToVertex();
			else {
				switch (closest) {
				case 0:
					if (edge_AB == regions[0])
						this->setToSegment(0);
					else
						this->setToSegment(1);
					break;
				case 1:
					if (edge_AB == regions[1])
						this->setToSegment(0);
					else
						this->setToSegment(2);
					break;
				case 2:
					if (edge_AB == regions[2])
						this->setToSegment(1);
					else
						this->setToSegment(2);
					break;
				}
			}
			return;
		}
		else if(3 == this->plex_dim) {
			ClosestElement region;
			if (4 == this->Check_triangle_origin) 
				region = getClosestInTriangle(this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff, this->vertices[2]->vertexDiff, coeff);
			else 								  
				region = static_cast<ClosestElement>(this->Check_triangle_origin);
			switch (region) {
			case face_ABC:
				this->setToFacet(0);
				break;
			case vertex_A:
				this->setToVertex();
				break;
			default:
				if(edge_AB == region) this->setToSegment(0);
				else 				  this->setToSegment(1);
				break;
			}
			return;
		}
		ClosestElement region = getClosestInSegment(this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff, coeff);
		if (vertex_A == region) this->setToVertex();
		else this->setToSegment(0);
	}

	void GjkEpa::Plex::setToVertex() {
		this->user.searchDirection = this->vertices[0]->vertexDiff;
		invert(this->user.searchDirection);
		normalizeInPlace(this->user.searchDirection);
		this->plex_dim = 1;
		std::swap(this->vertices[0], this->vertices[1]);
	}

	void GjkEpa::Plex::setToSegment(const std::uint8_t& kind) {
		Coordinate* A = &this->vertices[0]->vertexDiff;
		Coordinate* B = nullptr;
		switch (kind) {
		case 0:
			B = &this->vertices[1]->vertexDiff;
			std::swap(this->vertices[2], this->vertices[1]);
			std::swap(this->vertices[1], this->vertices[0]);
			break;
		case 1:
			B = &this->vertices[2]->vertexDiff;
			std::swap(this->vertices[0], this->vertices[1]);
			break;
		case 2:
			B = &this->vertices[3]->vertexDiff;
			std::swap(this->vertices[0], this->vertices[1]);
			std::swap(this->vertices[2], this->vertices[3]);
			break;
		}
		Coordinate B_A;
		diff(B_A, *B, *A);
		cross(this->user.searchDirection, *A, *B);
		this->user.searchDirection = cross(this->user.searchDirection, B_A);
		normalizeInPlace(this->user.searchDirection);
		this->plex_dim = 2;
	}

	void GjkEpa::Plex::setToFacet(const std::uint8_t& kind) {
		Coordinate* A = &this->vertices[0]->vertexDiff;
		Coordinate* B = nullptr;
		Coordinate* C = nullptr;
		switch (kind) {
		case 0:
			B = &this->vertices[1]->vertexDiff;
			C = &this->vertices[2]->vertexDiff;
			std::swap(this->vertices[2], this->vertices[3]);
			std::swap(this->vertices[1], this->vertices[2]);
			std::swap(this->vertices[0], this->vertices[1]);
			break;
		case 1:
			B = &this->vertices[1]->vertexDiff;
			C = &this->vertices[3]->vertexDiff;
			std::swap(this->vertices[1], this->vertices[2]);
			std::swap(this->vertices[0], this->vertices[1]);
			break;
		case 2:
			B = &this->vertices[2]->vertexDiff;
			C = &this->vertices[3]->vertexDiff;
			std::swap(this->vertices[0], this->vertices[1]);
			break;
		}
		if (3 == this->plex_dim) {
			computeOutsideNormal(this->user.searchDirection, *A, *B, *C, ORIGIN);
			invert(this->user.searchDirection);
		}
		else this->user.searchDirection = this->Normals[kind];
		this->plex_dim = 3;
	}
}
