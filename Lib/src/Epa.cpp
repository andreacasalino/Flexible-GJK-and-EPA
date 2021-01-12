/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <GjkEpa.h>
#include "Hull.h"
#include "GjkPlex.h"
#include <map>
#include <vector>

namespace flx {	
	class GjkEpa::EpaHullObserver : public hull::Observer {
	public:
		EpaHullObserver() = default;

		inline const std::map<const hull::Facet*, double>& getDistances() const { return this->distancesToOrigin; };
	private:
		void AddedChangedFacets(const std::list<const hull::Facet*>& added
									   ,const std::list<const hull::Facet*>& changed) const final {
			auto it = added.begin(), itEnd = added.end();
			for(it; it!=itEnd; ++it)  this->distancesToOrigin.find(*it)->second = dot((*it)->N, *(*it)->A);
			for(it = added.begin(); it!=itEnd; ++it)  this->distancesToOrigin.find(*it)->second = dot((*it)->N, *(*it)->A);			
		};

		void RemovedFacets(const std::list<const hull::Facet*>& removed) const final{
			auto itEnd = removed.end();
			for(auto it = removed.begin(); it!=itEnd; ++it) this->distancesToOrigin.erase(this->distancesToOrigin.find(*it));
		};

		mutable std::map<const hull::Facet*, double> distancesToOrigin;
	};

	void GjkEpa::epa(const Plex& lastPlex, CoordinatePair& result) {
		struct MinkowskiPair {
			Coordinate vertexA;
			Coordinate vertexB;
			MinkowskiPair(const Coordinate& a, const Coordinate& b) : vertexA(a), vertexB(b) {};
		};
		std::map<const Coordinate*, MinkowskiPair> originalVertices;
		EpaHullObserver observer;
		const std::map<const hull::Facet*, double>& distances = observer.getDistances();

		//build initial thetraedron
		std::vector<MinkowskiCoordinate> initialVertices;
		initialVertices.reserve(4);	
		{	
			const std::array<GjkEpa::Plex::MinkowskiCoordinatePtr, 4>& v = lastPlex.getVertices();
			std::uint8_t size = lastPlex.getPlexDimension();
			for(std::uint8_t k=0; k<size; ++k) initialVertices.emplace_back(*v[1 + k].get());
		}
		size_t Size;
		while (initialVertices.size() < 4) {
			Size = initialVertices.size();
			if (1 == Size) {
				this->searchDirection = initialVertices[0].vertexDiff;
				if (normSquared(this->searchDirection) <= GEOMETRIC_TOLLERANCE2) {
					this->searchDirection.x = 1.f;
					this->searchDirection.y = 0.f;
					this->searchDirection.z = 0.f;
				}
				else  invert(this->searchDirection);
				initialVertices.emplace_back();
				this->getSupportMinkowskiDiff(lastPlex.getPair(), initialVertices.back());
				if(squaredDistance(initialVertices.front().vertexDiff, initialVertices.back().vertexDiff) <= GEOMETRIC_TOLLERANCE2) {
					invert(this->searchDirection);
					this->getSupportMinkowskiDiff(lastPlex.getPair(), initialVertices.back());
				}
			}
			else if (2 == Size) {
				Coordinate D;
				diff(D, initialVertices[0].vertexDiff, initialVertices[1].vertexDiff);
				this->searchDirection = Coordinate{1.f, 0.f, 0.f};
				this->searchDirection = cross(D, this->searchDirection);
				if (normSquared(this->searchDirection) <= GEOMETRIC_TOLLERANCE2) {
					this->searchDirection.x = 0.f;
					this->searchDirection.y = 1.f;
					this->searchDirection.z = 0.f;
					this->searchDirection = cross(D, this->searchDirection);
				}
				initialVertices.emplace_back();
				this->getSupportMinkowskiDiff(lastPlex.getPair(), initialVertices.back());
				bool tempB[2];
				tempB[0] = (squaredDistance(initialVertices.back().vertexDiff, initialVertices.front().vertexDiff) <= GEOMETRIC_TOLLERANCE2);
				tempB[1] = (squaredDistance(initialVertices.back().vertexDiff, initialVertices[1].vertexDiff) <= GEOMETRIC_TOLLERANCE2);
				if (tempB[0] || tempB[1]) {
					invert(this->searchDirection);
					this->getSupportMinkowskiDiff(lastPlex.getPair(), initialVertices.back());
				}
			}
			else {
				computeOutsideNormal(this->searchDirection, initialVertices[0].vertexDiff, 
															initialVertices[1].vertexDiff, 
															initialVertices[2].vertexDiff, ORIGIN);
				invert(this->searchDirection);
				initialVertices.emplace_back();
				this->getSupportMinkowskiDiff(lastPlex.getPair(), initialVertices.back());
				Coordinate Delta;
				diff(Delta, initialVertices.back().vertexDiff, initialVertices.front().vertexDiff);
				if (dot(Delta, this->searchDirection) <= GEOMETRIC_TOLLERANCE) {
					invert(this->searchDirection);
					this->getSupportMinkowskiDiff(lastPlex.getPair(), initialVertices.back());
				}
			}
		}

		hull::Hull Minkowski_diff(initialVertices[0].vertexDiff,
								  initialVertices[1].vertexDiff,
								  initialVertices[2].vertexDiff,
								  initialVertices[3].vertexDiff,&observer);
		{
			auto it = Minkowski_diff.getVertices().begin();
			originalVertices.emplace(&(*it), MinkowskiPair(initialVertices[0].vertexA, initialVertices[0].vertexB));
			++it;
			originalVertices.emplace(&(*it), MinkowskiPair(initialVertices[1].vertexA, initialVertices[1].vertexB));
			++it;
			originalVertices.emplace(&(*it), MinkowskiPair(initialVertices[2].vertexA, initialVertices[2].vertexB));
			++it;
			originalVertices.emplace(&(*it), MinkowskiPair(initialVertices[3].vertexA, initialVertices[3].vertexB));
		}
		
		std::map<const hull::Facet*, double>::const_iterator itF, itFend = distances.end(), closestToOrigin;
		float temp;
		Coordinate V_temp;
		MinkowskiCoordinate newVertex;
		while (true) {
			itF = distances.begin();
			closestToOrigin = itF;
			itF++;
			for (itF; itF != itFend; ++itF) {
				if (itF->second < closestToOrigin->second) closestToOrigin = itF;
			}
			this->searchDirection = closestToOrigin->first->N;
			this->getSupportMinkowskiDiff(lastPlex.getPair(), newVertex);
			diff(V_temp, newVertex.vertexDiff, *closestToOrigin->first->A);
			temp = dot(closestToOrigin->first->N, V_temp);
			if (temp < GEOMETRIC_TOLLERANCE) break;
			Minkowski_diff.UpdateHull(newVertex.vertexDiff);
			originalVertices.emplace(&Minkowski_diff.getVertices().back(), MinkowskiPair(newVertex.vertexA, newVertex.vertexB));

		}
		float coeff[3];
		getClosestInTriangle(*closestToOrigin->first->A, *closestToOrigin->first->B, *closestToOrigin->first->C, coeff);
		const MinkowskiPair* originalFacet[3];
		originalFacet[0] = &originalVertices.find(closestToOrigin->first->A)->second;
		originalFacet[1] = &originalVertices.find(closestToOrigin->first->B)->second;
		originalFacet[2] = &originalVertices.find(closestToOrigin->first->C)->second;
		mix3(result.pointA, originalFacet[0]->vertexA, originalFacet[1]->vertexA, originalFacet[2]->vertexA, coeff);
		mix3(result.pointB, originalFacet[0]->vertexB, originalFacet[1]->vertexB, originalFacet[2]->vertexB, coeff);
	}
}
