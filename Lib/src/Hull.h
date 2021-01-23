/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_HULL_H
#define FLX_HULL_H

#include <list>
#include <cmath>
#include <Coordinate.h>

namespace flx::hull { 
	constexpr float HULL_GEOMETRIC_TOLLERANCE = static_cast<float>(1e-3);

	struct Facet {
		const Coordinate*		A; //pointer to vertex A
		const Coordinate*		B; //pointer to vertex B
		const Coordinate*		C; //pointer to vertex C
		Coordinate 				N; //outer normal 
		Facet*					Neighbour[3]; // AB, BC, CA
	// cache
		bool					bVisible; //only for update Hull
	};

	class Observer {
	public:
		virtual void AddedChangedFacets(const std::list<const Facet*>& added
									   ,const std::list<const Facet*>& changed) const = 0;
		virtual void RemovedFacets(const std::list<const Facet*>& removed) const = 0;
	};

	class Hull {
	public:
		Hull(const Coordinate& A, const Coordinate& B, const  Coordinate& C, const Coordinate& D, const Observer* obs = nullptr);

		void UpdateHull(const Coordinate& vertex_of_new_cone, const Facet& starting_facet_for_expansion);
		void UpdateHull(const Coordinate& vertex_of_new_cone);

		inline const std::list<Coordinate>& getVertices() const { return this->vertices; };
		inline const std::list<Facet>& getFacets() const { return this->Facets; };

	private:
		void _UpdateHull(const Coordinate& vertex_of_new_cone, Facet& starting_facet_for_expansion);

		void AppendFacet(const Coordinate& vertexA, const Coordinate& vertexB, const Coordinate& vertexC);
		void RecomputeNormal(Facet& facet);

	// data
		std::list<Coordinate>				vertices;
		std::list<Facet>					Facets;
		Coordinate							Mid_point;
		const Observer*						observer;
	};
}

#endif