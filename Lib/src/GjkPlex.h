/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_GJK_PLEX_H
#define FLX_GJK_PLEX_H

#include <GjkEpa.h>
#include <memory>
#include <array>
#include <list>

namespace flx {
	constexpr float GEOMETRIC_TOLLERANCE2 = GEOMETRIC_TOLLERANCE*GEOMETRIC_TOLLERANCE;
	constexpr float GEOMETRIC_TOLLERANCE4 = GEOMETRIC_TOLLERANCE2 * GEOMETRIC_TOLLERANCE2;

	class GjkEpa::Plex {
	public:
	 	// in the constructor the primal part is done
		Plex(GjkEpa& user, const ShapePair& pair);

		// evolve plex till finding the closest entity to origin
		void finishingLoop(CoordinatePair& closestPoints);

		inline bool isCollisionPresent() { return this->collision_present; };

		typedef std::unique_ptr<MinkowskiCoordinate> MinkowskiCoordinatePtr;

		inline const ShapePair& getPair() const { return this->pair; };		

		inline const std::array<MinkowskiCoordinatePtr, 4>& getVertices() const { return this->vertices; };
		inline const std::uint8_t& getPlexDimension() const { return this->plex_dim; };

	private:
		void update(); 
		void updateOriginVisibilityFlags();

		void setToVertex();
		void setToSegment(const std::uint8_t& kind);
		void setToFacet(const std::uint8_t& kind);

	// data
		const ShapePair& 			 pair;
		GjkEpa& 		 			 user;
		std::array<MinkowskiCoordinatePtr, 4> vertices;
		std::uint8_t 				 plex_dim = 1;
		bool						 collision_present = false;
	// cache
		Coordinate					 Normals[3];  //ABC, ABE, ACE
		bool						 Origin_is_visible[3];  //ABC, ABE, ACE
		std::size_t					 Check_triangle_origin; // 4 indicates that was not updated
	};
}

#endif