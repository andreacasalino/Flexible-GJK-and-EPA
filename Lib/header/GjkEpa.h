/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_GJK_EPA_H
#define FLX_GJK_EPA_H

#include <shape/ConvexShape.h>
#ifdef FLX_LOGGER_ENABLED
#include <string>
#endif

namespace flx { 
	class GjkEpa {
	public:
		GjkEpa() = default;

		struct ShapePair {
			const shape::ConvexShape& shapeA;
			const shape::ConvexShape& shapeB;
		};

		/** @brief Returns true if the passed set of shapes is in collision, otherwise returns false.
		 */
		bool isCollisionPresent(const ShapePair& pair
#ifdef FLX_LOGGER_ENABLED
		, std::string logFile = ""
#endif
		);

		struct CoordinatePair {
			Coordinate pointA;
			Coordinate pointB;
		};

		enum ResultType { closestPoint, penetrationVector };
		/** @brief Perform a complex query on the passed pair of shapes.
		 *  The pair of coordinates returned as result has the following meaning:
		 * 		- if the shapes are not in collision are the closest points
		 * 		- if the shapes are in collision is the penetration vector
		 *  @return the meaning of result
		 *  @param the pair of shapes involved in the query
		 *  @param the result of the query
		 */
		ResultType doComplexQuery(const ShapePair& pair, CoordinatePair& result
#ifdef FLX_LOGGER_ENABLED
		, std::string logFile = ""
#endif
		);

	private:
		struct MinkowskiCoordinate {
			Coordinate vertexA;
			Coordinate vertexB;
			Coordinate vertexDiff;
		};
		void getSupportMinkowskiDiff(const ShapePair& pair, const Coordinate& direction, MinkowskiCoordinate& result);
		
		class Plex;

		class Epa;

		enum ClosestElement { vertex_A, edge_AB, edge_AC, face_ABC };
		static ClosestElement getClosestInSegment(const Coordinate& A, const Coordinate& B, float* miximg_coeff);
		static ClosestElement getClosestInTriangle(const Coordinate& A, const Coordinate& B, const Coordinate& C, float* miximg_coeff);

		static void computeOutsideNormal(Coordinate& N, const Coordinate& P1, const Coordinate& P2, const Coordinate& P3, const Coordinate& Pother);

		static inline void mix2(Coordinate& result, const Coordinate& A, const Coordinate& B, const float* coeff) {
			result = A;
			prod(result, coeff[0]);
			result.x += coeff[1] * B.x;
			result.y += coeff[1] * B.y;
			result.z += coeff[1] * B.z;
		};

		static inline void mix3(Coordinate& result, const Coordinate& A, const Coordinate& B, const Coordinate& C, const float* coeff) {
			mix2(result, A, B, coeff);
			result.x += coeff[2] * C.x;
			result.y += coeff[2] * C.y;
			result.z += coeff[2] * C.z;
		};

	// cache to speed up computations
		Coordinate searchDirectionTwin;
	};
}

#endif