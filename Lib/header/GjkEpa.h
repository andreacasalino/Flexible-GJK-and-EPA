/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef FLX_GJK_EPA_H
#define FLX_GJK_EPA_H

#include <shape/ConvexShape.h>

namespace flx { 
	class GjkEpa {
	public:
		GjkEpa();

		struct ShapePair {
			const shape::ConvexShape& shapeA;
			const shape::ConvexShape& shapeB;
		};

		/** \brief Returns true if the last set pair of shapes is in collision, otherwise returns false.
		* \details The shapes can be set using GJK_EPA::Set_shape_A; GJK_EPA::Set_shape_B;
		GJK_EPA::Set_shape_A_transformed or GJK_EPA::Set_shape_B_transformed.
		In case at least one of the shape was not set, an exception is raised.
		*/
		bool isCollisionPresent(const ShapePair& pair);

		struct CoordinatePair {
			Coordinate pointA;
			Coordinate pointB;
		};

		enum ResultType { closestPoint, penetrationVector };
		ResultType doComplexQuery(const ShapePair& pair, CoordinatePair& result);

	private:
		struct MinkowskiCoordinate {
			Coordinate vertexA;
			Coordinate vertexB;
			Coordinate vertexDiff;
		};
		void getSupportMinkowskiDiff(const ShapePair& pair, MinkowskiCoordinate& result);
		
		class Plex;

		class EpaHullObserver;
		void epa(const Plex& lastPlex, CoordinatePair& result);

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

	// data
		Coordinate searchDirection;
	// cache to speed up computations
		Coordinate searchDirectionTwin;
	};
}

#endif