/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>
#include <Hull/Coordinate.h>
#include <vector>

namespace flx {
enum ClosestElement { vertex_A, edge_AB, edge_AC, face_ABC };

using Coefficients = std::vector<float>;

struct ClosestResult {
  ClosestElement region;
  Coefficients coefficients;
};
ClosestResult getClosestInSegment(const hull::Coordinate &A,
                                  const hull::Coordinate &B);

ClosestResult getClosestInTriangle(const hull::Coordinate &A,
                                   const hull::Coordinate &B,
                                   const hull::Coordinate &C);

hull::Coordinate computeOutsideNormal(const hull::Coordinate &P1,
                                      const hull::Coordinate &P2,
                                      const hull::Coordinate &P3,
                                      const hull::Coordinate &Pother);

inline hull::Coordinate mix2(const hull::Coordinate &A,
                             const hull::Coordinate &B,
                             const Coefficients &coeff) {
  hull::Coordinate result = A;
  prod(result, coeff[0]);
  result.x += coeff[1] * B.x;
  result.y += coeff[1] * B.y;
  result.z += coeff[1] * B.z;
};

inline hull::Coordinate mix3(const hull::Coordinate &A,
                             const hull::Coordinate &B,
                             const hull::Coordinate &C,
                             const Coefficients &coeff) {
  auto result = mix2(A, B, coeff);
  result.x += coeff[2] * C.x;
  result.y += coeff[2] * C.y;
  result.z += coeff[2] * C.z;
};

struct ShapePair {
  const shape::ConvexShape &shape_a;
  const shape::ConvexShape &shape_b;
};

struct MinkowskiDiffCoordinate {
  hull::Coordinate vertexA;
  hull::Coordinate vertexB;
  hull::Coordinate vertexDiff;
};

MinkowskiDiffCoordinate
getSupportMinkowskiDiff(const ShapePair &pair,
                        const hull::Coordinate &direction);
} // namespace flx
