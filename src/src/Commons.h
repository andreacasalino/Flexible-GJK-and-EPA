/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>
#include <Hull/Coordinate.h>
#include <Hull/Definitions.h>
#include <vector>

namespace flx {
constexpr float GEOMETRIC_TOLLERANCE_SQUARED =
    hull::HULL_GEOMETRIC_TOLLERANCE * hull::HULL_GEOMETRIC_TOLLERANCE;
constexpr float GEOMETRIC_TOLLERANCE_SQUARED_SQUARED =
    GEOMETRIC_TOLLERANCE_SQUARED * GEOMETRIC_TOLLERANCE_SQUARED;

enum ClosestRegionToOrigin { vertex_A, edge_AB, edge_AC, face_ABC };

using Coefficients = std::vector<float>;

struct ClosestResult {
  ClosestRegionToOrigin region;
  Coefficients coefficients;
};
ClosestResult getClosestToOriginInSegment(const hull::Coordinate &A,
                                          const hull::Coordinate &B);

ClosestResult getClosestToOriginInTriangle(const hull::Coordinate &A,
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
  return result;
};

inline hull::Coordinate mix3(const hull::Coordinate &A,
                             const hull::Coordinate &B,
                             const hull::Coordinate &C,
                             const Coefficients &coeff) {
  auto result = mix2(A, B, coeff);
  result.x += coeff[2] * C.x;
  result.y += coeff[2] * C.y;
  result.z += coeff[2] * C.z;
  return result;
};

struct ShapePair {
  const shape::ConvexShape &shape_a;
  const shape::ConvexShape &shape_b;
};

struct MinkowskiDiffCoordinate {
  hull::Coordinate vertex_in_shape_a;
  hull::Coordinate vertex_in_shape_b;
  hull::Coordinate vertex_in_Minkowski_diff;
};

class MinkowskiDifference {
public:
  MinkowskiDifference(const ShapePair &pair) : pair(pair){};

  void getSupport(MinkowskiDiffCoordinate &result,
                  const hull::Coordinate &direction) const;

private:
  const ShapePair &pair;
};
} // namespace flx
