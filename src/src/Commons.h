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
#include <array>

namespace flx {
constexpr float GEOMETRIC_TOLLERANCE_SQUARED =
    hull::HULL_GEOMETRIC_TOLLERANCE * hull::HULL_GEOMETRIC_TOLLERANCE;
constexpr float GEOMETRIC_TOLLERANCE_SQUARED_SQUARED =
    GEOMETRIC_TOLLERANCE_SQUARED * GEOMETRIC_TOLLERANCE_SQUARED;

bool is_greater(const float value, const float threshold);
bool is_lower(const float value, const float threshold);

// returns (first-second)
hull::Coordinate delta(const hull::Coordinate &first,
                       const hull::Coordinate &second);

enum ClosestRegionToOrigin { vertex_A, edge_AB, edge_AC, face_ABC };

template <std::size_t N> struct ClosestResult {
  ClosestRegionToOrigin region;
  std::array<float, N> coefficients;
};

ClosestResult<2> getClosestToOriginInSegment(const hull::Coordinate &A,
                                             const hull::Coordinate &B);

ClosestResult<3> getClosestToOriginInTriangle(const hull::Coordinate &A,
                                              const hull::Coordinate &B,
                                              const hull::Coordinate &C);

hull::Coordinate computeOutsideNormal(const hull::Coordinate &P1,
                                      const hull::Coordinate &P2,
                                      const hull::Coordinate &P3,
                                      const hull::Coordinate &Pother);

hull::Coordinate mix2(const hull::Coordinate &A, const hull::Coordinate &B,
                      const std::array<float, 2> &coeff);

hull::Coordinate mix3(const hull::Coordinate &A, const hull::Coordinate &B,
                      const hull::Coordinate &C,
                      const std::array<float, 3> &coeff);

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
