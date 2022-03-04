#include "Commons.h"

namespace flx {
ClosestResult getClosestToOriginInSegment(const hull::Coordinate &A,
                                          const hull::Coordinate &B) {
  Coefficients miximg_coeff = Coefficients{0, 0};
  hull::Coordinate B_A;
  hull::diff(B_A, B, A);
  miximg_coeff[1] = -dot(B_A, A) / dot(B_A, B_A);
  if (miximg_coeff[1] <= 0.f) {
    miximg_coeff[0] = 1.f;
    miximg_coeff[1] = 0.f;
    return ClosestResult{vertex_A, std::move(miximg_coeff)};
  }
  miximg_coeff[0] = 1.f - miximg_coeff[1];
  return ClosestResult{edge_AB, std::move(miximg_coeff)};
}

ClosestResult getClosestToOriginInTriangle(const hull::Coordinate &A,
                                           const hull::Coordinate &B,
                                           const hull::Coordinate &C) {
  hull::Coordinate B_A;
  hull::diff(B_A, B, A);
  hull::Coordinate C_A;
  hull::diff(C_A, C, A);

  float m11 = dot(B_A, B_A);
  float m22 = dot(C_A, C_A);
  float m12 = dot(B_A, C_A);

  float c1 = -dot(A, B_A);
  float c2 = -dot(A, C_A);

  Coefficients miximg_coeff = Coefficients{0, 0, 0};
  miximg_coeff[1] = (c1 * m22 - m12 * c2) / (m11 * m22 - m12 * m12);
  miximg_coeff[2] = (c2 - m12 * miximg_coeff[1]) / m22;

  bool temp[3];
  temp[0] = ((miximg_coeff[2] >= 0.f) && (miximg_coeff[2] <= 1.f));
  temp[1] = ((miximg_coeff[1] >= 0.f) && (miximg_coeff[1] <= 1.f));
  temp[2] = (miximg_coeff[1] + miximg_coeff[2] <= 1.f);

  if (temp[0] && temp[1] && temp[2]) {
    miximg_coeff[0] = 1.f - miximg_coeff[1] - miximg_coeff[2];
    return ClosestResult{face_ABC, std::move(miximg_coeff)};
  }
  auto [closest_AB, miximg_coeff_AB] = getClosestToOriginInSegment(A, B);
  hull::Coordinate V_AB(A);
  prod(V_AB, miximg_coeff_AB[0]);
  V_AB.x += miximg_coeff_AB[1] * B.x;
  V_AB.y += miximg_coeff_AB[1] * B.y;
  V_AB.z += miximg_coeff_AB[1] * B.z;
  float d_AB = dot(V_AB, V_AB);

  auto [closest_AC, miximg_coeff_AC] = getClosestToOriginInSegment(A, C);
  hull::Coordinate V_AC(A);
  prod(V_AC, miximg_coeff_AC[0]);
  V_AC.x += miximg_coeff_AC[1] * C.x;
  V_AC.y += miximg_coeff_AC[1] * C.y;
  V_AC.z += miximg_coeff_AC[1] * C.z;
  float d_AC = dot(V_AC, V_AC);

  if (d_AB < d_AC)
    return ClosestResult{
        closest_AB, Coefficients{miximg_coeff_AB[0], miximg_coeff_AB[1], 0}};

  if (closest_AC == edge_AB)
    closest_AC = edge_AC;
  return ClosestResult{closest_AC,
                       Coefficients{miximg_coeff_AC[0], 0, miximg_coeff_AC[1]}};
}

hull::Coordinate computeOutsideNormal(const hull::Coordinate &P1,
                                      const hull::Coordinate &P2,
                                      const hull::Coordinate &P3,
                                      const hull::Coordinate &Pother) {
  hull::Coordinate N;
  hull::Coordinate Delta1, Delta2;
  hull::diff(Delta1, P2, P1);
  hull::diff(Delta2, P3, P1);
  hull::cross(N, Delta1, Delta2);

  hull::diff(Delta1, Pother, P1);
  if (hull::dot(N, Delta1) > hull::HULL_GEOMETRIC_TOLLERANCE)
    hull::invert(N);
  hull::normalizeInPlace(N);
  return N;
}

void MinkowskiDifference::getSupport(MinkowskiDiffCoordinate &result,
                                     const hull::Coordinate &direction) const {
  pair.shape_a.getSupport(result.vertex_in_shape_a, direction);

  auto direction_twin = direction;
  hull::invert(direction_twin);
  pair.shape_b.getSupport(result.vertex_in_shape_b, direction_twin);

  hull::diff(result.vertex_in_Minkowski_diff, result.vertex_in_shape_a,
             result.vertex_in_shape_b);
}
} // namespace flx
