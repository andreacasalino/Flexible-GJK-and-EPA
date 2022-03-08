#include "Commons.h"

namespace flx {
bool is_greater(const float value, const float threshold) {
  return value >= threshold;
}
bool is_lower(const float value, const float threshold) {
  return value <= -threshold;
}

hull::Coordinate delta(const hull::Coordinate &first,
                       const hull::Coordinate &second) {
  hull::Coordinate delta;
  hull::diff(delta, first, second);
  return delta;
}

ClosestResult<2> getClosestToOriginInSegment(const hull::Coordinate &A,
                                             const hull::Coordinate &B) {
  std::array<float, 2> miximg_coeff = std::array<float, 2>{0, 0};
  hull::Coordinate B_A = delta(B, A);
  miximg_coeff[1] = -dot(B_A, A) / dot(B_A, B_A);
  if (miximg_coeff[1] <= 0.f) {
    miximg_coeff[0] = 1.f;
    miximg_coeff[1] = 0.f;
    return ClosestResult<2>{vertex_A, std::move(miximg_coeff)};
  }
  miximg_coeff[0] = 1.f - miximg_coeff[1];
  return ClosestResult<2>{edge_AB, std::move(miximg_coeff)};
}

ClosestResult<3> getClosestToOriginInTriangle(const hull::Coordinate &A,
                                              const hull::Coordinate &B,
                                              const hull::Coordinate &C) {
  hull::Coordinate B_A = delta(B, A);
  hull::Coordinate C_A = delta(C, A);

  float m11 = dot(B_A, B_A);
  float m22 = dot(C_A, C_A);
  float m12 = dot(B_A, C_A);

  float c1 = -dot(A, B_A);
  float c2 = -dot(A, C_A);

  std::array<float, 3> miximg_coeff = std::array<float, 3>{0, 0, 0};
  miximg_coeff[1] = (c1 * m22 - m12 * c2) / (m11 * m22 - m12 * m12);
  miximg_coeff[2] = (c2 - m12 * miximg_coeff[1]) / m22;

  bool temp[3];
  temp[0] = ((miximg_coeff[2] >= 0.f) && (miximg_coeff[2] <= 1.f));
  temp[1] = ((miximg_coeff[1] >= 0.f) && (miximg_coeff[1] <= 1.f));
  temp[2] = (miximg_coeff[1] + miximg_coeff[2] <= 1.f);

  if (temp[0] && temp[1] && temp[2]) {
    miximg_coeff[0] = 1.f - miximg_coeff[1] - miximg_coeff[2];
    return ClosestResult<3>{face_ABC, std::move(miximg_coeff)};
  }
  auto [closest_AB, miximg_coeff_AB] = getClosestToOriginInSegment(A, B);
  hull::Coordinate V_AB = mix2(A, B, miximg_coeff_AB);

  auto [closest_AC, miximg_coeff_AC] = getClosestToOriginInSegment(A, C);
  hull::Coordinate V_AC = mix2(A, C, miximg_coeff_AC);

  if (hull::normSquared(V_AB) <= hull::normSquared(V_AC))
    return ClosestResult<3>{
        closest_AB,
        std::array<float, 3>{miximg_coeff_AB[0], miximg_coeff_AB[1], 0}};

  if (closest_AC == edge_AB)
    closest_AC = edge_AC;
  return ClosestResult<3>{
      closest_AC,
      std::array<float, 3>{miximg_coeff_AC[0], 0, miximg_coeff_AC[1]}};
}

hull::Coordinate computeOutsideNormal(const hull::Coordinate &P1,
                                      const hull::Coordinate &P2,
                                      const hull::Coordinate &P3,
                                      const hull::Coordinate &Pother) {
  hull::Coordinate Delta1 = delta(P2, P1);
  hull::Coordinate Delta2 = delta(P3, P1);
  hull::Coordinate N = hull::cross(Delta1, Delta2);
  hull::normalizeInPlace(N);

  hull::diff(Delta1, P1, Pother);
  if (is_greater(hull::dot(N, Delta1), hull::HULL_GEOMETRIC_TOLLERANCE)) {
    return N;
  }

  hull::invert(N);
  hull::normalizeInPlace(N);
  return N;
}

namespace {
void add_scaled(hull::Coordinate &result, const hull::Coordinate &to_add,
                const float scale) {
  result.x += scale * to_add.x;
  result.y += scale * to_add.y;
  result.z += scale * to_add.z;
}
} // namespace

hull::Coordinate mix2(const hull::Coordinate &A, const hull::Coordinate &B,
                      const std::array<float, 2> &coeff) {
  hull::Coordinate result = A;
  prod(result, coeff[0]);
  add_scaled(result, B, coeff[1]);
  return result;
};

hull::Coordinate mix3(const hull::Coordinate &A, const hull::Coordinate &B,
                      const hull::Coordinate &C,
                      const std::array<float, 3> &coeff) {
  hull::Coordinate result = A;
  prod(result, coeff[0]);
  add_scaled(result, B, coeff[1]);
  add_scaled(result, C, coeff[2]);
  return result;
};

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
