/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Epa.h"
#include "EpaHull.h"

#include <optional>

namespace flx::epa {
namespace {
std::optional<MinkowskiDiffCoordinate>
find_vertex_trying_direction_and_opposite(
    const MinkowskiDifference &mink_diff,
    const MinkowskiDiffCoordinate &existing_vertex,
    hull::Coordinate direction) {
  hull::normalizeInPlace(direction);
  std::optional<MinkowskiDiffCoordinate> result =
      std::make_optional<MinkowskiDiffCoordinate>();
  mink_diff.getSupport(*result, direction);
  hull::Coordinate delta;
  hull::diff(delta, result->vertex_in_Minkowski_diff,
             existing_vertex.vertex_in_Minkowski_diff);
  if (is_greater(hull::dot(direction, delta),
                 hull::HULL_GEOMETRIC_TOLLERANCE)) {
    return result;
  }
  hull::invert(direction);
  mink_diff.getSupport(*result, direction);
  hull::diff(delta, result->vertex_in_Minkowski_diff,
             existing_vertex.vertex_in_Minkowski_diff);
  if (is_greater(hull::dot(direction, delta),
                 hull::HULL_GEOMETRIC_TOLLERANCE)) {
    return result;
  }
  return std::nullopt;
}

CoordinatePair closest_in_boundaries(const MinkowskiDiffCoordinate &P) {
  return CoordinatePair{P.vertex_in_shape_a, P.vertex_in_shape_b};
}

CoordinatePair closest_in_boundaries(const MinkowskiDiffCoordinate &A,
                                     const MinkowskiDiffCoordinate &B) {
  float dist_A = hull::normSquared(A.vertex_in_Minkowski_diff);
  if (dist_A < GEOMETRIC_TOLLERANCE_SQUARED) {
    return CoordinatePair{A.vertex_in_shape_a, A.vertex_in_shape_b};
  }
  float dist_B = hull::normSquared(B.vertex_in_Minkowski_diff);
  if (dist_B < GEOMETRIC_TOLLERANCE_SQUARED) {
    return CoordinatePair{B.vertex_in_shape_a, B.vertex_in_shape_b};
  }
  dist_A = sqrt(dist_A);
  dist_B = sqrt(dist_B);
  std::array<float, 2> coeff = {dist_A / (dist_A + dist_B),
                                dist_B / (dist_A + dist_B)};
  return CoordinatePair{mix2(A.vertex_in_shape_a, B.vertex_in_shape_a, coeff),
                        mix2(A.vertex_in_shape_b, B.vertex_in_shape_b, coeff)};
}

struct ClosestInLine {
  float distance;
  float s_parameter;
};
ClosestInLine closest_in_line(const hull::Coordinate &A,
                              const hull::Coordinate &B) {
  hull::Coordinate BA = delta(B, A);
  float s = -hull::dot(A, BA) / hull::dot(BA, BA);
  hull::Coordinate closest = mix2(A, B, {s, 1.f - s});
  return ClosestInLine{hull::normSquared(closest), s};
}
CoordinatePair closest_in_boundaries(const MinkowskiDiffCoordinate &A,
                                     const MinkowskiDiffCoordinate &B,
                                     const MinkowskiDiffCoordinate &C) {
  std::array<ClosestInLine, 3> edges_results = {
      closest_in_line(A.vertex_in_Minkowski_diff, B.vertex_in_Minkowski_diff),
      closest_in_line(A.vertex_in_Minkowski_diff, C.vertex_in_Minkowski_diff),
      closest_in_line(B.vertex_in_Minkowski_diff, C.vertex_in_Minkowski_diff)};
  std::size_t closest_edge = 0;
  if (edges_results[1].distance < edges_results[closest_edge].distance) {
    closest_edge = 1;
  }
  if (edges_results[2].distance < edges_results[closest_edge].distance) {
    closest_edge = 2;
  }
  std::array<float, 2> coeff = {edges_results[closest_edge].s_parameter,
                                1.f - edges_results[closest_edge].s_parameter};
  if (closest_edge == 0) {
    return CoordinatePair{
        mix2(A.vertex_in_shape_a, B.vertex_in_shape_a, coeff),
        mix2(A.vertex_in_shape_b, B.vertex_in_shape_b, coeff)};
  }
  if (closest_edge == 1) {
    return CoordinatePair{
        mix2(A.vertex_in_shape_a, C.vertex_in_shape_a, coeff),
        mix2(A.vertex_in_shape_b, C.vertex_in_shape_b, coeff)};
  }
  return CoordinatePair{mix2(B.vertex_in_shape_a, C.vertex_in_shape_a, coeff),
                        mix2(B.vertex_in_shape_b, C.vertex_in_shape_b, coeff)};
}

CoordinatePair
closest_in_boundaries(const MinkowskiDifference &mink_diff,
                      const std::vector<MinkowskiDiffCoordinate> &initial_points
#ifdef GJK_EPA_DIAGNOSTIC
                      ,
                      diagnostic::Diagnostic &log
#endif
) {
  EpaHull epa_hull(mink_diff, initial_points);
#ifdef GJK_EPA_DIAGNOSTIC
  log.addEpaIter(epa_hull.toJson());
#endif
  while (epa_hull.update()) {
#ifdef GJK_EPA_DIAGNOSTIC
    log.addEpaIter(epa_hull.toJson());
#endif
  }
  auto closest_facet = epa_hull.getClosestFacetToOrigin();
  auto closest_info =
      getClosestToOriginInTriangle(closest_facet[0]->vertex_in_Minkowski_diff,
                                   closest_facet[1]->vertex_in_Minkowski_diff,
                                   closest_facet[2]->vertex_in_Minkowski_diff);
  return CoordinatePair{
      mix3(closest_facet[0]->vertex_in_shape_a,
           closest_facet[1]->vertex_in_shape_a,
           closest_facet[2]->vertex_in_shape_a, closest_info.coefficients),
      mix3(closest_facet[0]->vertex_in_shape_b,
           closest_facet[1]->vertex_in_shape_b,
           closest_facet[2]->vertex_in_shape_b, closest_info.coefficients)};
}

void extend_initial_points(
    const MinkowskiDifference &mink_diff,
    std::vector<MinkowskiDiffCoordinate> &initial_points) {
  while (initial_points.size() < 4) {
    switch (initial_points.size()) {
    case 1: {
      auto new_point = find_vertex_trying_direction_and_opposite(
          mink_diff, initial_points.front(), hull::Coordinate{1.f, 0, 0});
      if (std::nullopt == new_point) {
        return;
      }
      initial_points.push_back(*new_point);
    } break;
    case 2: {
      const auto &A = initial_points[0];
      const auto &B = initial_points[1];
      hull::Coordinate delta;
      hull::diff(delta, A.vertex_in_Minkowski_diff, B.vertex_in_Minkowski_diff);
      auto direction1 = hull::cross(delta, hull::Coordinate{1.f, 0, 0});
      auto direction2 = hull::cross(delta, hull::Coordinate{0, 1.f, 0});
      auto &direction = direction1;
      if (hull::normSquared(direction2) > hull::normSquared(direction1)) {
        direction = direction2;
      }
      auto new_point =
          find_vertex_trying_direction_and_opposite(mink_diff, A, direction);
      if (std::nullopt == new_point) {
        return;
      }
      initial_points.push_back(*new_point);
    } break;
    case 3: {
      hull::Coordinate delta_AB =
          delta(initial_points[0].vertex_in_Minkowski_diff,
                initial_points[1].vertex_in_Minkowski_diff);
      hull::Coordinate delta_AC =
          delta(initial_points[0].vertex_in_Minkowski_diff,
                initial_points[2].vertex_in_Minkowski_diff);
      auto new_point = find_vertex_trying_direction_and_opposite(
          mink_diff, initial_points.front(), hull::cross(delta_AB, delta_AC));
      if (std::nullopt == new_point) {
        return;
      }
      initial_points.push_back(*new_point);
    } break;
    }
  }
}
} // namespace

CoordinatePair EPA(const ShapePair &pair, const gjk::Plex &initial_plex
#ifdef GJK_EPA_DIAGNOSTIC
                   ,
                   diagnostic::Diagnostic &log
#endif
) {
  struct Visitor {
    mutable std::vector<MinkowskiDiffCoordinate> result;

    void operator()(const gjk::VertexCase &subject) const {
      if (hull::squaredDistance(
              subject.data->vertices[0]->vertex_in_Minkowski_diff,
              subject.data->vertices[1]->vertex_in_Minkowski_diff) <=
          GEOMETRIC_TOLLERANCE_SQUARED) {
        result =
            std::vector<MinkowskiDiffCoordinate>{*subject.data->vertices[0]};
      } else {
        result = std::vector<MinkowskiDiffCoordinate>{
            *subject.data->vertices[0], *subject.data->vertices[1]};
      }
    }

    void operator()(const gjk::SegmentCase &subject) const {
      result = std::vector<MinkowskiDiffCoordinate>{*subject.data->vertices[0],
                                                    *subject.data->vertices[1],
                                                    *subject.data->vertices[2]};
    }

    void operator()(const gjk::FacetCase &subject) const {
      result = std::vector<MinkowskiDiffCoordinate>{
          *subject.data->vertices[0], *subject.data->vertices[1],
          *subject.data->vertices[2], *subject.data->vertices[3]};
    }
  } visitor;
  std::visit(visitor, initial_plex);
  std::vector<MinkowskiDiffCoordinate> initial_points =
      std::move(visitor.result);

  MinkowskiDifference mink_diff(pair);
  extend_initial_points(mink_diff, initial_points);

  switch (initial_points.size()) {
  case 1:
    return closest_in_boundaries(initial_points[0]);
  case 2:
    return closest_in_boundaries(initial_points[0], initial_points[1]);
  case 3:
    return closest_in_boundaries(initial_points[0], initial_points[1],
                                 initial_points[2]);
  default:
    break;
  }
  return closest_in_boundaries(mink_diff, initial_points);
}
} // namespace flx::epa
