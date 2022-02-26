/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Gjk.h"

namespace flx {
InitialLoopResult initial_GJK_loop(const ShapePair &pair) {
  auto plex_data = std::make_shared<PlexData>();
  plex_data->search_direction = hull::Coordinate{1.f, 0, 0};
  getSupportMinkowskiDiff(pair, plex_data->search_direction,
                          *plex_data->vertices[0]);
  if (hull::normSquared(plex_data->vertices[0]->vertex_in_Minkowski_diff) <=
      GEOMETRIC_TOLLERANCE2) {
    *plex_data->vertices[0] = *plex_data->vertices[1];
    return InitialLoopResult{true, VertexCase{plex_data}};
  }

  Plex plex = set_to_vertex(plex_data);
  do {
    getSupportMinkowskiDiff(pair, plex_data->search_direction,
                            *plex_data->vertices[0]);
    if (hull::dot(plex_data->vertices[0]->vertex_in_Minkowski_diff,
                  plex_data->search_direction) <=
        hull::HULL_GEOMETRIC_TOLLERANCE) {
      return InitialLoopResult{false, plex};
    }
    auto update_result = update_plex(plex);
    if (nullptr != std::get_if<CollisionCase>(&update_result)) {
      return InitialLoopResult{true, plex};
    }
    plex = std::get<Plex>(update_result);
  } while (true);
}

CoordinatePair finishing_GJK_loop(const ShapePair &pair,
                                  const Plex &initial_plex) {
  auto plex_data = extract_data(initial_plex);
  auto plex = initial_plex;
  hull::Coordinate delta;
  do {
    getSupportMinkowskiDiff(pair, plex_data->search_direction,
                            *plex_data->vertices[0]);
    plex = std::get<Plex>(update_plex(plex));
    diff(delta, plex_data->vertices[0]->vertex_in_Minkowski_diff,
         plex_data->vertices[1]->vertex_in_Minkowski_diff);
  } while (hull::HULL_GEOMETRIC_TOLLERANCE <
           dot(plex_data->search_direction, delta));

  struct Visitor {
    mutable CoordinatePair closest_pair;

    void operator()(const VertexCase &subject) const {
      closest_pair.point_in_shape_a =
          subject.data->vertices[0]->vertex_in_shape_a;
      closest_pair.point_in_shape_b =
          subject.data->vertices[0]->vertex_in_shape_b;
    };

    void operator()(const SegmentCase &subject) const {
      auto temp = getClosestToOriginInSegment(
          subject.data->vertices[0]->vertex_in_Minkowski_diff,
          subject.data->vertices[1]->vertex_in_Minkowski_diff);
      closest_pair.point_in_shape_a =
          mix2(subject.data->vertices[0]->vertex_in_shape_a,
               subject.data->vertices[1]->vertex_in_shape_a, temp.coefficients);
      closest_pair.point_in_shape_b =
          mix2(subject.data->vertices[0]->vertex_in_shape_b,
               subject.data->vertices[1]->vertex_in_shape_b, temp.coefficients);
    };

    void operator()(const FacetCase &subject) const {
      auto temp = getClosestToOriginInTriangle(
          subject.data->vertices[0]->vertex_in_Minkowski_diff,
          subject.data->vertices[1]->vertex_in_Minkowski_diff,
          subject.data->vertices[2]->vertex_in_Minkowski_diff);
      closest_pair.point_in_shape_a =
          mix3(subject.data->vertices[0]->vertex_in_shape_a,
               subject.data->vertices[1]->vertex_in_shape_a,
               subject.data->vertices[2]->vertex_in_shape_a, temp.coefficients);
      closest_pair.point_in_shape_b =
          mix3(subject.data->vertices[0]->vertex_in_shape_b,
               subject.data->vertices[1]->vertex_in_shape_b,
               subject.data->vertices[2]->vertex_in_shape_b, temp.coefficients);
    };
  } visitor;
  std::visit(visitor, plex);
  return visitor.closest_pair;
}
} // namespace flx
