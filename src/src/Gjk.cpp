/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Gjk.h"

namespace flx {
Plex initial_GJK_loop(const ShapePair &pair) {
  auto plex_data = std::make_shared<PlexData>();
  plex_data->search_direction = hull::Coordinate{1.f, 0, 0};
  getSupportMinkowskiDiff(pair, plex_data->search_direction,
                          *plex_data->vertices[0]);
  if (hull::normSquared(plex_data->vertices[0]->vertex_in_Minkowski_diff) <=
      GEOMETRIC_TOLLERANCE2) {
    return CollisionCase{VertexCase{plex_data}};
  }
  plex_data->search_direction =
      plex_data->vertices[0]->vertex_in_Minkowski_diff;
  hull::invert(plex_data->search_direction);
  std::swap(plex_data->vertices[0], plex_data->vertices[1]);
  PlexCase plex = VertexCase{plex_data};
  while (true) {
    getSupportMinkowskiDiff(pair, plex_data->search_direction,
                            *plex_data->vertices[0]);
    if (hull::dot(plex_data->vertices[0]->vertex_in_Minkowski_diff,
                  plex_data->search_direction) <=
        hull::HULL_GEOMETRIC_TOLLERANCE) {
      return plex;
    }
    auto updated_plex = update_plex(plex);
    auto *maybe_collision = std::get_if<CollisionCase>(&updated_plex);
    if (nullptr != maybe_collision) {
      return *maybe_collision;
    }
    plex = std::get<PlexCase>(updated_plex);
  }
}

CoordinatePair finishing_GJK_loop(const ShapePair &pair,
                                  const PlexCase &initial_plex) {
  PlexData &plex_data = ;
  PlexCase plex = initial_plex;
  hull::Coordinate delta;
  diff(delta, plex_data.vertices[0]->vertex_in_Minkowski_diff,
       plex_data.vertices[1]->vertex_in_Minkowski_diff);
  if (dot(plex_data.search_direction, delta) >
      hull::HULL_GEOMETRIC_TOLLERANCE) {
    do {
      plex = std::get<PlexCase>(update_plex(plex));
      getSupportMinkowskiDiff(pair, plex_data.search_direction,
                              *plex_data.vertices[0]);
      diff(delta, plex_data.vertices[0]->vertex_in_Minkowski_diff,
           plex_data.vertices[1]->vertex_in_Minkowski_diff);
      if (dot(plex_data.search_direction, delta) <=
          hull::HULL_GEOMETRIC_TOLLERANCE) {
        break;
      }
    } while (true);
  }

  struct Visitor {
    mutable CoordinatePair closest_pair;

    void operator()(const VertexCase &subject) const {
      closest_pair.point_in_shape_a =
          subject.data->vertices[0]->vertex_in_shape_a;
      closest_pair.point_in_shape_b =
          subject.data->vertices[0]->vertex_in_shape_b;
    };

    void operator()(const SegmentCase &subject) const {
      auto temp = getClosestInSegment(
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
      auto temp = getClosestInTriangle(
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
