/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Gjk.h"
#include <Flexible-GJK-and-EPA/Error.h>

namespace flx::gjk {
namespace {
class SupportFinder {
public:
  SupportFinder(const ShapePair &pair, const PlexDataPtr &plex_data)
      : mink_diff(pair), plex_data(plex_data) {}

  void findSupport() {
    mink_diff.getSupport(*plex_data->vertices[0],
                         plex_data->search_direction.get());
  };

  hull::Coordinate getImprovment() const {
    return delta(plex_data->vertices[0]->vertex_in_Minkowski_diff,
                 plex_data->vertices[1]->vertex_in_Minkowski_diff);
  }

private:
  MinkowskiDifference mink_diff;
  PlexDataPtr plex_data;
};
} // namespace

InitialLoopResult initial_GJK_loop(const ShapePair &pair
#ifdef GJK_EPA_DIAGNOSTIC
                                   ,
                                   diagnostic::Diagnostic &log
#endif
) {
  auto plex_data = std::make_shared<PlexData>();
  SupportFinder support_finder(pair, plex_data);
  support_finder.findSupport();
  if (hull::normSquared(plex_data->vertices[0]->vertex_in_Minkowski_diff) <=
      GEOMETRIC_TOLLERANCE_SQUARED) {
    *plex_data->vertices[1] = *plex_data->vertices[0];
    return InitialLoopResult{true, VertexCase{plex_data}};
  }

  Plex plex = set_to_vertex(plex_data);
  do {
    support_finder.findSupport();
    if (is_lower(hull::dot(plex_data->vertices[0]->vertex_in_Minkowski_diff,
                           plex_data->search_direction.get()),
                 hull::HULL_GEOMETRIC_TOLLERANCE)) {
      return InitialLoopResult{false, plex};
    }
#ifdef GJK_EPA_DIAGNOSTIC
    nlohmann::json gjk_iter_json;
#endif
    auto updated_result = update_plex(plex
#ifdef GJK_EPA_DIAGNOSTIC
                                      ,
                                      gjk_iter_json
#endif
    );
#ifdef GJK_EPA_DIAGNOSTIC
    log.addGjkInitialIter(std::move(gjk_iter_json));
#endif
    if (nullptr != std::get_if<CollisionCase>(&updated_result)) {
      return InitialLoopResult{true, plex};
    }
    plex = std::get<Plex>(updated_result);
  } while (true);
}

CoordinatePair finishing_GJK_loop(const ShapePair &pair,
                                  const Plex &initial_plex
#ifdef GJK_EPA_DIAGNOSTIC
                                  ,
                                  diagnostic::Diagnostic &log
#endif
) {
  auto plex_data = extract_data(initial_plex);
  SupportFinder support_finder(pair, plex_data);
  auto plex = initial_plex;
  hull::Coordinate delta = support_finder.getImprovment();
  while (is_greater(dot(plex_data->search_direction.get(), delta),
                    hull::HULL_GEOMETRIC_TOLLERANCE)) {
#ifdef GJK_EPA_DIAGNOSTIC
    nlohmann::json gjk_iter_json;
#endif
    auto plex_updated = update_plex(plex
#ifdef GJK_EPA_DIAGNOSTIC
                                    ,
                                    gjk_iter_json
#endif
    );
#ifdef GJK_EPA_DIAGNOSTIC
    log.addGjkFinalIter(std::move(gjk_iter_json));
#endif
    auto *plex_updated_ptr = std::get_if<Plex>(&plex_updated);
    if (nullptr == plex_updated_ptr) {
      throw Error{
          "Trying to call finishing_GJK_loop on a plex that contains origin"};
    }
    plex = *plex_updated_ptr;
    support_finder.findSupport();
    delta = support_finder.getImprovment();
  }

  struct Visitor {
    mutable CoordinatePair closest_pair;

    void operator()(const VertexCase &subject) const {
      closest_pair.point_in_shape_a =
          subject.data->vertices[1]->vertex_in_shape_a;
      closest_pair.point_in_shape_b =
          subject.data->vertices[1]->vertex_in_shape_b;
    };

    void operator()(const SegmentCase &subject) const {
      auto temp = getClosestToOriginInSegment(
          subject.data->vertices[1]->vertex_in_Minkowski_diff,
          subject.data->vertices[2]->vertex_in_Minkowski_diff);
      closest_pair.point_in_shape_a =
          mix2(subject.data->vertices[1]->vertex_in_shape_a,
               subject.data->vertices[2]->vertex_in_shape_a, temp.coefficients);
      closest_pair.point_in_shape_b =
          mix2(subject.data->vertices[1]->vertex_in_shape_b,
               subject.data->vertices[2]->vertex_in_shape_b, temp.coefficients);
    };

    void operator()(const FacetCase &subject) const {
      auto temp = getClosestToOriginInTriangle(
          subject.data->vertices[1]->vertex_in_Minkowski_diff,
          subject.data->vertices[2]->vertex_in_Minkowski_diff,
          subject.data->vertices[3]->vertex_in_Minkowski_diff);
      closest_pair.point_in_shape_a =
          mix3(subject.data->vertices[1]->vertex_in_shape_a,
               subject.data->vertices[2]->vertex_in_shape_a,
               subject.data->vertices[3]->vertex_in_shape_a, temp.coefficients);
      closest_pair.point_in_shape_b =
          mix3(subject.data->vertices[1]->vertex_in_shape_b,
               subject.data->vertices[2]->vertex_in_shape_b,
               subject.data->vertices[3]->vertex_in_shape_b, temp.coefficients);
    };
  } visitor;
  std::visit(visitor, plex);
  return visitor.closest_pair;
}
} // namespace flx::gjk
