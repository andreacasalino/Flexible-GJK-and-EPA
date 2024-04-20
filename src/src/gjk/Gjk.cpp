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
  SupportFinder(const ShapePair &pair, Plex &plex_data)
      : mink_diff(pair), plex_data(plex_data) {}

  void findSupport() {
    mink_diff.getSupport(*plex_data.vertices[0], plex_data.search_direction);
  };

  hull::Coordinate getImprovment() const {
    return delta(plex_data.vertices[0]->vertex_in_Minkowski_diff,
                 plex_data.vertices[1]->vertex_in_Minkowski_diff);
  }

private:
  MinkowskiDifference mink_diff;
  Plex &plex_data;
};
} // namespace

Plex initial_GJK_loop(const ShapePair &pair
#ifdef GJK_EPA_DIAGNOSTIC
                      ,
                      Observer *obsv
#endif
) {
  Plex plex;
#ifdef GJK_EPA_DIAGNOSTIC
  plex.obsv = obsv;
  if (obsv) {
    obsv->onEvent(Observer::Event::InitialGjkStarted);
  }
#endif
  SupportFinder support_finder(pair, plex);
  support_finder.findSupport();
  if (hull::normSquared(plex.vertices[0]->vertex_in_Minkowski_diff) <=
      GEOMETRIC_TOLLERANCE_SQUARED) {
    plex.collision = true;
    return plex;
  }

  set_to_vertex(plex);
  while (!plex.collision) {
    support_finder.findSupport();
    if (is_lower(hull::dot(plex.vertices[0]->vertex_in_Minkowski_diff,
                           plex.search_direction),
                 hull::HULL_GEOMETRIC_TOLLERANCE)) {
      // no collision
      return plex;
    }
    updatePlex(plex);
  }
  return plex;
}

CoordinatePair finishing_GJK_loop(const ShapePair &pair, Plex &&plex) {
#ifdef GJK_EPA_DIAGNOSTIC
  if (plex.obsv) {
    plex.obsv->onEvent(Observer::Event::FinalGjkStarted);
  }
#endif
  SupportFinder support_finder(pair, plex);
  hull::Coordinate delta = support_finder.getImprovment();
  for (; is_greater(dot(plex.search_direction, delta),
                    hull::HULL_GEOMETRIC_TOLLERANCE);
       support_finder.findSupport(), delta = support_finder.getImprovment()) {
    updatePlex(plex);
    if (plex.collision) {
      throw Error{
          "Trying to call finishing_GJK_loop on a plex that contains origin"};
    }
  }

  CoordinatePair closest_pair;

  switch (plex.size) {
  case 1:
    closest_pair.point_in_shape_a = plex.vertices[1]->vertex_in_shape_a;
    closest_pair.point_in_shape_b = plex.vertices[1]->vertex_in_shape_b;
    break;
  case 2: {
    auto temp =
        getClosestToOriginInSegment(plex.vertices[1]->vertex_in_Minkowski_diff,
                                    plex.vertices[2]->vertex_in_Minkowski_diff);
    closest_pair.point_in_shape_a =
        mix2(plex.vertices[1]->vertex_in_shape_a,
             plex.vertices[2]->vertex_in_shape_a, temp.coefficients);
    closest_pair.point_in_shape_b =
        mix2(plex.vertices[1]->vertex_in_shape_b,
             plex.vertices[2]->vertex_in_shape_b, temp.coefficients);

  } break;
  case 3: {
    auto temp = getClosestToOriginInTriangle(
        plex.vertices[1]->vertex_in_Minkowski_diff,
        plex.vertices[2]->vertex_in_Minkowski_diff,
        plex.vertices[3]->vertex_in_Minkowski_diff);
    closest_pair.point_in_shape_a =
        mix3(plex.vertices[1]->vertex_in_shape_a,
             plex.vertices[2]->vertex_in_shape_a,
             plex.vertices[3]->vertex_in_shape_a, temp.coefficients);
    closest_pair.point_in_shape_b =
        mix3(plex.vertices[1]->vertex_in_shape_b,
             plex.vertices[2]->vertex_in_shape_b,
             plex.vertices[3]->vertex_in_shape_b, temp.coefficients);

  } break;
  }
  return closest_pair;
}
} // namespace flx::gjk
