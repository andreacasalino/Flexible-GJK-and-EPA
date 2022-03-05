/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Epa.h"
#include "EpaHull.h"

namespace flx::epa {
CoordinatePair EPA(const ShapePair &pair, const gjk::Plex &initial_plex
#ifdef GJK_EPA_DIAGNOSTIC
                   ,
                   diagnostic::Diagnostic &log
#endif
) {
  EpaHull epa_hull(pair, initial_plex);
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
} // namespace flx::epa
