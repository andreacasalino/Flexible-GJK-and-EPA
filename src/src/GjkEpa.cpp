/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/GjkEpa.h>

#include "Diagnostic.h"
#include "epa/Epa.h"
#include "gjk/Gjk.h"

namespace flx {
bool is_collision_present(const shape::ConvexShape &shape_a,
                          const shape::ConvexShape &shape_b) {
#ifdef GJK_EPA_DIAGNOSTIC
  diagnostic::Diagnostic log;
#endif
  return gjk::initial_GJK_loop(
             ShapePair { shape_a, shape_b }
#ifdef GJK_EPA_DIAGNOSTIC
             ,
             log
#endif
             )
      .collision_present;
}

std::optional<CoordinatePair>
get_closest_points(const shape::ConvexShape &shape_a,
                   const shape::ConvexShape &shape_b) {
#ifdef GJK_EPA_DIAGNOSTIC
  diagnostic::Diagnostic log;
#endif
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = gjk::initial_GJK_loop(pair
#ifdef GJK_EPA_DIAGNOSTIC
                                      ,
                                      log
#endif
  );
  if (result.collision_present) {
    return std::nullopt;
  }
  return gjk::finishing_GJK_loop(pair, result.last_plex
#ifdef GJK_EPA_DIAGNOSTIC
                                 ,
                                 log
#endif
  );
}

std::optional<CoordinatePair>
get_penetration_info(const shape::ConvexShape &shape_a,
                     const shape::ConvexShape &shape_b) {
#ifdef GJK_EPA_DIAGNOSTIC
  diagnostic::Diagnostic log;
#endif
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = gjk::initial_GJK_loop(pair
#ifdef GJK_EPA_DIAGNOSTIC
                                      ,
                                      log
#endif
  );
  if (result.collision_present) {
    return epa::EPA(pair, result.last_plex
#ifdef GJK_EPA_DIAGNOSTIC
                    ,
                    log
#endif
    );
  }
  return std::nullopt;
}

QueryResult
get_closest_points_or_penetration_info(const shape::ConvexShape &shape_a,
                                       const shape::ConvexShape &shape_b) {
#ifdef GJK_EPA_DIAGNOSTIC
  diagnostic::Diagnostic log;
#endif
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = gjk::initial_GJK_loop(pair
#ifdef GJK_EPA_DIAGNOSTIC
                                      ,
                                      log
#endif
  );
  if (result.collision_present) {
    return QueryResult{false, epa::EPA(pair, result.last_plex
#ifdef GJK_EPA_DIAGNOSTIC
                                       ,
                                       log
#endif
                                       )};
  }
  return QueryResult{true, gjk::finishing_GJK_loop(pair, result.last_plex
#ifdef GJK_EPA_DIAGNOSTIC
                                                   ,
                                                   log
#endif
                                                   )};
}
} // namespace flx
