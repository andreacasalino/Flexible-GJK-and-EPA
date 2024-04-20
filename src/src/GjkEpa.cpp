/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/Diagnostic.h>
#include <Flexible-GJK-and-EPA/GjkEpa.h>

#include "epa/Epa.h"
#include "gjk/Gjk.h"

namespace flx {
bool is_collision_present(const shape::ConvexShape &shape_a,
                          const shape::ConvexShape &shape_b
#ifdef GJK_EPA_DIAGNOSTIC
                          ,
                          Observer *obsv
#endif
) {
  return gjk::initial_GJK_loop(ShapePair{shape_a, shape_b}
#ifdef GJK_EPA_DIAGNOSTIC
                               ,
                               obsv
#endif
                               )
      .collision;
}

std::optional<CoordinatePair>
get_closest_points(const shape::ConvexShape &shape_a,
                   const shape::ConvexShape &shape_b
#ifdef GJK_EPA_DIAGNOSTIC
                   ,
                   Observer *obsv
#endif
) {
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = gjk::initial_GJK_loop(pair
#ifdef GJK_EPA_DIAGNOSTIC
                                      ,
                                      obsv
#endif
  );

  std::optional<CoordinatePair> res;
  if (!result.collision) {
    res = gjk::finishing_GJK_loop(pair, std::move(result));
  }
  return res;
}

std::optional<CoordinatePair>
get_penetration_info(const shape::ConvexShape &shape_a,
                     const shape::ConvexShape &shape_b
#ifdef GJK_EPA_DIAGNOSTIC
                     ,
                     Observer *obsv
#endif
) {
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = gjk::initial_GJK_loop(pair
#ifdef GJK_EPA_DIAGNOSTIC
                                      ,
                                      obsv
#endif
  );

  std::optional<CoordinatePair> res;
  if (result.collision) {
    res = epa::EPA(pair, result);
  }
  return res;
}

QueryResult
get_closest_points_or_penetration_info(const shape::ConvexShape &shape_a,
                                       const shape::ConvexShape &shape_b
#ifdef GJK_EPA_DIAGNOSTIC
                                       ,
                                       Observer *obsv
#endif
) {
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = gjk::initial_GJK_loop(pair
#ifdef GJK_EPA_DIAGNOSTIC
                                      ,
                                      obsv
#endif
  );
  QueryResult res;
  if (result.collision) {
    res = QueryResult{false, epa::EPA(pair, result)};
  } else {
    res = QueryResult{true, gjk::finishing_GJK_loop(pair, std::move(result))};
  }
  return res;
}
} // namespace flx
