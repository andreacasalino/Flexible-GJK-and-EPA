/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Flexible-GJK-and-EPA/GjkEpa.h>

#include "Gjk.h"

namespace flx {
bool is_collision_present(const shape::ConvexShape &shape_a,
                          const shape::ConvexShape &shape_b) {
  return initial_GJK_loop(ShapePair{shape_a, shape_b}).collision_present;
}

std::optional<CoordinatePair>
get_closest_points(const shape::ConvexShape &shape_a,
                   const shape::ConvexShape &shape_b) {
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = initial_GJK_loop(pair);
  if (result.collision_present) {
    return std::nullopt;
  }
  return finishing_GJK_loop(pair, result.last_plex);
}

std::optional<CoordinatePair>
get_penetration_depth(const shape::ConvexShape &shape_a,
                      const shape::ConvexShape &shape_b) {
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = initial_GJK_loop(pair);
  if (!result.collision_present) {
    return std::nullopt;
  }
  return;
}

QueryResult
get_closest_points_or_penetration_depth(const shape::ConvexShape &shape_a,
                                        const shape::ConvexShape &shape_b) {
  ShapePair pair = ShapePair{shape_a, shape_b};
  auto result = initial_GJK_loop(pair);
  if (result.collision_present) {
    return QueryResult{
        false,
    };
  }
  return QueryResult{true, finishing_GJK_loop(pair, result.last_plex)};
}
} // namespace flx
