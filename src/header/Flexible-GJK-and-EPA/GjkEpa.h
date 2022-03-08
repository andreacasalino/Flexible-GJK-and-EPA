/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/CoordinatePair.h>
#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>

#include <optional>

namespace flx {
/** @brief Returns true if the passed pair is in collision,
 * otherwise false.
 */
bool is_collision_present(const shape::ConvexShape &shape_a,
                          const shape::ConvexShape &shape_b);

/** @brief Returns closest pair of points in the pair, if they actually aren't
 * in collision. Otherwise returns a nullopt.
 *
 * This might be used to call the primal and finishing GJK, but avoid the EPA if
 * a collision exists.
 */
std::optional<CoordinatePair>
get_closest_points(const shape::ConvexShape &shape_a,
                   const shape::ConvexShape &shape_b);

/** @brief Returns penetration info of the shapes pair, if they actually are
 * in collision. Otherwise returns a nullopt.
 *
 * This might be used to call the primal and EPA, but avoid the finishing GJK if
 * a collision does not exist.
 */
std::optional<CoordinatePair>
get_penetration_info(const shape::ConvexShape &shape_a,
                     const shape::ConvexShape &shape_b);

/** @brief Perform a complex query on the passed pair of shapes.
 * The pair of coordinates returned as result might have 2 different meanings:
 *
 * - be the closest pair of points in the shapes, when they are not in
 * collision. Notice that the norm of the difference of the 2 vectors in the
 * result represents the distance between the shapes.
 *
 * - if the shapes are in collision, the difference of the 2 vectors in the
 * result represents the penetration vector. Notice that the norm of the
 * penetration vector represents the penetration depth.
 */
struct QueryResult {
  bool is_closest_pair_or_penetration_info;
  CoordinatePair result;
};
QueryResult
get_closest_points_or_penetration_info(const shape::ConvexShape &shape_a,
                                       const shape::ConvexShape &shape_b);
} // namespace flx
