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
#include <variant>

namespace flx {
/** @brief Returns true if the passed set of shapes is in collision,
otherwise
 * returns false. IMPORTANT!!!! this method is not thread safe: use
 different
 * GjkEpa solvers to implement multi-threading strategies
 */
bool is_collision_present(const shape::ConvexShape &shape_a,
                          const shape::ConvexShape &shape_b);

// TODO spiegare
std::optional<CoordinatePair>
get_closest_points(const shape::ConvexShape &shape_a,
                   const shape::ConvexShape &shape_b);

// TODO spiegare
std::optional<CoordinatePair>
get_penetration_info(const shape::ConvexShape &shape_a,
                     const shape::ConvexShape &shape_b);

/** @brief Perform a complex query on the passed pair of shapes.
 *  The pair of coordinates returned as result has the following meaning:
 *
 * 		- if the shapes are not in collision are the closest
 points.
 *        The norm of the difference of these 2 vectors represent the
 distance
 * between the shapes.
 *
 * 		- if the shapes are in collision is the penetration
 vector
 *        The norm of the difference of these 2 vectors represent the
 * penetration depth.
 *
 *  @return the meaning of result
 *  @param the pair of shapes involved in the query
 *  @param the result of the query
 * IMPORTANT!!!! this method is not thread safe: use different GjkEpa
 solvers
 * to implement multi-threading strategies
 */
struct QueryResult {
  bool is_closest_pair_or_penetration_info;
  CoordinatePair result;
};
QueryResult
get_closest_points_or_penetration_info(const shape::ConvexShape &shape_a,
                                       const shape::ConvexShape &shape_b);
} // namespace flx
