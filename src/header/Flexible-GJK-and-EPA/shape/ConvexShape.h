/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Coordinate.h>

namespace flx::shape {
/** @brief A general interface for describing a convex shape
 */
class ConvexShape {
public:
  ConvexShape() = default;
  ConvexShape(const ConvexShape &) = delete;
  ConvexShape &operator=(const ConvexShape &) = delete;
  ConvexShape(ConvexShape &&) = delete;
  ConvexShape &operator=(ConvexShape &&) = delete;

  /** @param[out] the support point in the passed direction
   *  @param[out] the search direction
   */
  virtual void getSupport(Coordinate &result,
                          const Coordinate &direction) const = 0;
};
} // namespace flx::shape
