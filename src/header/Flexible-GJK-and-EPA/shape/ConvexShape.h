/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Coordinate.h>

namespace flx::shape {
/** @brief A general interface for describing a convex solid
 */
class ConvexShape {
public:
  virtual ~ConvexShape() = default;

  /** @param[out] the support point in the passed direction
   *  @param[in] the search direction
   */
  virtual void getSupport(hull::Coordinate &support,
                          const hull::Coordinate &direction) const = 0;
};
} // namespace flx::shape
