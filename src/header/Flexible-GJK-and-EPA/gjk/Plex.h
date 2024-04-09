/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/Commons.h>

#include <memory>

namespace flx {
#ifdef GJK_EPA_DIAGNOSTIC
class Observer;
#endif
} // namespace flx

namespace flx::gjk {
using MinkowskiCoordinates =
    std::array<std::unique_ptr<MinkowskiDiffCoordinate>, 4>;

struct Plex {
  std::size_t size = 0;
  hull::Coordinate search_direction = hull::Coordinate{1.f, 0, 0};
  bool collision = false;
  MinkowskiCoordinates vertices = {std::make_unique<MinkowskiDiffCoordinate>(),
                                   std::make_unique<MinkowskiDiffCoordinate>(),
                                   std::make_unique<MinkowskiDiffCoordinate>(),
                                   std::make_unique<MinkowskiDiffCoordinate>()};
#ifdef GJK_EPA_DIAGNOSTIC
  Observer *obsv = nullptr;
#endif
};
} // namespace flx::gjk
