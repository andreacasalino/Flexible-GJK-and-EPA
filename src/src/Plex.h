/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "Commons.h"

#include <array>
#include <memory>
#include <variant>

namespace flx {
using MinkowskiCoordinates =
    std::array<std::unique_ptr<MinkowskiDiffCoordinate>, 4>;

struct PlexData {
  hull::Coordinate search_direction;
  MinkowskiCoordinates vertices = {std::make_unique<MinkowskiDiffCoordinate>(),
                                   std::make_unique<MinkowskiDiffCoordinate>(),
                                   std::make_unique<MinkowskiDiffCoordinate>(),
                                   std::make_unique<MinkowskiDiffCoordinate>()};
};
using PlexDataPtr = std::shared_ptr<PlexData>;

struct VertexCase {
  PlexDataPtr data;
};

struct SegmentCase {
  PlexDataPtr data;
};

struct FacetCase {
  PlexDataPtr data;
};

using Plex = std::variant<VertexCase, SegmentCase, FacetCase>;

struct CollisionCase {};

using PlexUpdateResult = std::variant<CollisionCase, Plex>;

// the new vertex is supposed to have already been placed at the front of the
// vertices
PlexUpdateResult update_plex(const Plex &subject);
} // namespace flx
