/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "../Commons.h"
#include "../Diagnostic.h"

#include <memory>
#include <variant>

namespace flx::gjk {
using MinkowskiCoordinates =
    std::array<std::unique_ptr<MinkowskiDiffCoordinate>, 4>;

class SearcDirection {
public:
  SearcDirection() = default;

  // direction is normalized before updating the internal value
  void udpate(const hull::Coordinate &new_direction);

  void udpateAsIs(const hull::Coordinate &new_direction) {
    search_direction = new_direction;
  };

  const hull::Coordinate &get() const { return search_direction; };

private:
  hull::Coordinate search_direction = hull::Coordinate{1.f, 0, 0};
};

struct PlexData {
  SearcDirection search_direction;
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

PlexDataPtr extract_data(const Plex &plex);

struct CollisionCase {};

using PlexUpdateResult = std::variant<CollisionCase, Plex>;

// the new vertex is supposed to have already been placed at the front of the
// vertices
PlexUpdateResult update_plex(const Plex &subject
#ifdef GJK_EPA_DIAGNOSTIC
                             ,
                             nlohmann::json &log
#endif
);

VertexCase set_to_vertex(const PlexDataPtr &data);
} // namespace flx::gjk
