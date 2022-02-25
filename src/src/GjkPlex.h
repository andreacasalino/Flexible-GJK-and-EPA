/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "Commons.h"
#include <Flexible-GJK-and-EPA/GjkEpa.h>

#include <array>
#include <memory>
#include <variant>

#ifdef FLX_LOGGER_ENABLED
#include "Logger.h"
#endif

namespace flx {
// constexpr float GEOMETRIC_TOLLERANCE2 =
//     hull::GEOMETRIC_TOLLERANCE * hull::GEOMETRIC_TOLLERANCE;
// constexpr float GEOMETRIC_TOLLERANCE4 =
//     GEOMETRIC_TOLLERANCE2 * GEOMETRIC_TOLLERANCE2;

using MinkowskiCoordinates =
    std::array<std::unique_ptr<MinkowskiDiffCoordinate>, 4>;

struct PlexData {
  ShapePair shape_pair;
  hull::Coordinate search_direction;
  MinkowskiCoordinates vertices;
  // hull::Coordinate Normals[3]; // ABC, ABE, ACE
  // bool Origin_is_visible[3];   // ABC, ABE, ACE
};
using PlexDataPtr = std::shared_ptr<PlexData>;

struct PlexCaseBase {
  PlexDataPtr data;
};

class PlexCase;

struct VertexCase : public PlexCaseBase {
  PlexCase update() const;
};

struct SegmentCase : public PlexCaseBase {
  PlexCase update() const;
};

struct FacetCase : public PlexCaseBase {
  PlexCase update() const;
};

using PlexCase = std::variant<VertexCase, SegmentCase, FacetCase>;

const PlexData *access_data(const PlexCase &plex);

struct InitialGJKLoopResult {
  bool collision_present;
  PlexCase plex;
};
InitialGJKLoopResult initial_GJK_loop(const ShapePair &pair);

PlexCase finishing_GJK_loop(const PlexCase &initial_plex);
} // namespace flx
