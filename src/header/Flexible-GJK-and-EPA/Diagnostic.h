#ifdef GJK_EPA_DIAGNOSTIC
#pragma once

#include <Flexible-GJK-and-EPA/CoordinatePair.h>
#include <Flexible-GJK-and-EPA/epa/EpaHull.h>
#include <Flexible-GJK-and-EPA/gjk/Plex.h>
#include <Hull/Coordinate.h>

#include <optional>
#include <variant>

namespace flx {
struct GjkIteration {
  const gjk::Plex &plex;

  struct ClosestToRegionInfo {
    ClosestRegionToOrigin region;
    hull::Coordinate point;
  };
  struct TethraedronFacetInfo {
    bool isOriginVisible;
    hull::Coordinate normal;
    std::optional<ClosestToRegionInfo> closest;
  };
  std::variant<ClosestToRegionInfo,
               std::array<TethraedronFacetInfo, 3> // order is ABC, ABD, ACD
               >
      info;
};

struct EpaIteration {
  const epa::EpaHull &hull;
};

struct QueryResult;

class Observer {
public:
  virtual ~Observer() = default;

  enum class Event { InitialGjkStarted, FinalGjkStarted, EpaStarted };

  virtual void onEvent(Event event) = 0;

  virtual void onUpdate(const GjkIteration &info) = 0;
  virtual void onUpdate(const EpaIteration &info) = 0;
};
} // namespace flx
#endif
