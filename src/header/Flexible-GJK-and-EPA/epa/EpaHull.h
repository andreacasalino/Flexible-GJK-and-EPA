/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/gjk/Plex.h>
#include <Hull/Hull.h>

#include <optional>
#include <set>
#include <unordered_map>

namespace flx::epa {
class EpaHull : public hull::Observer {
public:
  EpaHull(const MinkowskiDifference &minkowski_diff,
          const std::vector<MinkowskiDiffCoordinate> &initial_vertices);

  bool update();

  std::array<const MinkowskiDiffCoordinate *, 3>
  getClosestFacetToOrigin() const;

  struct DistanceInfo {
    float distance_to_origin;
    const hull::Facet *facet;

    bool operator<(const DistanceInfo &o) const {
      return distance_to_origin < o.distance_to_origin;
    }
  };
  using Collection = std::multiset<DistanceInfo>;

#ifdef GJK_EPA_DIAGNOSTIC
  const auto &getFacets() const { return distances_facets_map; }
  const auto &getVertices() const { return vertices; }
#endif

protected:
  void hullChanges(const Notification &notification) final;

  float getSquaredDistanceToOrigin(const hull::Facet *facet);

  std::optional<hull::Hull> hull;
  MinkowskiDifference mink_diff;
  std::vector<MinkowskiDiffCoordinate> vertices;

  std::unordered_map<const hull::Facet *, Collection::iterator> facets_table;
  Collection distances_facets_map;
};

} // namespace flx::epa
