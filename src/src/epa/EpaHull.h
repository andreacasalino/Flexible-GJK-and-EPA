/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "../Diagnostic.h"
#include "../gjk/Plex.h"
#include <Hull/Hull.h>

#include <map>

namespace flx::epa {

class EpaHull : public hull::Observer {
public:
  EpaHull(const MinkowskiDifference &minkowski_diff,
          const std::vector<MinkowskiDiffCoordinate> &initial_vertices);

  bool update();

  std::array<const MinkowskiDiffCoordinate *, 3>
  getClosestFacetToOrigin() const;

#ifdef GJK_EPA_DIAGNOSTIC
  nlohmann::json toJson() const;
#endif

protected:
  void hullChanges(Notification &&notification) final;

  std::multimap<float, const hull::Facet *>::const_iterator
  findDistanceFacetPair(const hull::Facet *facet) const;

  float getSquaredDistanceToOrigin(const hull::Facet *facet) const;

  MinkowskiDifference mink_diff;
  std::unique_ptr<hull::Hull> hull;
  std::vector<MinkowskiDiffCoordinate> vertices;
  std::multimap<float, const hull::Facet *> distances_facets_map;
};

} // namespace flx::epa
