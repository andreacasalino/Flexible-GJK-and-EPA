/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "Hull.h"
#include "Plex.h"
#include <array>
#include <map>

namespace flx {
class GjkEpa::Epa : public hull::Observer {
public:
  Epa(GjkEpa::Plex &lastPlex, CoordinatePair &result
#ifdef FLX_LOGGER_ENABLED
      ,
      std::shared_ptr<Logger> log
#endif
  );

private:
  struct MinkowskiPair {
    Coordinate vertexA;
    Coordinate vertexB;
    MinkowskiPair(const Coordinate &a, const Coordinate &b)
        : vertexA(a), vertexB(b){};
  };

  void
  AddedChangedFacets(const std::list<const hull::Facet *> &added,
                     const std::list<const hull::Facet *> &changed) const final;
  void RemovedFacets(const std::list<const hull::Facet *> &removed) const final;

  std::array<MinkowskiCoordinate, 4>
  getInitialTethraedron(GjkEpa::Plex &lastPlex);

  // data
  mutable std::map<const hull::Facet *, double> facetDistances;
  // cache
  Coordinate searchDirection;

#ifdef FLX_LOGGER_ENABLED
  std::string print(const hull::Hull &h, const hull::Facet &closest) const;
#endif
};
} // namespace flx
