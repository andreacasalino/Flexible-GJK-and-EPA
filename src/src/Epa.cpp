/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Epa.h"

namespace flx {
GjkEpa::Epa::Epa(GjkEpa::Plex &lastPlex, CoordinatePair &result
#ifdef FLX_LOGGER_ENABLED
                 ,
                 std::shared_ptr<Logger> log
#endif
) {
#ifdef FLX_LOGGER_ENABLED
  log->add(",\n\"Epa\":");
  Array iterations;
#endif
  // build initial Hull
  std::map<const hull::Coordinate *, MinkowskiPair> originalVertices;
  std::array<GjkEpa::MinkowskiCoordinate, 4> tetra =
      this->getInitialTethraedron(lastPlex);
  hull::Hull Minkowski_diff(tetra[0].vertexDiff, tetra[1].vertexDiff,
                            tetra[2].vertexDiff, tetra[3].vertexDiff, this);
  {
    auto it = Minkowski_diff.getVertices().begin();
    originalVertices.emplace(&(*it),
                             MinkowskiPair(tetra[0].vertexA, tetra[0].vertexB));
    ++it;
    originalVertices.emplace(&(*it),
                             MinkowskiPair(tetra[1].vertexA, tetra[1].vertexB));
    ++it;
    originalVertices.emplace(&(*it),
                             MinkowskiPair(tetra[2].vertexA, tetra[2].vertexB));
    ++it;
    originalVertices.emplace(&(*it),
                             MinkowskiPair(tetra[3].vertexA, tetra[3].vertexB));
  }
  // expand Hull till convergence
  std::map<const hull::Facet *, double>::const_iterator itF, closestToOrigin;
  float temp;
  hull::Coordinate V_temp;
  MinkowskiCoordinate newVertex;
  GjkEpa &user = lastPlex.getUser();
  do {
    itF = this->facetDistances.begin();
    closestToOrigin = itF;
    ++itF;
    for (itF; itF != this->facetDistances.end(); ++itF) {
      if (itF->second < closestToOrigin->second)
        closestToOrigin = itF;
    }
    searchDirection = closestToOrigin->first->N;
    user.getSupportMinkowskiDiff(lastPlex.getPair(), searchDirection,
                                 newVertex);
    diff(V_temp, newVertex.vertexDiff, *closestToOrigin->first->A);
    temp = dot(closestToOrigin->first->N, V_temp);
#ifdef FLX_LOGGER_ENABLED
    iterations.add(this->print(Minkowski_diff, *closestToOrigin->first));
#endif
    if (temp < GEOMETRIC_TOLLERANCE)
      break;
    Minkowski_diff.UpdateHull(newVertex.vertexDiff);
    originalVertices.emplace(
        &Minkowski_diff.getVertices().back(),
        MinkowskiPair(newVertex.vertexA, newVertex.vertexB));
  } while (true);
#ifdef FLX_LOGGER_ENABLED
  log->add(iterations.str());
#endif
  // get penetration vector
  float coeff[3];
  getClosestInTriangle(*closestToOrigin->first->A, *closestToOrigin->first->B,
                       *closestToOrigin->first->C, coeff);
  const MinkowskiPair *originalFacet[3];
  originalFacet[0] = &originalVertices.find(closestToOrigin->first->A)->second;
  originalFacet[1] = &originalVertices.find(closestToOrigin->first->B)->second;
  originalFacet[2] = &originalVertices.find(closestToOrigin->first->C)->second;
  mix3(result.pointA, originalFacet[0]->vertexA, originalFacet[1]->vertexA,
       originalFacet[2]->vertexA, coeff);
  mix3(result.pointB, originalFacet[0]->vertexB, originalFacet[1]->vertexB,
       originalFacet[2]->vertexB, coeff);
};

std::array<GjkEpa::MinkowskiCoordinate, 4>
GjkEpa::Epa::getInitialTethraedron(GjkEpa::Plex &lastPlex) {
  std::array<GjkEpa::MinkowskiCoordinate, 4> initialVertices;
  std::uint8_t size = lastPlex.getPlexDimension();
  {
    const GjkEpa::Plex::MinkowskiCoordinatePtr *v = lastPlex.getVertices();
    for (std::uint8_t k = 0; k < size; ++k)
      initialVertices[k] = *v[1 + k].get();
  }
  while (size < 4) {
    if (1 == size) {
      searchDirection = initialVertices[0].vertexDiff;
      if (normSquared(searchDirection) <= GEOMETRIC_TOLLERANCE2) {
        searchDirection.x = 1.f;
        searchDirection.y = 0.f;
        searchDirection.z = 0.f;
      } else
        invert(searchDirection);
      lastPlex.getUser().getSupportMinkowskiDiff(
          lastPlex.getPair(), searchDirection, initialVertices[1]);
      if (squaredDistance(initialVertices[0].vertexDiff,
                          initialVertices[1].vertexDiff) <=
          GEOMETRIC_TOLLERANCE2) {
        invert(searchDirection);
        lastPlex.getUser().getSupportMinkowskiDiff(
            lastPlex.getPair(), searchDirection, initialVertices[1]);
      }
    } else if (2 == size) {
      hull::Coordinate D;
      diff(D, initialVertices[0].vertexDiff, initialVertices[1].vertexDiff);
      searchDirection = hull::Coordinate{1.f, 0.f, 0.f};
      searchDirection = cross(D, searchDirection);
      if (normSquared(searchDirection) <= GEOMETRIC_TOLLERANCE2) {
        searchDirection.x = 0.f;
        searchDirection.y = 1.f;
        searchDirection.z = 0.f;
        searchDirection = cross(D, searchDirection);
      }
      lastPlex.getUser().getSupportMinkowskiDiff(
          lastPlex.getPair(), searchDirection, initialVertices[2]);
      bool tempB[2];
      tempB[0] = (squaredDistance(initialVertices[2].vertexDiff,
                                  initialVertices[0].vertexDiff) <=
                  GEOMETRIC_TOLLERANCE2);
      tempB[1] = (squaredDistance(initialVertices[2].vertexDiff,
                                  initialVertices[1].vertexDiff) <=
                  GEOMETRIC_TOLLERANCE2);
      if (tempB[0] || tempB[1]) {
        invert(searchDirection);
        lastPlex.getUser().getSupportMinkowskiDiff(
            lastPlex.getPair(), searchDirection, initialVertices[2]);
      }
    } else {
      computeOutsideNormal(searchDirection, initialVertices[0].vertexDiff,
                           initialVertices[1].vertexDiff,
                           initialVertices[2].vertexDiff, ORIGIN);
      invert(searchDirection);
      lastPlex.getUser().getSupportMinkowskiDiff(
          lastPlex.getPair(), searchDirection, initialVertices[3]);
      hull::Coordinate Delta;
      diff(Delta, initialVertices[3].vertexDiff, initialVertices[0].vertexDiff);
      if (dot(Delta, searchDirection) <= GEOMETRIC_TOLLERANCE) {
        invert(searchDirection);
        lastPlex.getUser().getSupportMinkowskiDiff(
            lastPlex.getPair(), searchDirection, initialVertices[3]);
      }
    }
    ++size;
  }
  return initialVertices;
}

void GjkEpa::Epa::AddedChangedFacets(
    const std::list<const hull::Facet *> &added,
    const std::list<const hull::Facet *> &changed) const {
  auto it = added.begin();
  for (it; it != added.end(); ++it)
    this->facetDistances.emplace(*it, dot((*it)->N, *(*it)->A));
  for (it = changed.begin(); it != changed.end(); ++it)
    this->facetDistances.find(*it)->second = dot((*it)->N, *(*it)->A);
};

void GjkEpa::Epa::RemovedFacets(
    const std::list<const hull::Facet *> &removed) const {
  auto itEnd = removed.end();
  for (auto it = removed.begin(); it != itEnd; ++it)
    this->facetDistances.erase(this->facetDistances.find(*it));
};

#ifdef FLX_LOGGER_ENABLED
std::string GjkEpa::Epa::print(const hull::Hull &h,
                               const hull::Facet &closest) const {
  std::stringstream ss;
  ss << '{';
  ss << "\"facets\":";
  {
    Array facets;
    for (auto it = h.getFacets().begin(); it != h.getFacets().end(); ++it) {
      Array temp;
      temp.add(str(*it->A));
      temp.add(str(*it->B));
      temp.add(str(*it->C));
      temp.add(str(it->N));
      facets.add(temp.str());
    }
    ss << facets.str();
  }
  ss << '\n' << ",\"closest\":";
  {
    Array temp;
    temp.add(str(*closest.A));
    temp.add(str(*closest.B));
    temp.add(str(*closest.C));
    ss << temp.str();
  }
  ss << '}';
  return ss.str();
}
#endif
} // namespace flx
