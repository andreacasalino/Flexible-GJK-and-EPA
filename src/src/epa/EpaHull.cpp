#include "EpaHull.h"

#include <algorithm>

namespace flx::epa {
EpaHull::EpaHull(const MinkowskiDifference &minkowski_diff,
                 const std::vector<MinkowskiDiffCoordinate> &initial_vertices)
    : mink_diff(minkowski_diff) {
  this->vertices = initial_vertices;
  this->hull =
      std::make_unique<hull::Hull>(vertices[0].vertex_in_Minkowski_diff,
                                   vertices[1].vertex_in_Minkowski_diff,
                                   vertices[2].vertex_in_Minkowski_diff,
                                   vertices[3].vertex_in_Minkowski_diff, *this);
};

bool EpaHull::update() {
  const hull::Facet &closest_facet = *distances_facets_map.begin()->second;
  mink_diff.getSupport(vertices.emplace_back(), closest_facet.normal);
  hull::Coordinate improvement =
      delta(vertices.back().vertex_in_Minkowski_diff,
            vertices[closest_facet.vertexA].vertex_in_Minkowski_diff);
  if (is_greater(hull::dot(improvement, closest_facet.normal),
                 hull::HULL_GEOMETRIC_TOLLERANCE)) {
    hull->update(vertices.back().vertex_in_Minkowski_diff);
    return true;
  }
  return false;
};

std::array<const MinkowskiDiffCoordinate *, 3>
EpaHull::getClosestFacetToOrigin() const {
  const hull::Facet &closest_facet = *distances_facets_map.begin()->second;
  std::array<const MinkowskiDiffCoordinate *, 3> result;
  result[0] = &vertices[closest_facet.vertexA];
  result[1] = &vertices[closest_facet.vertexB];
  result[2] = &vertices[closest_facet.vertexC];
  return result;
};

void EpaHull::hullChanges(Notification &&notification) {
  for (const auto *added : notification.added) {
    distances_facets_map.emplace(getSquaredDistanceToOrigin(added), added);
  }
  for (const auto *changed : notification.changed) {
    distances_facets_map.erase(findDistanceFacetPair(changed));
    distances_facets_map.emplace(getSquaredDistanceToOrigin(changed), changed);
  }
  for (const auto &removed : notification.removed) {
    distances_facets_map.erase(findDistanceFacetPair(removed.get()));
  }
};

std::multimap<float, const hull::Facet *>::const_iterator
EpaHull::findDistanceFacetPair(const hull::Facet *facet) const {
  return std::find_if(
      distances_facets_map.begin(), distances_facets_map.end(),
      [&facet](const std::pair<float, const hull::Facet *> &element) {
        return element.second == facet;
      });
}

float EpaHull::getSquaredDistanceToOrigin(const hull::Facet *facet) const {
  const auto &A = vertices[facet->vertexA].vertex_in_Minkowski_diff;
  const auto &B = vertices[facet->vertexB].vertex_in_Minkowski_diff;
  const auto &C = vertices[facet->vertexC].vertex_in_Minkowski_diff;
  auto closest_info = getClosestToOriginInTriangle(A, B, C);
  return hull::normSquared(mix3(A, B, C, closest_info.coefficients));
}

#ifdef GJK_EPA_DIAGNOSTIC
nlohmann::json EpaHull::toJson() const {
  nlohmann::json recipient;
  auto facet_to_json = [&hull = this->hull](nlohmann::json &recipient,
                                            const hull::Facet &facet) {
    diagnostic::to_json(recipient["A"], hull->getVertices()[facet.vertexA]);
    diagnostic::to_json(recipient["B"], hull->getVertices()[facet.vertexB]);
    diagnostic::to_json(recipient["C"], hull->getVertices()[facet.vertexC]);
  };

  auto &facets = recipient["facets"];
  facets = nlohmann::json::array();
  for (const auto &facet : hull->getFacets()) {
    facet_to_json(facets.emplace_back(), *facet);
  }
  facet_to_json(recipient["closest"], *distances_facets_map.begin()->second);
  return recipient;
}
#endif
} // namespace flx::epa
