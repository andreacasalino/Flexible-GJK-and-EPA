#include "EpaHull.h"

namespace flx::epa {
class EpaHull : public hull::Observer {
public:
  EpaHull(const ShapePair &pair, const Plex &initial_plex) : mink_diff(pair) {
    this->vertices = initial_thetraedron(pair, initial_plex);
    this->hull = std::make_unique<hull::Hull>(
        vertices[0].vertex_in_Minkowski_diff,
        vertices[1].vertex_in_Minkowski_diff,
        vertices[2].vertex_in_Minkowski_diff,
        vertices[3].vertex_in_Minkowski_diff, *this);
  };

  bool update() {
    const hull::Facet &closest_facet = *distances_facets_map.begin()->second;
    mink_diff.getSupport(vertices.emplace_back(), closest_facet.normal);
    hull::Coordinate improvement;
    hull::diff(improvement, vertices.back().vertex_in_Minkowski_diff,
               vertices[closest_facet.vertexA].vertex_in_Minkowski_diff);
    if (hull::dot(improvement, closest_facet.normal) <=
        hull::HULL_GEOMETRIC_TOLLERANCE) {
      return false;
    }
    hull->update(vertices.back().vertex_in_Minkowski_diff);
    return true;
  };

  std::array<const MinkowskiDiffCoordinate *, 3>
  getClosestFacetToOrigin() const {
    const hull::Facet &closest_facet = *distances_facets_map.begin()->second;
    std::array<const MinkowskiDiffCoordinate *, 3> result;
    result[0] = &vertices[closest_facet.vertexA];
    result[1] = &vertices[closest_facet.vertexB];
    result[2] = &vertices[closest_facet.vertexC];
    return result;
  };

#ifdef GJK_EPA_DIAGNOSTIC
  nlohmann::json toJson() const;
#endif

protected:
  void hullChanges(Notification &&notification) final {
    for (const auto *added : notification.added) {
      distances_facets_map.emplace(getSquaredDistanceToOrigin(added), added);
    }
    for (const auto *changed : notification.changed) {
      distances_facets_map.erase(getDistanceFacetPair(changed));
      distances_facets_map.emplace(getSquaredDistanceToOrigin(changed),
                                   changed);
    }
    for (const auto &removed : notification.removed) {
      distances_facets_map.erase(getDistanceFacetPair(removed.get()));
    }
  };

  std::multimap<float, const hull::Facet *>::const_iterator
  getDistanceFacetPair(const hull::Facet *facet) const {
    return std::find_if(
        distances_facets_map.begin(), distances_facets_map.end(),
        [&facet](const std::pair<float, const hull::Facet *> &element) {
          return element.second == facet;
        });
  }

  float getSquaredDistanceToOrigin(const hull::Facet *facet) const {
    const auto &A = vertices[facet->vertexA].vertex_in_Minkowski_diff;
    const auto &B = vertices[facet->vertexB].vertex_in_Minkowski_diff;
    const auto &C = vertices[facet->vertexC].vertex_in_Minkowski_diff;
    auto closest_info = getClosestToOriginInTriangle(A, B, C);
    return hull::normSquared(mix3(A, B, C, closest_info.coefficients));
  }

  MinkowskiDifference mink_diff;
  std::unique_ptr<hull::Hull> hull;
  std::vector<MinkowskiDiffCoordinate> vertices;
  std::multimap<float, const hull::Facet *> distances_facets_map;
};

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
