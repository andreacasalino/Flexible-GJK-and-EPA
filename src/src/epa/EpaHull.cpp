#include <Flexible-GJK-and-EPA/epa/EpaHull.h>

#include <algorithm>

namespace flx::epa {
EpaHull::EpaHull(const MinkowskiDifference &minkowski_diff,
                 const std::vector<MinkowskiDiffCoordinate> &initial_vertices)
    : mink_diff(minkowski_diff) {
  this->vertices = initial_vertices;
  this->hull.emplace(vertices[0].vertex_in_Minkowski_diff,
                     vertices[1].vertex_in_Minkowski_diff,
                     vertices[2].vertex_in_Minkowski_diff,
                     vertices[3].vertex_in_Minkowski_diff, *this);
};

bool EpaHull::update() {
  const hull::Facet &closest_facet = *distances_facets_map.begin()->facet;
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
  const hull::Facet &closest_facet = *distances_facets_map.begin()->facet;
  std::array<const MinkowskiDiffCoordinate *, 3> result;
  result[0] = &vertices[closest_facet.vertexA];
  result[1] = &vertices[closest_facet.vertexB];
  result[2] = &vertices[closest_facet.vertexC];
  return result;
};

void EpaHull::hullChanges(const Notification &notification) {
  for (const auto *added : notification.added) {
    facets_table[added] = distances_facets_map.emplace(
        DistanceInfo{getSquaredDistanceToOrigin(added), added});
  }
  for (const auto *changed : notification.changed) {
    auto table_it = facets_table.find(changed);
    distances_facets_map.erase(table_it->second);
    facets_table.erase(table_it);
    facets_table[changed] = distances_facets_map.emplace(
        DistanceInfo{getSquaredDistanceToOrigin(changed), changed});
  }
  for (const auto &removed : notification.removed) {
    auto table_it = facets_table.find(removed);
    distances_facets_map.erase(table_it->second);
    facets_table.erase(table_it);
  }
};

float EpaHull::getSquaredDistanceToOrigin(const hull::Facet *facet) {
  const auto &A = vertices[facet->vertexA].vertex_in_Minkowski_diff;
  const auto &B = vertices[facet->vertexB].vertex_in_Minkowski_diff;
  const auto &C = vertices[facet->vertexC].vertex_in_Minkowski_diff;
  auto closest_info = getClosestToOriginInTriangle(A, B, C);
  return hull::normSquared(mix3(A, B, C, closest_info.coefficients));
}
} // namespace flx::epa
