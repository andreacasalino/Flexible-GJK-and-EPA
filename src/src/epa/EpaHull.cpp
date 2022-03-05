#include "EpaHull.h"

#include <algorithm>

namespace flx::epa {
namespace {
MinkowskiDiffCoordinate find_vertex_trying_direction_and_opposite(
    const MinkowskiDifference &mink_diff,
    const MinkowskiDiffCoordinate &existing_vertex,
    hull::Coordinate direction) {
  hull::normalizeInPlace(direction);
  MinkowskiDiffCoordinate result;
  mink_diff.getSupport(result, direction);
  hull::Coordinate delta;
  hull::diff(delta, result.vertex_in_Minkowski_diff,
             existing_vertex.vertex_in_Minkowski_diff);
  if (hull::dot(direction, delta) <= hull::HULL_GEOMETRIC_TOLLERANCE) {
    hull::invert(direction);
    mink_diff.getSupport(result, direction);
  }
  return result;
}

std::vector<MinkowskiDiffCoordinate>
initial_thetraedron(const ShapePair &pair, const gjk::Plex &initial_plex) {
  struct Visitor {
    mutable std::vector<MinkowskiDiffCoordinate> result;

    void operator()(const gjk::VertexCase &subject) const {
      if (hull::squaredDistance(
              subject.data->vertices[0]->vertex_in_Minkowski_diff,
              subject.data->vertices[1]->vertex_in_Minkowski_diff) <=
          GEOMETRIC_TOLLERANCE_SQUARED) {
        result =
            std::vector<MinkowskiDiffCoordinate>{*subject.data->vertices[0]};
      } else {
        result = std::vector<MinkowskiDiffCoordinate>{
            *subject.data->vertices[0], *subject.data->vertices[1]};
      }
    }

    void operator()(const gjk::SegmentCase &subject) const {
      result = std::vector<MinkowskiDiffCoordinate>{*subject.data->vertices[0],
                                                    *subject.data->vertices[1],
                                                    *subject.data->vertices[2]};
    }

    void operator()(const gjk::FacetCase &subject) const {
      result = std::vector<MinkowskiDiffCoordinate>{
          *subject.data->vertices[0], *subject.data->vertices[1],
          *subject.data->vertices[2], *subject.data->vertices[3]};
    }
  } visitor;
  std::visit(visitor, initial_plex);
  std::vector<MinkowskiDiffCoordinate> result = std::move(visitor.result);

  MinkowskiDifference mink_diff(pair);

  while (result.size() < 4) {
    switch (result.size()) {
    case 1:
      result.push_back(find_vertex_trying_direction_and_opposite(
          mink_diff, result.front(), hull::Coordinate{1.f, 0, 0}));
      break;
    case 2: {
      hull::Coordinate delta;
      hull::diff(delta, result[1].vertex_in_Minkowski_diff,
                 result[0].vertex_in_Minkowski_diff);
      auto direction1 = hull::cross(delta, hull::Coordinate{1.f, 0, 0});
      auto direction2 = hull::cross(delta, hull::Coordinate{0, 1.f, 0});
      auto &direction = direction1;
      if (hull::normSquared(direction2) > hull::normSquared(direction1)) {
        direction = direction2;
      }
      result.push_back(find_vertex_trying_direction_and_opposite(
          mink_diff, result.front(), direction));
    } break;
    case 3:
      hull::Coordinate delta_AB;
      hull::diff(delta_AB, result[0].vertex_in_Minkowski_diff,
                 result[1].vertex_in_Minkowski_diff);
      hull::Coordinate delta_AC;
      hull::diff(delta_AC, result[0].vertex_in_Minkowski_diff,
                 result[2].vertex_in_Minkowski_diff);
      result.push_back(find_vertex_trying_direction_and_opposite(
          mink_diff, result.front(), hull::cross(delta_AB, delta_AC)));
      break;
    }
  }
  return result;
};
} // namespace

EpaHull::EpaHull(const ShapePair &pair, const gjk::Plex &initial_plex)
    : mink_diff(pair) {
  this->vertices = initial_thetraedron(pair, initial_plex);
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
  if (hull::dot(improvement, closest_facet.normal) <=
      hull::HULL_GEOMETRIC_TOLLERANCE) {
    return false;
  }
  hull->update(vertices.back().vertex_in_Minkowski_diff);
  return true;
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
