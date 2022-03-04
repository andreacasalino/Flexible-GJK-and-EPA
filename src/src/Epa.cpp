/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Epa.h"
#include "Commons.h"

#include <Hull/Hull.h>

#include <algorithm>
#include <map>

namespace flx {
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
initial_thetraedron(const ShapePair &pair, const Plex &initial_plex) {
  struct Visitor {
    mutable std::vector<MinkowskiDiffCoordinate> result;

    void operator()(const VertexCase &subject) const {
      if (hull::squaredDistance(
              subject.data->vertices[0]->vertex_in_Minkowski_diff,
              subject.data->vertices[1]->vertex_in_Minkowski_diff) <=
          GEOMETRIC_TOLLERANCE2) {
        result =
            std::vector<MinkowskiDiffCoordinate>{*subject.data->vertices[0]};
      } else {
        result = std::vector<MinkowskiDiffCoordinate>{
            *subject.data->vertices[0], *subject.data->vertices[1]};
      }
    }

    void operator()(const SegmentCase &subject) const {
      result = std::vector<MinkowskiDiffCoordinate>{*subject.data->vertices[0],
                                                    *subject.data->vertices[1],
                                                    *subject.data->vertices[2]};
    }

    void operator()(const FacetCase &subject) const {
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
  void toJson(nlohmann::json &recipient) const;
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
void EpaHull::toJson(nlohmann::json &recipient) const {
  auto &facets = recipient["facets"];
  facets = nlohmann::json::array();
  for (const auto &facet : hull->getFacets()) {
    auto &facet_json = facets.emplace_back();
    diagnostic::to_json(facet_json["A"], hull->getVertices()[facet->vertexA]);
    diagnostic::to_json(facet_json["B"], hull->getVertices()[facet->vertexB]);
    diagnostic::to_json(facet_json["C"], hull->getVertices()[facet->vertexC]);
  }
}
#endif
} // namespace

CoordinatePair EPA(const ShapePair &pair, const Plex &initial_plex
#ifdef GJK_EPA_DIAGNOSTIC
                   ,
                   nlohmann::json &log
#endif
) {
#ifdef GJK_EPA_DIAGNOSTIC
  auto &epa_log = log["EPA"];
  epa_log = nlohmann::json::array();
#endif
  EpaHull epa_hull(pair, initial_plex);
#ifdef GJK_EPA_DIAGNOSTIC
  epa_hull.toJson(epa_log.emplace_back());
#endif
  while (epa_hull.update()) {
#ifdef GJK_EPA_DIAGNOSTIC
    epa_hull.toJson(epa_log.emplace_back());
#endif
  }
  auto closest_facet = epa_hull.getClosestFacetToOrigin();
  auto closest_info =
      getClosestToOriginInTriangle(closest_facet[0]->vertex_in_Minkowski_diff,
                                   closest_facet[1]->vertex_in_Minkowski_diff,
                                   closest_facet[2]->vertex_in_Minkowski_diff);
  return CoordinatePair{
      mix3(closest_facet[0]->vertex_in_shape_a,
           closest_facet[1]->vertex_in_shape_a,
           closest_facet[2]->vertex_in_shape_a, closest_info.coefficients),
      mix3(closest_facet[0]->vertex_in_shape_b,
           closest_facet[1]->vertex_in_shape_b,
           closest_facet[2]->vertex_in_shape_b, closest_info.coefficients)};
}
} // namespace flx
