/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Epa.h"
#include "Commons.h"

#include <Hull/Hull.h>

#include <map>

namespace flx {
namespace {
float get_squared_distance_to_origin(const hull::Coordinate &a,
                                     const hull::Coordinate &b,
                                     const hull::Coordinate &c) {
  auto coeff = getClosestToOriginInTriangle(a, b, c);
  return hull::normSquared(mix3(a, b, c, coeff.coefficients));
}

class EpaHull : public hull::Observer {
public:
  EpaHull(const ShapePair &pair, const Plex &initial_plex);

  bool update() {
    const hull::Facet &closest_facet =
        *origin_distances_facets_map.begin()->second;
    getSupportMinkowskiDiff(pair, closest_facet.normal,
                            vertices.emplace_back());
    if (
        // check new vertex is farther from origin than closest_facet
    ) {
      return false;
    }
    hull.update(vertices.back().vertex_in_Minkowski_diff);
    return true;
  };

  std::array<const MinkowskiDiffCoordinate *, 3>
  getClosestFacetToOrigin() const {
    const hull::Facet &closest_facet =
        *origin_distances_facets_map.begin()->second;
    std::array<const MinkowskiDiffCoordinate *, 3> result;
    result[0] = &vertices[closest_facet.vertexA];
    result[1] = &vertices[closest_facet.vertexB];
    result[2] = &vertices[closest_facet.vertexC];
    return result;
  };

protected:
  void hullChanges(Notification &&notification) final {
    for (const auto *added : notification.added) {
      origin_distances_facets_map.emplace(get_squared_distance_to_origin(, , ),
                                          added);
    }

    for (const auto &removed : notification.removed) {
    }
  };

private:
  ShapePair pair;
  hull::Hull hull;
  std::vector<MinkowskiDiffCoordinate> vertices;
  std::multimap<float, const hull::Facet *> origin_distances_facets_map;
};
} // namespace

CoordinatePair EPA(const ShapePair &pair, const Plex &initial_plex) {
  EpaHull epa_hull(pair, initial_plex);
  while (epa_hull.update()) {
  }
  // auto closest_facet = epa_hull.getClosestFacetToOrigin();
  // auto coefficients =
  //     getClosestToOriginInTriangle(closest_facet[0]->vertex_in_Minkowski_diff,
  //                                  closest_facet[1]->vertex_in_Minkowski_diff,
  //                                  closest_facet[2]->vertex_in_Minkowski_diff);
  // return CoordinatePair{mix3(closest_facet[0]->vertex_in_shape_a,
  //                            closest_facet[1]->vertex_in_shape_a,
  //                            closest_facet[2]->vertex_in_shape_a,
  //                            coefficients),
  //                       mix3(closest_facet[0]->vertex_in_shape_b,
  //                            closest_facet[1]->vertex_in_shape_b,
  //                            closest_facet[2]->vertex_in_shape_b,
  //                            coefficients)};
}
} // namespace flx
