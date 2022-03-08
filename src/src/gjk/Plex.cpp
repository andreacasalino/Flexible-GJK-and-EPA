/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Plex.h"
#include <Flexible-GJK-and-EPA/Error.h>

#include <limits>

namespace flx::gjk {
void SearcDirection::udpate(const hull::Coordinate &new_direction) {
  search_direction = new_direction;
  hull::normalizeInPlace(search_direction);
}

VertexCase set_to_vertex(const PlexDataPtr &data) {
  hull::Coordinate direction = data->vertices.front()->vertex_in_Minkowski_diff;
  hull::invert(direction);
  data->search_direction.udpate(direction);
  std::swap(data->vertices[0], data->vertices[1]);
  return VertexCase{data};
}

namespace {
enum SegmentUpdateCase { AB, AC, AD };
SegmentCase set_to_segment(const PlexDataPtr &data,
                           const SegmentUpdateCase segment_case) {
  hull::Coordinate *A = &data->vertices[0]->vertex_in_Minkowski_diff;
  hull::Coordinate *B = nullptr;
  switch (segment_case) {
  case SegmentUpdateCase::AB:
    B = &data->vertices[1]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[2], data->vertices[1]);
    std::swap(data->vertices[1], data->vertices[0]);
    break;
  case SegmentUpdateCase::AC:
    B = &data->vertices[2]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[0], data->vertices[1]);
    break;
  case SegmentUpdateCase::AD:
    B = &data->vertices[3]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[0], data->vertices[1]);
    std::swap(data->vertices[2], data->vertices[3]);
    break;
  }
  hull::Coordinate search_direction = hull::cross(*A, *B);
  search_direction = cross(search_direction, delta(*B, *A));
  data->search_direction.udpate(search_direction);
  return SegmentCase{data};
}

enum FacetUpdateCase { ABC, ABD, ACD };
FacetCase set_to_facet(const PlexDataPtr &data,
                       const FacetUpdateCase facet_case,
                       const hull::Coordinate *outward_normals) {
  hull::Coordinate *A = &data->vertices[0]->vertex_in_Minkowski_diff;
  hull::Coordinate *B = nullptr;
  hull::Coordinate *C = nullptr;
  switch (facet_case) {
  case FacetUpdateCase::ABC:
    B = &data->vertices[1]->vertex_in_Minkowski_diff;
    C = &data->vertices[2]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[2], data->vertices[3]);
    std::swap(data->vertices[1], data->vertices[2]);
    std::swap(data->vertices[0], data->vertices[1]);
    data->search_direction.udpateAsIs(outward_normals[0]);
    break;
  case FacetUpdateCase::ABD:
    B = &data->vertices[1]->vertex_in_Minkowski_diff;
    C = &data->vertices[3]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[1], data->vertices[2]);
    std::swap(data->vertices[0], data->vertices[1]);
    data->search_direction.udpateAsIs(outward_normals[1]);
    break;
  case FacetUpdateCase::ACD:
    B = &data->vertices[2]->vertex_in_Minkowski_diff;
    C = &data->vertices[3]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[0], data->vertices[1]);
    data->search_direction.udpateAsIs(outward_normals[2]);
    break;
  }
  return FacetCase{data};
}

bool are_outwardnormal_and_origin_in_same_direction(
    const hull::Coordinate &first, const hull::Coordinate &second,
    const hull::Coordinate &third, const hull::Coordinate &other,
    hull::Coordinate &normal) {
  normal = computeOutsideNormal(first, second, third, other);
  return is_lower(hull::dot(normal, first), hull::HULL_GEOMETRIC_TOLLERANCE);
};

constexpr float MAX_DISTANCE = std::numeric_limits<float>::max();
} // namespace

namespace {
static inline const uint8_t INCIDENCES[3][3] = {
    {0, 1, 2}, {0, 1, 3}, {0, 2, 3}};

#ifdef GJK_EPA_DIAGNOSTIC
void to_json(nlohmann::json &recipient, const ClosestRegionToOrigin &region) {
  switch (region) {
  case vertex_A:
    recipient = "vertex_A";
    break;
  case edge_AB:
    recipient = "edge_AB";
    break;
  case edge_AC:
    recipient = "edge_AC";
    break;
  case face_ABC:
    recipient = "face_ABC";
    break;
  }
}

void to_json(nlohmann::json &recipient, const PlexData &data,
             const std::size_t size) {
  diagnostic::to_json(recipient["direction"], data.search_direction.get());
  auto &plex = recipient["plex"];
  plex = nlohmann::json::array();
  for (std::size_t k = 0; k < size; ++k) {
    diagnostic::to_json(plex.emplace_back(),
                        data.vertices[k]->vertex_in_Minkowski_diff);
  }
}

void tp_json_closest(nlohmann::json &recipient,
                     const std::array<float, 2> &coeff,
                     const MinkowskiCoordinates &coordinates) {
  hull::Coordinate closest;
  closest = mix2(coordinates[0]->vertex_in_Minkowski_diff,
                 coordinates[1]->vertex_in_Minkowski_diff, coeff);
  diagnostic::to_json(recipient, closest);
}

void tp_json_closest(nlohmann::json &recipient,
                     const std::array<float, 3> &coeff,
                     const MinkowskiCoordinates &coordinates) {
  hull::Coordinate closest;
  closest = mix3(coordinates[0]->vertex_in_Minkowski_diff,
                 coordinates[1]->vertex_in_Minkowski_diff,
                 coordinates[2]->vertex_in_Minkowski_diff, coeff);
  diagnostic::to_json(recipient, closest);
}
#endif

PlexUpdateResult update_segment(const VertexCase &subject
#ifdef GJK_EPA_DIAGNOSTIC
                                ,
                                nlohmann::json &log
#endif
) {
  auto &data = *subject.data;
#ifdef GJK_EPA_DIAGNOSTIC
  to_json(log, data, 2);
#endif
  hull::Coordinate temp =
      hull::cross(data.vertices[0]->vertex_in_Minkowski_diff,
                  data.vertices[1]->vertex_in_Minkowski_diff);
  if (normSquared(temp) <= GEOMETRIC_TOLLERANCE_SQUARED_SQUARED) {
#ifdef GJK_EPA_DIAGNOSTIC
    log["collision"] = true;
#endif
    return CollisionCase{};
  }
  auto closest =
      getClosestToOriginInSegment(data.vertices[0]->vertex_in_Minkowski_diff,
                                  data.vertices[1]->vertex_in_Minkowski_diff);
#ifdef GJK_EPA_DIAGNOSTIC
  tp_json_closest(log["closest"]["point"], closest.coefficients, data.vertices);
  to_json(log["closest"]["region"], closest.region);
#endif
  if (vertex_A == closest.region) {
    return set_to_vertex(subject.data);
  }
  return set_to_segment(subject.data, SegmentUpdateCase::AB);
}

PlexUpdateResult update_facet(const SegmentCase &subject
#ifdef GJK_EPA_DIAGNOSTIC
                              ,
                              nlohmann::json &log
#endif
) {
  auto &data = *subject.data;
#ifdef GJK_EPA_DIAGNOSTIC
  to_json(log, data, 3);
#endif
  auto closest =
      getClosestToOriginInTriangle(data.vertices[0]->vertex_in_Minkowski_diff,
                                   data.vertices[1]->vertex_in_Minkowski_diff,
                                   data.vertices[2]->vertex_in_Minkowski_diff);
  hull::Coordinate temp =
      mix3(data.vertices[0]->vertex_in_Minkowski_diff,
           data.vertices[1]->vertex_in_Minkowski_diff,
           data.vertices[2]->vertex_in_Minkowski_diff, closest.coefficients);
#ifdef GJK_EPA_DIAGNOSTIC
  diagnostic::to_json(log["closest"]["point"], temp);
  to_json(log["closest"]["region"], closest.region);
#endif
  if (normSquared(temp) <= GEOMETRIC_TOLLERANCE_SQUARED) {
#ifdef GJK_EPA_DIAGNOSTIC
    log["collision"] = true;
#endif
    return CollisionCase{};
  }
  switch (closest.region) {
  case ClosestRegionToOrigin::face_ABC: {
    hull::Coordinate normal = computeOutsideNormal(
        data.vertices[0]->vertex_in_Minkowski_diff,
        data.vertices[1]->vertex_in_Minkowski_diff,
        data.vertices[2]->vertex_in_Minkowski_diff, hull::ORIGIN);
    hull::invert(normal);
    return set_to_facet(subject.data, FacetUpdateCase::ABC, &normal);
  }
  case ClosestRegionToOrigin::vertex_A:
    return set_to_vertex(subject.data);
  case ClosestRegionToOrigin::edge_AB:
    return set_to_segment(subject.data, SegmentUpdateCase::AB);
  case ClosestRegionToOrigin::edge_AC:
    return set_to_segment(subject.data, SegmentUpdateCase::AC);
  }
  throw Error{"Internal error updating plex"};
}

PlexUpdateResult update_tethreadron(const FacetCase &subject
#ifdef GJK_EPA_DIAGNOSTIC
                                    ,
                                    nlohmann::json &log
#endif
) {
  auto &data = *subject.data;
#ifdef GJK_EPA_DIAGNOSTIC
  to_json(log, data, 4);
#endif
  std::array<hull::Coordinate, 3> normals;
  std::array<bool, 3> is_origin_visible;
  // update visibility flags
  is_origin_visible[0] = are_outwardnormal_and_origin_in_same_direction(
      data.vertices[0]->vertex_in_Minkowski_diff,
      data.vertices[1]->vertex_in_Minkowski_diff,
      data.vertices[2]->vertex_in_Minkowski_diff,
      data.vertices[3]->vertex_in_Minkowski_diff, normals[0]);
  is_origin_visible[1] = are_outwardnormal_and_origin_in_same_direction(
      data.vertices[0]->vertex_in_Minkowski_diff,
      data.vertices[1]->vertex_in_Minkowski_diff,
      data.vertices[3]->vertex_in_Minkowski_diff,
      data.vertices[2]->vertex_in_Minkowski_diff, normals[1]);
  is_origin_visible[2] = are_outwardnormal_and_origin_in_same_direction(
      data.vertices[0]->vertex_in_Minkowski_diff,
      data.vertices[2]->vertex_in_Minkowski_diff,
      data.vertices[3]->vertex_in_Minkowski_diff,
      data.vertices[1]->vertex_in_Minkowski_diff, normals[2]);
#ifdef GJK_EPA_DIAGNOSTIC
  auto &log_facets = log["facets"];
  auto &log_normals = log_facets["normals"];
  log_normals = nlohmann::json::array();
  for (const auto &normal : normals) {
    diagnostic::to_json(log_normals.emplace_back(), normal);
  }
  log_facets["visibility"] = is_origin_visible;
#endif

  // check contains origin
  if (!(is_origin_visible[0] || is_origin_visible[1] || is_origin_visible[2])) {
#ifdef GJK_EPA_DIAGNOSTIC
    log["collision"] = true;
#endif
    return CollisionCase{};
  }

  // update plex
  ClosestRegionToOrigin regions[3];
  float distances[3];
#ifdef GJK_EPA_DIAGNOSTIC
  auto &log_facets_closest = log_facets["closest"];
  log_facets_closest = nlohmann::json::array();
#endif
  for (std::uint8_t k = 0; k < 3; ++k) {
#ifdef GJK_EPA_DIAGNOSTIC
    auto &closest_info = log_facets_closest.emplace_back();
#endif
    if (is_origin_visible[k]) {
      auto [region, coeff] = getClosestToOriginInTriangle(
          data.vertices[INCIDENCES[k][0]]->vertex_in_Minkowski_diff,
          data.vertices[INCIDENCES[k][1]]->vertex_in_Minkowski_diff,
          data.vertices[INCIDENCES[k][2]]->vertex_in_Minkowski_diff);
      hull::Coordinate temp = mix3(
          data.vertices[INCIDENCES[k][0]]->vertex_in_Minkowski_diff,
          data.vertices[INCIDENCES[k][1]]->vertex_in_Minkowski_diff,
          data.vertices[INCIDENCES[k][2]]->vertex_in_Minkowski_diff, coeff);
      distances[k] = normSquared(temp);
      regions[k] = region;
#ifdef GJK_EPA_DIAGNOSTIC
      diagnostic::to_json(closest_info["point"], temp);
      tp_json_closest(closest_info["region"], coeff, data.vertices);
#endif
    } else {
      distances[k] = MAX_DISTANCE;
#ifdef GJK_EPA_DIAGNOSTIC
      closest_info = nullptr;
#endif
    }
  }

  std::uint8_t closest_facet = 0;
  if (distances[1] < distances[closest_facet])
    closest_facet = 1;
  if (distances[2] < distances[closest_facet])
    closest_facet = 2;

  if (regions[closest_facet] == vertex_A) {
    return set_to_vertex(subject.data);
  }

  if (regions[closest_facet] == face_ABC) {
    switch (closest_facet) {
    case 0:
      return set_to_facet(subject.data, FacetUpdateCase::ABC, normals.data());
    case 1:
      return set_to_facet(subject.data, FacetUpdateCase::ABD, normals.data());
    case 2:
      return set_to_facet(subject.data, FacetUpdateCase::ACD, normals.data());
    }
  }

  switch (closest_facet) {
  case 0:
    if (edge_AB == regions[closest_facet])
      return set_to_segment(subject.data, SegmentUpdateCase::AB);
    else
      return set_to_segment(subject.data, SegmentUpdateCase::AC);
  case 1:
    if (edge_AB == regions[closest_facet])
      return set_to_segment(subject.data, SegmentUpdateCase::AB);
    else
      return set_to_segment(subject.data, SegmentUpdateCase::AD);
  case 2:
    if (edge_AB == regions[closest_facet])
      return set_to_segment(subject.data, SegmentUpdateCase::AC);
    else
      return set_to_segment(subject.data, SegmentUpdateCase::AD);
  }
  throw Error{"Internal error updating plex"};
}
} // namespace

PlexUpdateResult update_plex(const Plex &subject
#ifdef GJK_EPA_DIAGNOSTIC
                             ,
                             nlohmann::json &log
#endif
) {
  struct Visitor {
#ifdef GJK_EPA_DIAGNOSTIC
    nlohmann::json &log;
#endif
    mutable PlexUpdateResult result;

    void operator()(const VertexCase &subject) const {
      result = update_segment(subject
#ifdef GJK_EPA_DIAGNOSTIC
                              ,
                              log
#endif
      );
    };

    void operator()(const SegmentCase &subject) const {
      result = update_facet(subject
#ifdef GJK_EPA_DIAGNOSTIC
                            ,
                            log
#endif
      );
    };

    void operator()(const FacetCase &subject) const {
      result = update_tethreadron(subject
#ifdef GJK_EPA_DIAGNOSTIC
                                  ,
                                  log
#endif
      );
    }
  }
#ifdef GJK_EPA_DIAGNOSTIC
  visitor {
    log
  }
#else
  visitor
#endif
  ;
  std::visit(visitor, subject);
  return visitor.result;
}

PlexDataPtr extract_data(const Plex &plex) {
  struct Visitor {
    mutable PlexDataPtr result;

    void operator()(const VertexCase &subject) const { result = subject.data; };

    void operator()(const SegmentCase &subject) const {
      result = subject.data;
    };

    void operator()(const FacetCase &subject) const { result = subject.data; };
  } visitor;
  std::visit(visitor, plex);
  return visitor.result;
}

} // namespace flx::gjk
