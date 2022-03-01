/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Plex.h"

#include <limits>

namespace flx {
VertexCase set_to_vertex(const PlexDataPtr &data) {
  data->search_direction = data->vertices.front()->vertex_in_Minkowski_diff;
  hull::invert(data->search_direction);
  hull::normalizeInPlace(data->search_direction);
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
  hull::Coordinate B_A;
  diff(B_A, *B, *A);
  hull::cross(data->search_direction, *A, *B);
  data->search_direction = cross(data->search_direction, B_A);
  hull::normalizeInPlace(data->search_direction);
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
    data->search_direction = outward_normals[0];
    break;
  case FacetUpdateCase::ABD:
    B = &data->vertices[1]->vertex_in_Minkowski_diff;
    C = &data->vertices[3]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[1], data->vertices[2]);
    std::swap(data->vertices[0], data->vertices[1]);
    data->search_direction = outward_normals[1];
    break;
  case FacetUpdateCase::ACD:
    B = &data->vertices[2]->vertex_in_Minkowski_diff;
    C = &data->vertices[3]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[0], data->vertices[1]);
    data->search_direction = outward_normals[2];
    break;
  }
  return FacetCase{data};
}

bool compute_facet_visibility(hull::Coordinate &first, hull::Coordinate &second,
                              hull::Coordinate &third, hull::Coordinate &other,
                              hull::Coordinate &normal) {
  normal = computeOutsideNormal(first, second, third, other);
  return hull::dot(normal, first) <= hull::HULL_GEOMETRIC_TOLLERANCE;
};

constexpr float MAX_DISTANCE = std::numeric_limits<float>::max();
} // namespace

namespace {
static inline const uint8_t INCIDENCES[3][3] = {
    {0, 1, 2}, {0, 1, 3}, {0, 2, 3}};
} // namespace
PlexUpdateResult update_plex(const Plex &subject) {
  struct Visitor {
    mutable PlexUpdateResult result;

    void operator()(const VertexCase &subject) const {
      auto &data = *subject.data;
      hull::Coordinate temp;
      hull::cross(temp, data.vertices[0]->vertex_in_Minkowski_diff,
                  data.vertices[1]->vertex_in_Minkowski_diff);
      if (normSquared(temp) <= GEOMETRIC_TOLLERANCE4) {
        result = CollisionCase{};
        return;
      }
      auto closest = getClosestToOriginInSegment(
          data.vertices[0]->vertex_in_Minkowski_diff,
          data.vertices[1]->vertex_in_Minkowski_diff);
      if (vertex_A == closest.region) {
        result = set_to_vertex(subject.data);
        return;
      }
      result = set_to_segment(subject.data, SegmentUpdateCase::AB);
    };

    void operator()(const SegmentCase &subject) const {
      auto &data = *subject.data;
      auto closest = getClosestToOriginInTriangle(
          data.vertices[0]->vertex_in_Minkowski_diff,
          data.vertices[1]->vertex_in_Minkowski_diff,
          data.vertices[2]->vertex_in_Minkowski_diff);
      hull::Coordinate temp = mix3(data.vertices[0]->vertex_in_Minkowski_diff,
                                   data.vertices[1]->vertex_in_Minkowski_diff,
                                   data.vertices[2]->vertex_in_Minkowski_diff,
                                   closest.coefficients);
      if (normSquared(temp) <= GEOMETRIC_TOLLERANCE2) {
        result = CollisionCase{};
        return;
      }
      // update plex
      switch (closest.region) {
      case ClosestRegionToOrigin::face_ABC: {
        hull::Coordinate normal = computeOutsideNormal(
            data.vertices[0]->vertex_in_Minkowski_diff,
            data.vertices[1]->vertex_in_Minkowski_diff,
            data.vertices[2]->vertex_in_Minkowski_diff, hull::ORIGIN);
        hull::invert(normal);
        result = set_to_facet(subject.data, FacetUpdateCase::ABC, &normal);
      } break;
      case ClosestRegionToOrigin::vertex_A:
        result = set_to_vertex(subject.data);
        break;
      case ClosestRegionToOrigin::edge_AB:
        result = set_to_segment(subject.data, SegmentUpdateCase::AB);
        break;
      case ClosestRegionToOrigin::edge_AC:
        result = set_to_segment(subject.data, SegmentUpdateCase::AC);
        break;
      }
    };

    void operator()(const FacetCase &subject) const {
      auto &data = *subject.data;
      std::array<hull::Coordinate, 3> normals;
      bool is_origin_visible[3];
      // update visibility flags
      is_origin_visible[0] = compute_facet_visibility(
          data.vertices[0]->vertex_in_Minkowski_diff,
          data.vertices[1]->vertex_in_Minkowski_diff,
          data.vertices[2]->vertex_in_Minkowski_diff,
          data.vertices[3]->vertex_in_Minkowski_diff, normals[0]);
      is_origin_visible[1] = compute_facet_visibility(
          data.vertices[0]->vertex_in_Minkowski_diff,
          data.vertices[1]->vertex_in_Minkowski_diff,
          data.vertices[3]->vertex_in_Minkowski_diff,
          data.vertices[2]->vertex_in_Minkowski_diff, normals[1]);
      is_origin_visible[2] = compute_facet_visibility(
          data.vertices[0]->vertex_in_Minkowski_diff,
          data.vertices[2]->vertex_in_Minkowski_diff,
          data.vertices[3]->vertex_in_Minkowski_diff,
          data.vertices[1]->vertex_in_Minkowski_diff, normals[2]);

      // check contains origin
      if (!(is_origin_visible[0] || is_origin_visible[1] ||
            is_origin_visible[2])) {
        result = CollisionCase{};
        return;
      }

      // update plex
      ClosestRegionToOrigin regions[3];
      float distances[3];
      hull::Coordinate temp;
      for (std::uint8_t k = 0; k < 3; ++k) {
        if (is_origin_visible[k]) {
          auto [region, coeff] = getClosestToOriginInTriangle(
              data.vertices[INCIDENCES[k][0]]->vertex_in_Minkowski_diff,
              data.vertices[INCIDENCES[k][1]]->vertex_in_Minkowski_diff,
              data.vertices[INCIDENCES[k][2]]->vertex_in_Minkowski_diff);
          distances[k] = normSquared(
              mix3(data.vertices[INCIDENCES[k][0]]->vertex_in_Minkowski_diff,
                   data.vertices[INCIDENCES[k][1]]->vertex_in_Minkowski_diff,
                   data.vertices[INCIDENCES[k][2]]->vertex_in_Minkowski_diff,
                   coeff));
          regions[k] = region;
        } else
          distances[k] = MAX_DISTANCE;
      }

      std::uint8_t closest_facet = 0;
      if (distances[1] < distances[closest_facet])
        closest_facet = 1;
      if (distances[2] < distances[closest_facet])
        closest_facet = 2;

      if (regions[closest_facet] == vertex_A) {
        result = set_to_vertex(subject.data);
        return;
      }

      if (regions[closest_facet] == face_ABC) {
        switch (closest_facet) {
        case 0:
          result =
              set_to_facet(subject.data, FacetUpdateCase::ABC, normals.data());
          break;
        case 1:
          result =
              set_to_facet(subject.data, FacetUpdateCase::ABD, normals.data());
          break;
        case 2:
          result =
              set_to_facet(subject.data, FacetUpdateCase::ACD, normals.data());
          break;
        }
        return;
      }

      switch (closest_facet) {
      case 0:
        if (edge_AB == regions[closest_facet])
          result = set_to_segment(subject.data, SegmentUpdateCase::AB);
        else
          result = set_to_segment(subject.data, SegmentUpdateCase::AC);
        break;
      case 1:
        if (edge_AB == regions[closest_facet])
          result = set_to_segment(subject.data, SegmentUpdateCase::AB);
        else
          result = set_to_segment(subject.data, SegmentUpdateCase::AD);
        break;
      case 2:
        if (edge_AB == regions[closest_facet])
          result = set_to_segment(subject.data, SegmentUpdateCase::AC);
        else
          result = set_to_segment(subject.data, SegmentUpdateCase::AD);
        break;
      }
    };
  } visitor;
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

} // namespace flx
