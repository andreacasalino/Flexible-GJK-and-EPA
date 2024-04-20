/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Plex.h"
#include <Flexible-GJK-and-EPA/Diagnostic.h>
#include <Flexible-GJK-and-EPA/Error.h>

#include <limits>

namespace flx::gjk {
void udpateDirection(Plex &subject, const hull::Coordinate &new_direction) {
  subject.search_direction = new_direction;
  hull::normalizeInPlace(subject.search_direction);
}

void set_to_vertex(Plex &subject) {
  hull::Coordinate direction =
      subject.vertices.front()->vertex_in_Minkowski_diff;
  hull::invert(direction);
  udpateDirection(subject, direction);
  std::swap(subject.vertices[0], subject.vertices[1]);
  subject.size = 1;
}

namespace {
enum class SegmentUpdateCase { AB, AC, AD };
void set_to_segment(Plex &subject, SegmentUpdateCase segment_case) {
  hull::Coordinate *A = &subject.vertices[0]->vertex_in_Minkowski_diff;
  hull::Coordinate *B = nullptr;
  switch (segment_case) {
  case SegmentUpdateCase::AB:
    B = &subject.vertices[1]->vertex_in_Minkowski_diff;
    std::swap(subject.vertices[2], subject.vertices[1]);
    std::swap(subject.vertices[1], subject.vertices[0]);
    break;
  case SegmentUpdateCase::AC:
    B = &subject.vertices[2]->vertex_in_Minkowski_diff;
    std::swap(subject.vertices[0], subject.vertices[1]);
    break;
  case SegmentUpdateCase::AD:
    B = &subject.vertices[3]->vertex_in_Minkowski_diff;
    std::swap(subject.vertices[0], subject.vertices[1]);
    std::swap(subject.vertices[2], subject.vertices[3]);
    break;
  }
  hull::Coordinate search_direction = hull::cross(*A, *B);
  search_direction = cross(search_direction, delta(*B, *A));
  udpateDirection(subject, search_direction);
  subject.size = 2;
}

enum class FacetUpdateCase { ABC, ABD, ACD };
void set_to_facet(Plex &subject, FacetUpdateCase facet_case,
                  const hull::Coordinate *outward_normals) {
  hull::Coordinate *A = &subject.vertices[0]->vertex_in_Minkowski_diff;
  hull::Coordinate *B = nullptr;
  hull::Coordinate *C = nullptr;
  switch (facet_case) {
  case FacetUpdateCase::ABC:
    B = &subject.vertices[1]->vertex_in_Minkowski_diff;
    C = &subject.vertices[2]->vertex_in_Minkowski_diff;
    std::swap(subject.vertices[2], subject.vertices[3]);
    std::swap(subject.vertices[1], subject.vertices[2]);
    std::swap(subject.vertices[0], subject.vertices[1]);
    subject.search_direction = outward_normals[0];
    break;
  case FacetUpdateCase::ABD:
    B = &subject.vertices[1]->vertex_in_Minkowski_diff;
    C = &subject.vertices[3]->vertex_in_Minkowski_diff;
    std::swap(subject.vertices[1], subject.vertices[2]);
    std::swap(subject.vertices[0], subject.vertices[1]);
    subject.search_direction = outward_normals[1];
    break;
  case FacetUpdateCase::ACD:
    B = &subject.vertices[2]->vertex_in_Minkowski_diff;
    C = &subject.vertices[3]->vertex_in_Minkowski_diff;
    std::swap(subject.vertices[0], subject.vertices[1]);
    subject.search_direction = outward_normals[2];
    break;
  }
  subject.size = 3;
}

bool is_facet_not_looking_at_origin(const hull::Coordinate &first,
                                    const hull::Coordinate &second,
                                    const hull::Coordinate &third,
                                    const hull::Coordinate &other,
                                    hull::Coordinate &normal) {
  normal = computeOutsideNormal(first, second, third, other);
  return is_lower(hull::dot(normal, first), hull::HULL_GEOMETRIC_TOLLERANCE);
};

constexpr float MAX_DISTANCE = std::numeric_limits<float>::max();
} // namespace

namespace {
static inline const std::size_t INCIDENCES[3][3] = {
    {0, 1, 2}, {0, 1, 3}, {0, 2, 3}};

#ifdef GJK_EPA_DIAGNOSTIC
struct NotificationGuard {
  NotificationGuard(const Plex &pl) : info{pl} {}
  ~NotificationGuard() {
    if (info.plex.obsv) {
      info.plex.obsv->onUpdate(info);
    }
  }

  GjkIteration info;
};
#endif

void update_segment(Plex &segment) {
#ifdef GJK_EPA_DIAGNOSTIC
  std::optional<NotificationGuard> notification{segment};
#endif
  hull::Coordinate temp =
      hull::cross(segment.vertices[0]->vertex_in_Minkowski_diff,
                  segment.vertices[1]->vertex_in_Minkowski_diff);
  if (normSquared(temp) <= GEOMETRIC_TOLLERANCE_SQUARED_SQUARED) {
    segment.collision = true;
    return;
  }
  auto closest = getClosestToOriginInSegment(
      segment.vertices[0]->vertex_in_Minkowski_diff,
      segment.vertices[1]->vertex_in_Minkowski_diff);
#ifdef GJK_EPA_DIAGNOSTIC
  GjkIteration::ClosestToRegionInfo info_closest;
  info_closest.region = closest.region;
  info_closest.point =
      mix2(segment.vertices[0]->vertex_in_Minkowski_diff,
           segment.vertices[1]->vertex_in_Minkowski_diff, closest.coefficients);
  notification->info.info = info_closest;
  notification.reset();
#endif
  if (ClosestRegionToOrigin::vertex_A == closest.region) {
    set_to_vertex(segment);
  } else {
    set_to_segment(segment, SegmentUpdateCase::AB);
  }
}

void update_facet(Plex &facet) {
#ifdef GJK_EPA_DIAGNOSTIC
  std::optional<NotificationGuard> notification{facet};
#endif
  auto closest =
      getClosestToOriginInTriangle(facet.vertices[0]->vertex_in_Minkowski_diff,
                                   facet.vertices[1]->vertex_in_Minkowski_diff,
                                   facet.vertices[2]->vertex_in_Minkowski_diff);
  hull::Coordinate temp =
      mix3(facet.vertices[0]->vertex_in_Minkowski_diff,
           facet.vertices[1]->vertex_in_Minkowski_diff,
           facet.vertices[2]->vertex_in_Minkowski_diff, closest.coefficients);
  if (normSquared(temp) <= GEOMETRIC_TOLLERANCE_SQUARED) {
    facet.collision = true;
    return;
  }
#ifdef GJK_EPA_DIAGNOSTIC
  GjkIteration::ClosestToRegionInfo info_closest;
  info_closest.region = closest.region;
  info_closest.point = temp;
  notification->info.info = info_closest;
  notification.reset();
#endif
  switch (closest.region) {
  case ClosestRegionToOrigin::face_ABC: {
    hull::Coordinate normal = computeOutsideNormal(
        facet.vertices[0]->vertex_in_Minkowski_diff,
        facet.vertices[1]->vertex_in_Minkowski_diff,
        facet.vertices[2]->vertex_in_Minkowski_diff, hull::ORIGIN);
    hull::invert(normal);
    set_to_facet(facet, FacetUpdateCase::ABC, &normal);
    return;
  }
  case ClosestRegionToOrigin::vertex_A:
    set_to_vertex(facet);
    return;
  case ClosestRegionToOrigin::edge_AB:
    set_to_segment(facet, SegmentUpdateCase::AB);
    return;
  case ClosestRegionToOrigin::edge_AC:
    set_to_segment(facet, SegmentUpdateCase::AC);
    return;
  }
  throw Error{"Internal error updating plex"};
}

void update_tethreadron(Plex &tethreadron) {
#ifdef GJK_EPA_DIAGNOSTIC
  std::optional<NotificationGuard> notification{tethreadron};
#endif
  std::array<hull::Coordinate, 3> normals;
  std::array<bool, 3> is_origin_visible;

  auto visibility_flag = [&](std::size_t Va, std::size_t Vb, std::size_t Vc,
                             std::size_t index) {
    is_origin_visible[index] = is_facet_not_looking_at_origin(
        tethreadron.vertices[0]->vertex_in_Minkowski_diff,
        tethreadron.vertices[Va]->vertex_in_Minkowski_diff,
        tethreadron.vertices[Vb]->vertex_in_Minkowski_diff,
        tethreadron.vertices[Vc]->vertex_in_Minkowski_diff, normals[index]);
    return is_origin_visible[index];
  };

#ifdef GJK_EPA_DIAGNOSTIC
  {
    std::array<GjkIteration::TethraedronFacetInfo, 3> info;
    for (std::size_t k = 0; k < 3; ++k) {
      info[k].isOriginVisible = is_origin_visible[k];
      info[k].normal = normals[k];
    }
    notification->info.info = std::move(info);
  }
#endif

  // check contains origin
  if (!(visibility_flag(1, 2, 3, 0) || visibility_flag(1, 3, 2, 1) ||
        visibility_flag(2, 3, 1, 2))) {
    tethreadron.collision = true;
    return;
  }

  // update plex
  ClosestRegionToOrigin regions[3];
  float distances[3];
  for (std::size_t k = 0; k < 3; ++k) {
    if (is_origin_visible[k]) {
      auto [region, coeff] = getClosestToOriginInTriangle(
          tethreadron.vertices[INCIDENCES[k][0]]->vertex_in_Minkowski_diff,
          tethreadron.vertices[INCIDENCES[k][1]]->vertex_in_Minkowski_diff,
          tethreadron.vertices[INCIDENCES[k][2]]->vertex_in_Minkowski_diff);
      hull::Coordinate temp =
          mix3(tethreadron.vertices[INCIDENCES[k][0]]->vertex_in_Minkowski_diff,
               tethreadron.vertices[INCIDENCES[k][1]]->vertex_in_Minkowski_diff,
               tethreadron.vertices[INCIDENCES[k][2]]->vertex_in_Minkowski_diff,
               coeff);
      distances[k] = normSquared(temp);
      regions[k] = region;
#ifdef GJK_EPA_DIAGNOSTIC
      auto *info =
          std::get_if<std::array<GjkIteration::TethraedronFacetInfo, 3>>(
              &notification->info.info);
      (*info)[k].closest = GjkIteration::ClosestToRegionInfo{region, temp};
#endif
    } else {
      distances[k] = MAX_DISTANCE;
    }
  }

#ifdef GJK_EPA_DIAGNOSTIC
  notification.reset();
#endif

  std::size_t closest_facet = 0;
  if (distances[1] < distances[closest_facet])
    closest_facet = 1;
  if (distances[2] < distances[closest_facet])
    closest_facet = 2;

  if (regions[closest_facet] == ClosestRegionToOrigin::vertex_A) {
    set_to_vertex(tethreadron);
    return;
  }

  if (regions[closest_facet] == ClosestRegionToOrigin::face_ABC) {
    switch (closest_facet) {
    case 0:
      set_to_facet(tethreadron, FacetUpdateCase::ABC, normals.data());
      break;
    case 1:
      set_to_facet(tethreadron, FacetUpdateCase::ABD, normals.data());
      break;
    case 2:
      set_to_facet(tethreadron, FacetUpdateCase::ACD, normals.data());
      break;
    }
    return;
  }

  switch (closest_facet) {
  case 0:
    if (ClosestRegionToOrigin::edge_AB == regions[closest_facet])
      set_to_segment(tethreadron, SegmentUpdateCase::AB);
    else
      set_to_segment(tethreadron, SegmentUpdateCase::AC);
    return;
  case 1:
    if (ClosestRegionToOrigin::edge_AB == regions[closest_facet])
      set_to_segment(tethreadron, SegmentUpdateCase::AB);
    else
      set_to_segment(tethreadron, SegmentUpdateCase::AD);
    return;
  case 2:
    if (ClosestRegionToOrigin::edge_AB == regions[closest_facet])
      set_to_segment(tethreadron, SegmentUpdateCase::AC);
    else
      set_to_segment(tethreadron, SegmentUpdateCase::AD);
    return;
  }
  throw Error{"Internal error updating plex"};
}
} // namespace

void updatePlex(Plex &subject) {
  switch (subject.size) {
  case 1:
    update_segment(subject);
    break;
  case 2:
    update_facet(subject);
    break;
  case 3:
    update_tethreadron(subject);
    break;
  }
}

} // namespace flx::gjk
