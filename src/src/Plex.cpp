/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "GjkPlex.h"

namespace flx {
namespace {
VertexCase update_as_vertex(PlexData *data) {
  data->search_direction = data->vertices.front()->vertex_in_Minkowski_diff;
  hull::invert(data->search_direction);
  hull::normalizeInPlace(data->search_direction);
  std::swap(data->vertices[0], data->vertices[1]);
  return VertexCase{data};
}

enum SegmentUpdateCase { AB, AC, AD };
SegmentCase update_as_segment(PlexData *data,
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
FacetCase update_as_facet(PlexData *data, const FacetUpdateCase facet_case,
                          const hull::Coordinate *normals) {
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
    data->search_direction = normals[0];
    break;
  case FacetUpdateCase::ABD:
    B = &data->vertices[1]->vertex_in_Minkowski_diff;
    C = &data->vertices[3]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[1], data->vertices[2]);
    std::swap(data->vertices[0], data->vertices[1]);
    data->search_direction = normals[1];
    break;
  case FacetUpdateCase::ACD:
    B = &data->vertices[2]->vertex_in_Minkowski_diff;
    C = &data->vertices[3]->vertex_in_Minkowski_diff;
    std::swap(data->vertices[0], data->vertices[1]);
    data->search_direction = normals[2];
    break;
  }
  hull::invert(data->search_direction);
  return FacetCase{data};
}

bool compute_facet_visibility(hull::Coordinate &first, hull::Coordinate &second,
                              hull::Coordinate &third, hull::Coordinate &other,
                              hull::Coordinate &normal) {
  normal = computeOutsideNormal(first, second, third, other);
  if (hull::dot(normal, first) <= hull::GEOMETRIC_TOLLERANCE)
    return true;
  return false;
};
} // namespace

Plex update_plex(const Plex &subject) {
  struct Visitor {
    mutable Plex result;

    void operator()(const CollisionCase &) const { return; };

    void operator()(const VertexCase &subject) const {
      auto &data = *subject.data;
      hull::Coordinate temp;
      hull::cross(temp, data.vertices[0]->vertex_in_Minkowski_diff,
                  data.vertices[1]->vertex_in_Minkowski_diff);
      if (normSquared(temp) <= GEOMETRIC_TOLLERANCE4) {
        result = CollisionCase{};
        return;
      }
      auto closest =
          getClosestInSegment(data.vertices[0]->vertex_in_Minkowski_diff,
                              data.vertices[1]->vertex_in_Minkowski_diff);
      if (vertex_A == closest.region) {
        result = update_as_vertex(subject.data);
        return;
      }
      result = update_as_segment(subject.data, SegmentUpdateCase::AB);
    };

    void operator()(const SegmentCase &subject) const {
      auto &data = *subject.data;
      auto closest =
          getClosestInTriangle(data.vertices[0]->vertex_in_Minkowski_diff,
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
      case ClosestElement::face_ABC: {
        hull::Coordinate normal = computeOutsideNormal(
            data.vertices[0]->vertex_in_Minkowski_diff,
            data.vertices[1]->vertex_in_Minkowski_diff,
            data.vertices[2]->vertex_in_Minkowski_diff, hull::ORIGIN);
        result = update_as_facet(subject.data, FacetUpdateCase::ABC, &normal);
      } break;
      case ClosestElement::vertex_A:
        result = update_as_vertex(subject.data);
        break;
      default:
        if (ClosestElement::edge_AB == closest.region)
          result = update_as_segment(subject.data, SegmentUpdateCase::AB);
        else
          result = update_as_segment(subject.data, SegmentUpdateCase::AC);
        break;
      }
    };

    void operator()(const FacetCase &) const {
      std::array<hull::Coordinate, 3> normals;
      // update visibility flags
      bool visibility_ABC = compute_facet_visibility(
          this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff,
          this->vertices[2]->vertexDiff, this->vertices[3]->vertexDiff,
          normals[0]);
      bool visibility_ABD = compute_facet_visibility(
          this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff,
          this->vertices[3]->vertexDiff, this->vertices[2]->vertexDiff,
          normals[1]);
      bool visibility_ACD = compute_facet_visibility(
          this->vertices[0]->vertexDiff, this->vertices[2]->vertexDiff,
          this->vertices[3]->vertexDiff, this->vertices[1]->vertexDiff,
          normals[2]);

      // check contains origin
      if (!(visibility_ABC && visibility_ABD && visibility_ACD)) {
        result = CollisionCase{};
        return;
      }

      //   // update plex
      //   ClosestElement regions[3];
      //   float distances[3];
      //   hull::Coordinate temp;
      //   for (uint8_t k = 0; k < 3; ++k) {
      //     if (this->Origin_is_visible[k]) {
      //       regions[k] = getClosestInTriangle(
      //           this->vertices[INCIDENCES[k][0]]->vertexDiff,
      //           this->vertices[INCIDENCES[k][1]]->vertexDiff,
      //           this->vertices[INCIDENCES[k][2]]->vertexDiff, this->coeff);
      //       mix3(temp, this->vertices[INCIDENCES[k][0]]->vertexDiff,
      //            this->vertices[INCIDENCES[k][1]]->vertexDiff,
      //            this->vertices[INCIDENCES[k][2]]->vertexDiff, this->coeff);
      //       distances[k] = normSquared(temp);
      //     } else
      //       distances[k] = FLT_MAX;
      //   }

      //   uint8_t closest = 0;
      //   if (distances[1] < distances[closest])
      //     closest = 1;
      //   if (distances[2] < distances[closest])
      //     closest = 2;

      //   if (face_ABC == regions[closest])
      //     this->setToFacet(closest);
      //   else if (vertex_A == regions[closest])
      //     this->setToVertex();
      //   else {
      //     switch (closest) {
      //     case 0:
      //       if (edge_AB == regions[0])
      //         this->setToSegment(0);
      //       else
      //         this->setToSegment(1);
      //       break;
      //     case 1:
      //       if (edge_AB == regions[1])
      //         this->setToSegment(0);
      //       else
      //         this->setToSegment(2);
      //       break;
      //     default: // 2
      //       if (edge_AB == regions[2])
      //         this->setToSegment(1);
      //       else
      //         this->setToSegment(2);
      //       break;
      //     }
      //   }
    };
  };

  Visitor visitor;
  std::visit(visitor, subject);
  return visitor.result;
}

} // namespace flx

// // GjkEpa::Plex::Plex(GjkEpa &user, const ShapePair &pair
// // #ifdef FLX_LOGGER_ENABLED
// //                    ,
// //                    std::shared_ptr<Logger> log
// // #endif
// //                    )
// //     : pair(pair), user(user)
// // #ifdef FLX_LOGGER_ENABLED
// //       ,
// //       logger(log)
// // #endif
// // {
// // #ifdef FLX_LOGGER_ENABLED
// //   this->logger->add("\n\"GjkPrimal\":");
// //   Array iterations;
// // #endif
// //   for (std::size_t k = 0; k < 4; ++k)
// //     this->vertices[k] = std::make_unique<MinkowskiCoordinate>();
// //   this->searchDirection = {1.f, 0.f, 0.f};
// //   this->user.getSupportMinkowskiDiff(this->pair, this->searchDirection,
// //                                      *this->vertices[1]);
// //   if (normSquared(this->vertices[1]->vertexDiff) <= GEOMETRIC_TOLLERANCE2)
// {
// //     this->collision_present = true;
// // #ifdef FLX_LOGGER_ENABLED
// //     iterations.add(this->print());
// //     this->logger->add(iterations.str());
// // #endif
// //     return;
// //   }
// //   this->searchDirection = this->vertices[1]->vertexDiff;
// //   invert(this->searchDirection);
// //   normalizeInPlace(this->searchDirection);
// //   while (true) {
// //     this->user.getSupportMinkowskiDiff(this->pair, this->searchDirection,
// //                                        *this->vertices[0]);
// //     if (dot(this->vertices[0]->vertexDiff, this->searchDirection) <=
// //         hull::GEOMETRIC_TOLLERANCE) {
// // #ifdef FLX_LOGGER_ENABLED
// //       iterations.add(this->print());
// //       this->logger->add(iterations.str());
// // #endif
// //       return;
// //     }
// //     ++this->plex_dim;
// //     switch (this->plex_dim) {
// //     case 4:
// //       this->update4();
// //       break;
// //     case 3:
// //       this->update3();
// //       break;
// //     default: // 2
// //       this->update2();
// //       break;
// //     }
// //     if (this->collision_present) {
// //       --this->plex_dim;
// // #ifdef FLX_LOGGER_ENABLED
// //       iterations.add(this->print());
// //       this->logger->add(iterations.str());
// // #endif
// //       return;
// //     }
// // #ifdef FLX_LOGGER_ENABLED
// //     iterations.add(this->print());
// // #endif
// //   }
// // }

// // static inline const uint8_t INCIDENCES[3][3] = {
// //     {0, 1, 2}, {0, 1, 3}, {0, 2, 3}};

// // void GjkEpa::Plex::finishingLoop(CoordinatePair &closestPoints) {
// // #ifdef FLX_LOGGER_ENABLED
// //   this->logger->add(",\n\"GjkFinal\":");
// //   Array iterations;
// // #endif
// //   hull::Coordinate delta;
// //   diff(delta, this->vertices[0]->vertexDiff,
// this->vertices[1]->vertexDiff);
// //   if (dot(this->searchDirection, delta) > hull::GEOMETRIC_TOLLERANCE) {
// //     do {
// //       ++this->plex_dim;
// //       switch (this->plex_dim) {
// //       case 4:
// //         this->update4();
// //         break;
// //       case 3:
// //         this->update3();
// //         break;
// //       default: // 2
// //         this->update2();
// //         break;
// //       }
// // #ifdef FLX_LOGGER_ENABLED
// //       iterations.add(this->print());
// // #endif
// //       this->user.getSupportMinkowskiDiff(this->pair,
// this->searchDirection,
// //                                          *this->vertices[0]);
// //       diff(delta, this->vertices[0]->vertexDiff,
// //       this->vertices[1]->vertexDiff); if (dot(this->searchDirection,
// delta)
// //       <= hull::GEOMETRIC_TOLLERANCE) {
// //         break;
// //       }
// //     } while (true);
// //   }
// // #ifdef FLX_LOGGER_ENABLED
// //   this->logger->add(iterations.str());
// // #endif
// //   // compute closest pair
// //   if (1 == this->plex_dim) {
// //     closestPoints.pointA = this->vertices[0]->vertexA;
// //     closestPoints.pointB = this->vertices[0]->vertexB;
// //     return;
// //   }
// //   if (2 == this->plex_dim) {
// //     getClosestInSegment(this->vertices[1]->vertexDiff,
// //                         this->vertices[2]->vertexDiff, this->coeff);
// //     mix2(closestPoints.pointA, this->vertices[1]->vertexA,
// //          this->vertices[2]->vertexA, this->coeff);
// //     mix2(closestPoints.pointB, this->vertices[1]->vertexB,
// //          this->vertices[2]->vertexB, this->coeff);
// //     return;
// //   }
// //   getClosestInTriangle(this->vertices[1]->vertexDiff,
// //                        this->vertices[2]->vertexDiff,
// //                        this->vertices[3]->vertexDiff, this->coeff);
// //   mix3(closestPoints.pointA, this->vertices[1]->vertexA,
// //        this->vertices[2]->vertexA, this->vertices[3]->vertexA,
// this->coeff);
// //   mix3(closestPoints.pointB, this->vertices[1]->vertexB,
// //        this->vertices[2]->vertexB, this->vertices[3]->vertexB,
// this->coeff);
// // }
