/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Plex.h"
#include <float.h>

namespace flx {
GjkEpa::Plex::Plex(GjkEpa &user, const ShapePair &pair
#ifdef FLX_LOGGER_ENABLED
                   ,
                   std::shared_ptr<Logger> log
#endif
                   )
    : pair(pair), user(user)
#ifdef FLX_LOGGER_ENABLED
      ,
      logger(log)
#endif
{
#ifdef FLX_LOGGER_ENABLED
  this->logger->add("\n\"GjkPrimal\":");
  Array iterations;
#endif
  for (std::size_t k = 0; k < 4; ++k)
    this->vertices[k] = std::make_unique<MinkowskiCoordinate>();
  this->searchDirection = {1.f, 0.f, 0.f};
  this->user.getSupportMinkowskiDiff(this->pair, this->searchDirection,
                                     *this->vertices[1]);
  if (normSquared(this->vertices[1]->vertexDiff) <= GEOMETRIC_TOLLERANCE2) {
    this->collision_present = true;
#ifdef FLX_LOGGER_ENABLED
    iterations.add(this->print());
    this->logger->add(iterations.str());
#endif
    return;
  }
  this->searchDirection = this->vertices[1]->vertexDiff;
  invert(this->searchDirection);
  normalizeInPlace(this->searchDirection);
  while (true) {
    this->user.getSupportMinkowskiDiff(this->pair, this->searchDirection,
                                       *this->vertices[0]);
    if (dot(this->vertices[0]->vertexDiff, this->searchDirection) <=
        hull::GEOMETRIC_TOLLERANCE) {
#ifdef FLX_LOGGER_ENABLED
      iterations.add(this->print());
      this->logger->add(iterations.str());
#endif
      return;
    }
    ++this->plex_dim;
    switch (this->plex_dim) {
    case 4:
      this->update4();
      break;
    case 3:
      this->update3();
      break;
    default: // 2
      this->update2();
      break;
    }
    if (this->collision_present) {
      --this->plex_dim;
#ifdef FLX_LOGGER_ENABLED
      iterations.add(this->print());
      this->logger->add(iterations.str());
#endif
      return;
    }
#ifdef FLX_LOGGER_ENABLED
    iterations.add(this->print());
#endif
  }
}

static inline const uint8_t INCIDENCES[3][3] = {
    {0, 1, 2}, {0, 1, 3}, {0, 2, 3}};
void GjkEpa::Plex::update4() {
  // update visibility flags
  computeOutsideNormal(this->Normals[0], this->vertices[0]->vertexDiff,
                       this->vertices[1]->vertexDiff,
                       this->vertices[2]->vertexDiff,
                       this->vertices[3]->vertexDiff);
  if (dot(this->Normals[0], this->vertices[0]->vertexDiff) <=
      hull::GEOMETRIC_TOLLERANCE)
    this->Origin_is_visible[0] = true;
  else
    this->Origin_is_visible[0] = false;

  computeOutsideNormal(this->Normals[1], this->vertices[0]->vertexDiff,
                       this->vertices[1]->vertexDiff,
                       this->vertices[3]->vertexDiff,
                       this->vertices[2]->vertexDiff);
  if (dot(this->Normals[1], this->vertices[0]->vertexDiff) <=
      hull::GEOMETRIC_TOLLERANCE)
    this->Origin_is_visible[1] = true;
  else
    this->Origin_is_visible[1] = false;

  computeOutsideNormal(this->Normals[2], this->vertices[0]->vertexDiff,
                       this->vertices[2]->vertexDiff,
                       this->vertices[3]->vertexDiff,
                       this->vertices[1]->vertexDiff);
  if (dot(this->Normals[2], this->vertices[0]->vertexDiff) <=
      hull::GEOMETRIC_TOLLERANCE)
    this->Origin_is_visible[2] = true;
  else
    this->Origin_is_visible[2] = false;
  // check contains origin
  if (!(this->Origin_is_visible[0] && this->Origin_is_visible[1] &&
        this->Origin_is_visible[2])) {
    this->collision_present = true;
    return;
  }
  // update plex
  ClosestElement regions[3];
  float distances[3];
  hull::Coordinate temp;
  for (uint8_t k = 0; k < 3; ++k) {
    if (this->Origin_is_visible[k]) {
      regions[k] = getClosestInTriangle(
          this->vertices[INCIDENCES[k][0]]->vertexDiff,
          this->vertices[INCIDENCES[k][1]]->vertexDiff,
          this->vertices[INCIDENCES[k][2]]->vertexDiff, this->coeff);
      mix3(temp, this->vertices[INCIDENCES[k][0]]->vertexDiff,
           this->vertices[INCIDENCES[k][1]]->vertexDiff,
           this->vertices[INCIDENCES[k][2]]->vertexDiff, this->coeff);
      distances[k] = normSquared(temp);
    } else
      distances[k] = FLT_MAX;
  }
  uint8_t closest = 0;
  if (distances[1] < distances[closest])
    closest = 1;
  if (distances[2] < distances[closest])
    closest = 2;

  if (face_ABC == regions[closest])
    this->setToFacet(closest);
  else if (vertex_A == regions[closest])
    this->setToVertex();
  else {
    switch (closest) {
    case 0:
      if (edge_AB == regions[0])
        this->setToSegment(0);
      else
        this->setToSegment(1);
      break;
    case 1:
      if (edge_AB == regions[1])
        this->setToSegment(0);
      else
        this->setToSegment(2);
      break;
    default: // 2
      if (edge_AB == regions[2])
        this->setToSegment(1);
      else
        this->setToSegment(2);
      break;
    }
  }
}

void GjkEpa::Plex::update3() {
  auto closest = getClosestInTriangle(
      this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff,
      this->vertices[2]->vertexDiff, this->coeff);
  hull::Coordinate temp;
  mix3(temp, this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff,
       this->vertices[2]->vertexDiff, this->coeff);
  if (normSquared(temp) <= GEOMETRIC_TOLLERANCE2) {
    this->collision_present = true;
    return;
  }
  // update plex
  switch (closest) {
  case face_ABC:
    this->setToFacet(0);
    break;
  case vertex_A:
    this->setToVertex();
    break;
  default:
    if (edge_AB == closest)
      this->setToSegment(0);
    else
      this->setToSegment(1);
    break;
  }
}

void GjkEpa::Plex::update2() {
  hull::Coordinate temp;
  cross(temp, this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff);
  if (normSquared(temp) <= GEOMETRIC_TOLLERANCE4) {
    this->collision_present = true;
    return;
  }
  ClosestElement region =
      getClosestInSegment(this->vertices[0]->vertexDiff,
                          this->vertices[1]->vertexDiff, this->coeff);
  if (vertex_A == region)
    this->setToVertex();
  else
    this->setToSegment(0);
}

void GjkEpa::Plex::finishingLoop(CoordinatePair &closestPoints) {
#ifdef FLX_LOGGER_ENABLED
  this->logger->add(",\n\"GjkFinal\":");
  Array iterations;
#endif
  hull::Coordinate delta;
  diff(delta, this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff);
  if (dot(this->searchDirection, delta) > hull::GEOMETRIC_TOLLERANCE) {
    do {
      ++this->plex_dim;
      switch (this->plex_dim) {
      case 4:
        this->update4();
        break;
      case 3:
        this->update3();
        break;
      default: // 2
        this->update2();
        break;
      }
#ifdef FLX_LOGGER_ENABLED
      iterations.add(this->print());
#endif
      this->user.getSupportMinkowskiDiff(this->pair, this->searchDirection,
                                         *this->vertices[0]);
      diff(delta, this->vertices[0]->vertexDiff, this->vertices[1]->vertexDiff);
      if (dot(this->searchDirection, delta) <= hull::GEOMETRIC_TOLLERANCE) {
        break;
      }
    } while (true);
  }
#ifdef FLX_LOGGER_ENABLED
  this->logger->add(iterations.str());
#endif
  // compute closest pair
  if (1 == this->plex_dim) {
    closestPoints.pointA = this->vertices[0]->vertexA;
    closestPoints.pointB = this->vertices[0]->vertexB;
    return;
  }
  if (2 == this->plex_dim) {
    getClosestInSegment(this->vertices[1]->vertexDiff,
                        this->vertices[2]->vertexDiff, this->coeff);
    mix2(closestPoints.pointA, this->vertices[1]->vertexA,
         this->vertices[2]->vertexA, this->coeff);
    mix2(closestPoints.pointB, this->vertices[1]->vertexB,
         this->vertices[2]->vertexB, this->coeff);
    return;
  }
  getClosestInTriangle(this->vertices[1]->vertexDiff,
                       this->vertices[2]->vertexDiff,
                       this->vertices[3]->vertexDiff, this->coeff);
  mix3(closestPoints.pointA, this->vertices[1]->vertexA,
       this->vertices[2]->vertexA, this->vertices[3]->vertexA, this->coeff);
  mix3(closestPoints.pointB, this->vertices[1]->vertexB,
       this->vertices[2]->vertexB, this->vertices[3]->vertexB, this->coeff);
}

void GjkEpa::Plex::setToVertex() {
  this->searchDirection = this->vertices[0]->vertexDiff;
  invert(this->searchDirection);
  normalizeInPlace(this->searchDirection);
  this->plex_dim = 1;
  std::swap(this->vertices[0], this->vertices[1]);
}

void GjkEpa::Plex::setToSegment(const uint8_t &kind) {
  hull::Coordinate *A = &this->vertices[0]->vertexDiff;
  hull::Coordinate *B = nullptr;
  switch (kind) {
  case 0:
    B = &this->vertices[1]->vertexDiff;
    std::swap(this->vertices[2], this->vertices[1]);
    std::swap(this->vertices[1], this->vertices[0]);
    break;
  case 1:
    B = &this->vertices[2]->vertexDiff;
    std::swap(this->vertices[0], this->vertices[1]);
    break;
  case 2:
    B = &this->vertices[3]->vertexDiff;
    std::swap(this->vertices[0], this->vertices[1]);
    std::swap(this->vertices[2], this->vertices[3]);
    break;
  }
  hull::Coordinate B_A;
  diff(B_A, *B, *A);
  cross(this->searchDirection, *A, *B);
  this->searchDirection = cross(this->searchDirection, B_A);
  normalizeInPlace(this->searchDirection);
  this->plex_dim = 2;
}

void GjkEpa::Plex::setToFacet(const uint8_t &kind) {
  hull::Coordinate *A = &this->vertices[0]->vertexDiff;
  hull::Coordinate *B = nullptr;
  hull::Coordinate *C = nullptr;
  switch (kind) {
  case 0:
    B = &this->vertices[1]->vertexDiff;
    C = &this->vertices[2]->vertexDiff;
    std::swap(this->vertices[2], this->vertices[3]);
    std::swap(this->vertices[1], this->vertices[2]);
    std::swap(this->vertices[0], this->vertices[1]);
    break;
  case 1:
    B = &this->vertices[1]->vertexDiff;
    C = &this->vertices[3]->vertexDiff;
    std::swap(this->vertices[1], this->vertices[2]);
    std::swap(this->vertices[0], this->vertices[1]);
    break;
  case 2:
    B = &this->vertices[2]->vertexDiff;
    C = &this->vertices[3]->vertexDiff;
    std::swap(this->vertices[0], this->vertices[1]);
    break;
  }
  if (3 == this->plex_dim) {
    computeOutsideNormal(this->searchDirection, *A, *B, *C, hull::ORIGIN);
    invert(this->searchDirection);
  } else
    this->searchDirection = this->Normals[kind];
  this->plex_dim = 3;
}

#ifdef FLX_LOGGER_ENABLED
std::string GjkEpa::Plex::print() const {
  std::stringstream ss;
  ss << '{';
  ss << "\"direction\":";
  add(ss, this->searchDirection);
  ss << ",\"support\":";
  add(ss, this->vertices[0]->vertexDiff);
  ss << ",\"plex\":[";
  add(ss, this->vertices[1]->vertexDiff);
  for (uint8_t k = 1; k < this->plex_dim; ++k) {
    ss << ',';
    add(ss, this->vertices[k + 1]->vertexDiff);
  }
  ss << ']';
  ss << '}';
  return ss.str();
}
#endif
} // namespace flx
