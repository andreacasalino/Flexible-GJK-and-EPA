/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <list>
#include <memory>
#ifdef FLX_LOGGER_ENABLED
#include "Logger.h"
#endif

namespace flx {
constexpr float GEOMETRIC_TOLLERANCE2 =
    GEOMETRIC_TOLLERANCE * GEOMETRIC_TOLLERANCE;
constexpr float GEOMETRIC_TOLLERANCE4 =
    GEOMETRIC_TOLLERANCE2 * GEOMETRIC_TOLLERANCE2;

class GjkEpa::Plex {
public:
  // in the constructor the primal part is done
  Plex(GjkEpa &user, const ShapePair &pair
#ifdef FLX_LOGGER_ENABLED
       ,
       std::shared_ptr<Logger> log
#endif
  );

  // evolve plex till finding the closest entity to origin
  void finishingLoop(CoordinatePair &closestPoints);

  inline bool isCollisionPresent() { return this->collision_present; };

  typedef std::unique_ptr<MinkowskiCoordinate> MinkowskiCoordinatePtr;

  inline GjkEpa &getUser() const { return this->user; };
  inline const ShapePair &getPair() const { return this->pair; };
  inline const MinkowskiCoordinatePtr *getVertices() const {
    return &this->vertices[0];
  };
  inline const std::uint8_t &getPlexDimension() const {
    return this->plex_dim;
  };

private:
  void update4();
  void update3();
  void update2();

  void setToVertex();
  void setToSegment(const std::uint8_t &kind);
  void setToFacet(const std::uint8_t &kind);

  // data
  const ShapePair &pair;
  GjkEpa &user;
  MinkowskiCoordinatePtr vertices[4];
  std::uint8_t plex_dim = 1;
  bool collision_present = false;
  // cache
  Coordinate searchDirection;
  Coordinate Normals[3];     // ABC, ABE, ACE
  bool Origin_is_visible[3]; // ABC, ABE, ACE
  float coeff[3];

#ifdef FLX_LOGGER_ENABLED
  std::string print() const;
  std::shared_ptr<Logger> logger;
#endif
};
} // namespace flx
