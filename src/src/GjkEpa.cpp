/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Epa.h"
#include "Plex.h"
#include <Flexible-GJK-and-EPA/GjkEpa.h>
#ifdef FLX_LOGGER_ENABLED
#include "Logger.h"
#endif

namespace flx {
void GjkEpa::getSupportMinkowskiDiff(const ShapePair &pair,
                                     const hull::Coordinate &direction,
                                     MinkowskiCoordinate &result) {
  pair.shapeA.getSupport(result.vertexA, direction);

  this->searchDirectionTwin.x = -direction.x;
  this->searchDirectionTwin.y = -direction.y;
  this->searchDirectionTwin.z = -direction.z;
  pair.shapeB.getSupport(result.vertexB, this->searchDirectionTwin);

  diff(result.vertexDiff, result.vertexA, result.vertexB);
}

GjkEpa::ClosestElement GjkEpa::getClosestInSegment(const hull::Coordinate &A,
                                                   const hull::Coordinate &B,
                                                   float *miximg_coeff) {
  miximg_coeff[2] = 0.f;
  hull::Coordinate B_A = B;
  B_A.x -= A.x;
  B_A.y -= A.y;
  B_A.z -= A.z;
  miximg_coeff[1] = -dot(B_A, A) / dot(B_A, B_A);
  if (miximg_coeff[1] <= 0.f) {
    miximg_coeff[0] = 1.f;
    miximg_coeff[1] = 0.f;
    return vertex_A;
  } else {
    miximg_coeff[0] = 1.f - miximg_coeff[1];
    return edge_AB;
  }
};

GjkEpa::ClosestElement GjkEpa::getClosestInTriangle(const hull::Coordinate &A,
                                                    const hull::Coordinate &B,
                                                    const hull::Coordinate &C,
                                                    float *miximg_coeff) {
  hull::Coordinate B_A = B;
  B_A.x -= A.x;
  B_A.y -= A.y;
  B_A.z -= A.z;
  hull::Coordinate C_A = C;
  C_A.x -= A.x;
  C_A.y -= A.y;
  C_A.z -= A.z;

  float m11 = dot(B_A, B_A);
  float m22 = dot(C_A, C_A);
  float m12 = dot(B_A, C_A);

  float c1 = -dot(A, B_A);
  float c2 = -dot(A, C_A);

  miximg_coeff[1] = (c1 * m22 - m12 * c2) / (m11 * m22 - m12 * m12);
  miximg_coeff[2] = (c2 - m12 * miximg_coeff[1]) / m22;

  bool temp[3];
  temp[0] = ((miximg_coeff[2] >= 0.f) && (miximg_coeff[2] <= 1.f));
  temp[1] = ((miximg_coeff[1] >= 0.f) && (miximg_coeff[1] <= 1.f));
  temp[2] = (miximg_coeff[1] + miximg_coeff[2] <= 1.f);

  if (temp[0] && temp[1] && temp[2]) {
    miximg_coeff[0] = 1.f - miximg_coeff[1] - miximg_coeff[2];
    return face_ABC;
  } else {
    auto closest_AB = getClosestInSegment(A, B, miximg_coeff);
    hull::Coordinate V_AB(A);
    prod(V_AB, miximg_coeff[0]);
    V_AB.x += miximg_coeff[1] * B.x;
    V_AB.y += miximg_coeff[1] * B.y;
    V_AB.z += miximg_coeff[1] * B.z;
    float d_AB = dot(V_AB, V_AB);

    float coeff_AC[3];
    auto closest_AC = getClosestInSegment(A, C, &coeff_AC[0]);
    hull::Coordinate V_AC(A);
    prod(V_AC, coeff_AC[0]);
    V_AC.x += coeff_AC[1] * C.x;
    V_AC.y += coeff_AC[1] * C.y;
    V_AC.z += coeff_AC[1] * C.z;
    float d_AC = dot(V_AC, V_AC);

    if (d_AB < d_AC)
      return closest_AB;
    else {
      miximg_coeff[0] = coeff_AC[0];
      miximg_coeff[1] = coeff_AC[1];
      if (closest_AC == edge_AB)
        closest_AC = edge_AC;
      return closest_AC;
    }
  }
}

bool GjkEpa::isCollisionPresent(const ShapePair &pair
#ifdef FLX_LOGGER_ENABLED
                                ,
                                std::string logFile
#endif
) {
#ifdef FLX_LOGGER_ENABLED
  std::shared_ptr<Logger> logger = std::make_shared<Logger>(logFile);
#endif
  Plex plex(*this, pair
#ifdef FLX_LOGGER_ENABLED
            ,
            logger
#endif
  );
  return plex.isCollisionPresent();
}

GjkEpa::ResultType GjkEpa::doComplexQuery(const ShapePair &pair,
                                          CoordinatePair &result
#ifdef FLX_LOGGER_ENABLED
                                          ,
                                          std::string logFile
#endif
) {
#ifdef FLX_LOGGER_ENABLED
  std::shared_ptr<Logger> logger = std::make_shared<Logger>(logFile);
#endif
  Plex plex(*this, pair
#ifdef FLX_LOGGER_ENABLED
            ,
            logger
#endif
  );
  if (plex.isCollisionPresent()) {
    // EPA
    Epa(plex, result
#ifdef FLX_LOGGER_ENABLED
        ,
        logger
#endif
    );
    return ResultType::penetrationVector;
  }
  // second phase of GJK
  plex.finishingLoop(result);
  return ResultType::closestPoints;
}

void GjkEpa::computeOutsideNormal(hull::Coordinate &N,
                                  const hull::Coordinate &P1,
                                  const hull::Coordinate &P2,
                                  const hull::Coordinate &P3,
                                  const hull::Coordinate &Pother) {
  hull::Coordinate Delta1, Delta2;
  diff(Delta1, P2, P1);
  diff(Delta2, P3, P1);
  cross(N, Delta1, Delta2);

  diff(Delta1, Pother, P1);
  if (dot(N, Delta1) > hull::GEOMETRIC_TOLLERANCE)
    invert(N);
  normalizeInPlace(N);
}
} // namespace flx
