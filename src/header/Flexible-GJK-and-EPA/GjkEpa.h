/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>
/** @brief When enabling this compiler definition, every time that a query is
 * performed, a log file containing the iterations perormed by GjkEpa is
 * created. This has just the aim of debugging the solver when needed.
 */
#ifdef FLX_LOGGER_ENABLED
#include <string>
#endif

namespace flx {
struct CoordinatePair {
  hull::Coordinate pointA;
  hull::Coordinate pointB;
};

class GjkEpa {
public:
  GjkEpa() = default;

  struct ShapePair {
    const shape::ConvexShape &shapeA;
    const shape::ConvexShape &shapeB;
  };

  /** @brief Returns true if the passed set of shapes is in collision, otherwise
   * returns false. IMPORTANT!!!! this method is not thread safe: use different
   * GjkEpa solvers to implement multi-threading strategies
   */
  bool isCollisionPresent(const ShapePair &pair
#ifdef FLX_LOGGER_ENABLED
                          ,
                          std::string logFile = ""
#endif
  );

  enum ResultType { closestPoints, penetrationVector };
  /** @brief Perform a complex query on the passed pair of shapes.
   *  The pair of coordinates returned as result has the following meaning:
   *
   * 		- if the shapes are not in collision are the closest points.
   *        The norm of the difference of these 2 vectors represent the distance
   * between the shapes.
   *
   * 		- if the shapes are in collision is the penetration vector
   *        The norm of the difference of these 2 vectors represent the
   * penetration depth.
   *
   *  @return the meaning of result
   *  @param the pair of shapes involved in the query
   *  @param the result of the query
   * IMPORTANT!!!! this method is not thread safe: use different GjkEpa solvers
   * to implement multi-threading strategies
   */
  ResultType doComplexQuery(const ShapePair &pair, CoordinatePair &result
#ifdef FLX_LOGGER_ENABLED
                            ,
                            std::string logFile = ""
#endif
  );

private:
  // cache to speed up computations
  hull::Coordinate searchDirectionTwin;
};
} // namespace flx
