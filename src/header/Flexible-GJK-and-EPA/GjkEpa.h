/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

// #include <Flexible-GJK-and-EPA/shape/ConvexShape.h>
// /** @brief When enabling this compiler definition, every time that a query is
//  * performed, a log file containing the iterations perormed by GjkEpa is
//  * created. This has just the aim of debugging the solver when needed.
//  */
// #ifdef FLX_LOGGER_ENABLED
// #include <string>
// #endif

// namespace flx {

// class GjkEpa {
// public:
//   GjkEpa(const shape::ConvexShape &shape_a, const shape::ConvexShape
//   &shape_b);

//   /** @brief Returns true if the passed set of shapes is in collision,
//   otherwise
//    * returns false. IMPORTANT!!!! this method is not thread safe: use
//    different
//    * GjkEpa solvers to implement multi-threading strategies
//    */
//   bool isCollisionPresent();

//   enum ResultType { closestPoints, penetrationVector };
//   struct QueryResult {
//     ResultType result_type;
//     hull::Coordinate point_in_shapeA;
//     hull::Coordinate point_in_shapeB;
//   };
//   /** @brief Perform a complex query on the passed pair of shapes.
//    *  The pair of coordinates returned as result has the following meaning:
//    *
//    * 		- if the shapes are not in collision are the closest
//    points.
//    *        The norm of the difference of these 2 vectors represent the
//    distance
//    * between the shapes.
//    *
//    * 		- if the shapes are in collision is the penetration
//    vector
//    *        The norm of the difference of these 2 vectors represent the
//    * penetration depth.
//    *
//    *  @return the meaning of result
//    *  @param the pair of shapes involved in the query
//    *  @param the result of the query
//    * IMPORTANT!!!! this method is not thread safe: use different GjkEpa
//    solvers
//    * to implement multi-threading strategies
//    */
//   QueryResult doComplexQuery();

// private:
//   const shape::ConvexShape &shape_a;
//   const shape::ConvexShape &shape_b;
// };
// } // namespace flx
