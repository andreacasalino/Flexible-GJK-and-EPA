/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "GjkPlex.h"

namespace flx {
// bool GjkEpa::isCollisionPresent(const ShapePair &pair
// #ifdef FLX_LOGGER_ENABLED
//                                 ,
//                                 std::string logFile
// #endif
// ) {
// #ifdef FLX_LOGGER_ENABLED
//   std::shared_ptr<Logger> logger = std::make_shared<Logger>(logFile);
// #endif
//   Plex plex(*this, pair
// #ifdef FLX_LOGGER_ENABLED
//             ,
//             logger
// #endif
//   );
//   return plex.isCollisionPresent();
// }

// GjkEpa::ResultType GjkEpa::doComplexQuery(const ShapePair &pair,
//                                           CoordinatePair &result
// #ifdef FLX_LOGGER_ENABLED
//                                           ,
//                                           std::string logFile
// #endif
// ) {
// #ifdef FLX_LOGGER_ENABLED
//   std::shared_ptr<Logger> logger = std::make_shared<Logger>(logFile);
// #endif
//   Plex plex(*this, pair
// #ifdef FLX_LOGGER_ENABLED
//             ,
//             logger
// #endif
//   );
//   if (plex.isCollisionPresent()) {
//     // EPA
//     Epa(plex, result
// #ifdef FLX_LOGGER_ENABLED
//         ,
//         logger
// #endif
//     );
//     return ResultType::penetrationVector;
//   }
//   // second phase of GJK
//   plex.finishingLoop(result);
//   return ResultType::closestPoints;
// }

} // namespace flx
