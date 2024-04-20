/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "Plex.h"
#include <Flexible-GJK-and-EPA/CoordinatePair.h>
#include <Flexible-GJK-and-EPA/Diagnostic.h>

namespace flx::gjk {
Plex initial_GJK_loop(const ShapePair &pair
#ifdef GJK_EPA_DIAGNOSTIC
                      ,
                      Observer *obsv = nullptr
#endif
);

CoordinatePair finishing_GJK_loop(const ShapePair &pair, Plex &&plex);
} // namespace flx::gjk
