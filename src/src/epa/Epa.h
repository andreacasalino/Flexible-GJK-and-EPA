/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "../gjk/Plex.h"
#include <Flexible-GJK-and-EPA/CoordinatePair.h>

namespace flx::epa {
CoordinatePair EPA(const ShapePair &pair, const gjk::Plex &initial_plex
#ifdef GJK_EPA_DIAGNOSTIC
                   ,
                   diagnostic::Diagnostic &log
#endif
);
} // namespace flx::epa
