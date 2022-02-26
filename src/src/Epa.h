/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "Plex.h"
#include <Flexible-GJK-and-EPA/CoordinatePair.h>

namespace flx {
CoordinatePair EPA(const ShapePair &pair, const Plex &initial_plex);
} // namespace flx
