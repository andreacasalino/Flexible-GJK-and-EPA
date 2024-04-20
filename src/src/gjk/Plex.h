/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/gjk/Plex.h>

namespace flx::gjk {
// the new vertex is supposed to have already been placed at the front of the
// vertices
void updatePlex(Plex &subject);

// the direction is also normalized
void udpateDirection(Plex &subject, const hull::Coordinate &new_direction);

void set_to_vertex(Plex &subject);
} // namespace flx::gjk
