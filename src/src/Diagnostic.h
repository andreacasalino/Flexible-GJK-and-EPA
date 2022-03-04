#ifdef GJK_EPA_DIAGNOSTIC
#pragma once

#include <Hull/Coordinate.h>
#include <nlohmann/json.hpp>

namespace flx::diagnostic {
class Diagnostic {
public:
  Diagnostic() = default;
  ~Diagnostic();

  nlohmann::json &getLog() { return log; }

private:
  nlohmann::json log;
};

void to_json(nlohmann::json &recipient, const hull::Coordinate &c);
} // namespace flx::diagnostic
#endif
