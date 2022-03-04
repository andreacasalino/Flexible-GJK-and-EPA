#ifdef GJK_EPA_DIAGNOSTIC
#pragma once

#include <nlohmann/json.hpp>

namespace flx::diagnostic {
class Diagnostic {
public:
  Diagnostic();

  void print() const;

  nlohmann::json &add_to_initial_GJK_loop();
  nlohmann::json &add_to_finishing_GJK_loop();

  nlohmann::json &add_EPA_loop();

private:
  nlohmann::json log;
};

void to_json(nlohmann::json &recipient, const hull::Coordinate &c);
} // namespace flx::diagnostic
#endif
