#ifdef GJK_EPA_DIAGNOSTIC

#include "Diagnostic.h"

namespace flx::diagnostic {
namespace {
static const std::string INITIAL_GJK_FIELD = std::string{"initial-GJK"};
static const std::string FINISHING_GJK_FIELD = std::string{"finishing-GJK"};
static const std::string EPA_FIELD = std::string{"EPA"};
} // namespace

Diagnostic::Diagnostic() {
  log[INITIAL_GJK_FIELD] = nlohmann::json::array();
  log[FINISHING_GJK_FIELD] = nlohmann::json::array();
  log[EPA_FIELD] = nlohmann::json::array();
};

nlohmann::json &Diagnostic::add_to_initial_GJK_loop() {
  return log[INITIAL_GJK_FIELD].emplace_back();
}

nlohmann::json &Diagnostic::add_to_finishing_GJK_loop() {
  return log[FINISHING_GJK_FIELD].emplace_back();
}

nlohmann::json &Diagnostic::add_EPA_loop() {
  return log[EPA_FIELD].emplace_back();
}
} // namespace flx::diagnostic
#endif