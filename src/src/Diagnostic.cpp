#ifdef GJK_EPA_DIAGNOSTIC

#include "Diagnostic.h"
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>

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

namespace {
static std::mutex print_mtx;
static std::size_t print_counter = 0;
static const std::string DIAGNOSTIC_FOLDER = "./GJK-EPA-diagnostic";
} // namespace

void Diagnostic::print() const {
  std::scoped_lock locker(print_mtx);
  if (0 == print_counter) {
    std::filesystem::remove_all(DIAGNOSTIC_FOLDER);
  }
  if (!std::filesystem::exists(DIAGNOSTIC_FOLDER)) {
    std::filesystem::create_directory(DIAGNOSTIC_FOLDER);
  }
  std::stringstream file_name;
  file_name << DIAGNOSTIC_FOLDER << "/log_" << std::to_string(print_counter);
  std::ofstream stream(file_name.str());
  stream << log.dump();
  ++print_counter;
}

void to_json(nlohmann::json &recipient, const hull::Coordinate &c) {
  recipient = nlohmann::json::array();
  recipient.emplace_back() = c.x;
  recipient.emplace_back() = c.y;
  recipient.emplace_back() = c.z;
};
} // namespace flx::diagnostic
#endif