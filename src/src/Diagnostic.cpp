#ifdef GJK_EPA_DIAGNOSTIC

#include "Diagnostic.h"
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>

namespace flx::diagnostic {
namespace {
static std::mutex print_mtx;
static std::size_t print_counter = 0;
static const std::string DIAGNOSTIC_FOLDER = "./GJK-EPA-diagnostic";
} // namespace

Diagnostic::~Diagnostic() {
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