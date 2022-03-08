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

std::string generate_log_name() {
  std::scoped_lock locker(print_mtx);
  if (0 == print_counter) {
    std::filesystem::remove_all(DIAGNOSTIC_FOLDER);
  }
  std::stringstream file_name;
  file_name << DIAGNOSTIC_FOLDER << "/log_" << std::to_string(print_counter);
  ++print_counter;
  return file_name.str();
};
} // namespace

void Diagnostic::print() {
  if (!std::filesystem::exists(DIAGNOSTIC_FOLDER)) {
    std::filesystem::create_directory(DIAGNOSTIC_FOLDER);
  }
  std::ofstream stream(log_file_name);
  stream << diagnostic_log.dump();
}

namespace {
static const std::string GJK_INITIAL_NAME = "GJK_initial";
static const std::string GJK_ENDING_NAME = "GJK_ending";
static const std::string EPA_NAME = "EPA";
} // namespace

Diagnostic::Diagnostic() : log_file_name(generate_log_name()) {
  diagnostic_log[GJK_INITIAL_NAME] = nlohmann::json::array();
  diagnostic_log[GJK_ENDING_NAME] = nlohmann::json::array();
  diagnostic_log[EPA_NAME] = nlohmann::json::array();
}

Diagnostic::~Diagnostic() { print(); }

void Diagnostic::addGjkInitialIter(nlohmann::json &&log) {
  diagnostic_log[GJK_INITIAL_NAME].emplace_back() = std::move(log);
  print();
}

void Diagnostic::addGjkFinalIter(nlohmann::json &&log) {
  diagnostic_log[GJK_ENDING_NAME].emplace_back() = std::move(log);
  print();
}

void Diagnostic::addEpaIter(nlohmann::json &&log) {
  diagnostic_log[EPA_NAME].emplace_back() = std::move(log);
  print();
}

void to_json(nlohmann::json &recipient, const hull::Coordinate &c) {
  recipient = nlohmann::json::array();
  recipient.emplace_back() = c.x;
  recipient.emplace_back() = c.y;
  recipient.emplace_back() = c.z;
};
} // namespace flx::diagnostic
#endif