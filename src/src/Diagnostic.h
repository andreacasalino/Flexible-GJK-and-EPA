#ifdef GJK_EPA_DIAGNOSTIC
#pragma once

#include <Hull/Coordinate.h>
#include <nlohmann/json.hpp>

namespace flx::diagnostic {
class Diagnostic {
public:
  Diagnostic();
  ~Diagnostic();

  void addGjkInitialIter(nlohmann::json &&log);
  void addGjkFinalIter(nlohmann::json &&log);
  void addEpaIter(nlohmann::json &&log);

private:
  void print();

  const std::string log_file_name;
  nlohmann::json diagnostic_log;
};

void to_json(nlohmann::json &recipient, const hull::Coordinate &c);
} // namespace flx::diagnostic
#endif
