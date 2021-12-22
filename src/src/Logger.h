/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Hull/Coordinate.h>
#include <sstream>
#include <string>

namespace flx {
class Logger {
public:
  Logger(const std::string &fileName) : fileName(fileName){};
  ~Logger();

  template <typename T> inline void add(const T &o) { this->stream << o; };

private:
  std::string fileName;
  std::stringstream stream;
};

inline void add(std::stringstream &stream, const Coordinate &c) {
  stream << '[' << c.x << ',' << c.y << ',' << c.z << ']';
};

inline std::string str(const Coordinate &c) {
  std::stringstream stream;
  stream << '[' << c.x << ',' << c.y << ',' << c.z << ']';
  return stream.str();
};

class Array {
public:
  Array() = default;

  void add(const std::string &element);
  std::string str() const;

private:
  bool isFirst = true;
  std::stringstream stream;
};
} // namespace flx
