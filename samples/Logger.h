/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <nlohmann/json.hpp>
#include <set>

namespace logger {
// You can dig into the sources to understand the functionalities
// stored in this namespace if you really are interested in.
// However, this is not strictly required to understand how to use
// the functions in GjkEpa.h.
//
// In essence, this class is simply generating json files
// in order to later display the results in cool python plots.

class SubPlot {
  friend class Figure;

public:
  void addShape(const flx::shape::ConvexShape &shape);

  void addLine(const hull::Coordinate &a, const hull::Coordinate &b);

  void toJson(nlohmann::json &recipient) const;

private:
  SubPlot(const std::string &title) : title(title){};

  const std::string title;
  std::set<const flx::shape::ConvexShape *> shapes_ptr;
  std::vector<nlohmann::json> shapes;
  std::vector<nlohmann::json> lines;
};

class Figure {
public:
  Figure() = default;

  SubPlot &addSubPlot(const std::string &title);

  void log(const std::string &file_name) const;

private:
  std::vector<SubPlot> sub_plots;
};

void logSingleQuery(const flx::shape::ConvexShape &shape_a,
                    const flx::shape::ConvexShape &shape_b,
                    const flx::QueryResult &result,
                    const std::string &file_name);
} // namespace logger
