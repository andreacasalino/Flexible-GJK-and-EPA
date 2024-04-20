/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/Diagnostic.h>
#include <Flexible-GJK-and-EPA/GjkEpa.h>
#include <Flexible-GJK-and-EPA/shape/PointCloud.h>
#include <Hull/Coordinate.h>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <memory>
#include <vector>

namespace flx::utils {
/**
 * @brief Just an example of a 3D coordinate representation
 **/
class Vector3d {
public:
  Vector3d(float x, float y, float z) {
    data[0] = x;
    data[1] = y;
    data[2] = z;
  };

  float x() const { return data[0]; }
  float y() const { return data[1]; }
  float z() const { return data[2]; }

private:
  float data[3];
};

hull::Coordinate to_coordinate(const Vector3d &subject);

float dot_product(const Vector3d &subject, const hull::Coordinate &direction);

using Points = std::shared_ptr<std::vector<Vector3d>>;
Points make_random_cloud(const std::size_t samples);

class Vector3dCloud
    : public shape::PointCloud<std::vector<Vector3d>::const_iterator> {
public:
  Vector3dCloud(const Points &wrapped)
      : shape::PointCloud<std::vector<Vector3d>::const_iterator>(
            wrapped->begin(), wrapped->end(), dot_product, to_coordinate),
        points{wrapped} {};

  const std::vector<Vector3d> &getPoints() const { return *points; };

private:
  Points points;
};

struct LogRootFolder {
  static LogRootFolder &get() {
    static LogRootFolder res = LogRootFolder{};
    return res;
  }

  std::filesystem::path root = std::filesystem::path{LOG_FOLDER};

private:
  LogRootFolder();
};

class ShapesLog {
public:
  ShapesLog(const std::string &title);
  ~ShapesLog();

  void add(const shape::ConvexShape &shape);
  void add(const CoordinatePair &coordinates, bool collision_present);

  static void logSample(const std::string &title,
                        const shape::ConvexShape &shapeA,
                        const shape::ConvexShape &shapeB,
                        const QueryResult &result);

private:
  std::filesystem::path path;
  nlohmann::json content;
};

#ifdef GJK_EPA_DIAGNOSTIC
class GjkEpaLogger : public Observer {
public:
  GjkEpaLogger(const std::string &title);

  void onEvent(Event ev) override;

  void onUpdate(const GjkIteration &info) override;
  void onUpdate(const EpaIteration &info) override;

  void addResult(const shape::ConvexShape &shapeA,
                 const shape::ConvexShape &shapeB,
                 const CoordinatePair &coordinates, bool collision_present);

private:
  std::filesystem::path log_folder;
  std::filesystem::path log_path_current;
  std::size_t iter;
};
#endif

} // namespace flx::utils
