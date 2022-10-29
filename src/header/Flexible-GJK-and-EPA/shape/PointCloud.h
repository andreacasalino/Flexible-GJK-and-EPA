/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/Error.h>
#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>
#include <functional>

namespace flx::shape {
/**
 * @brief Wrapper around any kind of vertices collection, representing a point
 * cloud.
 */
template <typename PointCollectionIt> class PointCloud : public ConvexShape {
public:
  /**
   * @param begin, the starting iterator of the points collection
   * @param end, the ending iterator of the points collection
   * @param support_pred, A predicate able to compute the dor product:
   * support_pred(const PointCollectionIt& point_in_the_collection, const
   * hull::Coordinate & direction) -> float
   * @param convert_pred, A predicate able to convert a point in the collection
   * into a hull::Coordinate: convert_pred(const PointCollectionIt&
   * point_in_the_collection) -> hull::Coordinate
   *
   * IMPORTANT!!! The iterators delimiting the points collection are stored in
   * this class and should remain valid for the entire life of an instance of
   * this class.
   */
  template <typename SupportPredicate, typename ToCoordinatePredicate>
  PointCloud(const PointCollectionIt &begin, const PointCollectionIt &end,
             const SupportPredicate &support_pred,
             const ToCoordinatePredicate &convert_pred)
      : begin(begin), end(end), support_predicate(support_pred),
        convert_predicate(convert_pred) {
    if (begin == end) {
      throw Error{"Empty points collection"};
    }
  };

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const final {
    auto support_point = begin;
    float support_point_distance = support_predicate(support_point, direction);
    float distance;
    auto it = begin;
    ++it;
    for (; it != end; ++it) {
      distance = support_predicate(it, direction);
      if (distance > support_point_distance) {
        support_point = it;
        support_point_distance = distance;
      }
    }
    support = convert_predicate(support_point);
  };

private:
  const PointCollectionIt begin;
  const PointCollectionIt end;

  const std::function<float(const PointCollectionIt &,
                            const hull::Coordinate &)>
      support_predicate;

  const std::function<hull::Coordinate(const PointCollectionIt &)>
      convert_predicate;
};
} // namespace flx::shape
