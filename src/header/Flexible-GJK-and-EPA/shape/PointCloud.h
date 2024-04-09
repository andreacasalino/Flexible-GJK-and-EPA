/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/Error.h>
#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>

#include <algorithm>
#include <functional>
#include <limits>

namespace flx::shape {
/**
 * @brief Wrapper around any kind of vertices collection, representing a point
 * cloud.
 */
template <typename PointCollectionIt> class PointCloud : public ConvexShape {
public:
  using T = typename PointCollectionIt::value_type;

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
  template <typename SupportPredT, typename ConvertPredT>
  PointCloud(const PointCollectionIt &begin, const PointCollectionIt &end,
             SupportPredT &&support_pred, ConvertPredT &&convert_pred)
      : begin_(begin), end_(end),
        support_predicate(std::forward<SupportPredT>(support_pred)),
        convert_predicate(std::forward<ConvertPredT>(convert_pred)) {
    if (begin_ == end_) {
      throw Error{"Empty points collection"};
    }
  };

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const final {
    float distance_max = std::numeric_limits<float>::lowest();
    const T *best = nullptr;
    std::for_each(begin_, end_, [&](const T &candidate) {
      float d = support_predicate(candidate, direction);
      if (distance_max < d) {
        distance_max = d;
        best = &candidate;
      }
    });
    support = convert_predicate(*best);
  };

  auto begin() const { return begin_; }
  auto end() const { return end_; }

private:
  PointCollectionIt begin_;
  PointCollectionIt end_;

  std::function<float(const T &, const hull::Coordinate &)> support_predicate;
  std::function<hull::Coordinate(const T &)> convert_predicate;
};
} // namespace flx::shape
