/**
 * Author:    Andrea Casalino
 * Created:   05.01.2020
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Flexible-GJK-and-EPA/shape/ConvexShape.h>
#include <functional>

namespace flx::shape {
template <typename PointCollectionIt, typename SupportPredicate,
          typename ToCoordinatePredicate>
class PointCloud : public ConvexShape {
public:
  PointCloud(const PointCollectionIt &begin, const PointCollectionIt &end,
             SupportPredicate &&support_pred,
             ToCoordinatePredicate &&convert_pred)
      : begin(begin), end(end), support_predicate(support_pred),
        convert_predicate(convert_pred){};

  void getSupport(hull::Coordinate &support,
                  const hull::Coordinate &direction) const final {
    auto it = begin;
    float max_support_value = support_predicate(it, direction), support_value;
    PointCollectionIt max_support_point = it;
    ++it;
    while (it; it != end; ++it) {
      support_value = support_predicate(it, direction);
      if (support_value > max_support_value) {
        max_support_value = support_value;
        max_support_point = it;
      }
    }
    return convert_predicate(it);
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
