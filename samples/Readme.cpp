#include <Flexible-GJK-and-EPA/GjkEpa.h>

int main() {
  // build the first convex shape ... have a look at the samples to understand
  // how to do it
  const flx::shape::ConvexShape &shapeA = ;
  // build the second convex shape ... have a look at the samples to understand
  // how to do it
  const flx::shape::ConvexShape &shapeB = ;

  {
    // you can simply check that the shapes are in collision or not by calling
    bool result = flx::is_collision_present(shapeA, shapeB);
    // this will call only the initial iterations of the GJK (see the
    // documentation) required to answer the query.
  }

  {
    // if you suspect the shapes ARE in collision, you can get the penetration
    // vector (see the documentation) by calling
    std::optional<flx::CoordinatePair> result =
        flx::get_penetration_info(shapeA, shapeB);
    // this will call the initial iterations of the GJK + the EPA ones in
    // case the shapes are in collision.
    //
    // On the contrary, when the shapes are not in collision a std::nullopt is
    // actually returned, and only the  initial iterations of the GJK are done.
    //
    // If the result is not null, you can access the extremals of the
    // penetration vector:
    if (std::nullopt != result) {
      const hull::Coordinate &point_on_shapeA = result->point_in_shape_a;
      const hull::Coordinate &point_on_shapeB = result->point_in_shape_b;
    }
  }

  {
    // if you suspect the shapes ARE NOT in collision, you can get the
    // closest pair of points on the 2 shapes by calling
    std::optional<flx::CoordinatePair> result =
        flx::get_closest_points(shapeA, shapeB);
    // this will call only initial iterations of the GJK + refining iterations
    // of the GJK, evolving the plex till finding the closest region of the
    // Minkowski difference to the origin. This is actually done only in case
    // the shapes are not in collision.
    //
    // On the contrary, when the shapes are in collision a std::nullopt is
    // actually returned, and only the  initial iterations of the GJK are done.
    //
    // If the result is not null, you can access the the closest pair from the
    // result:
    if (std::nullopt != result) {
      const hull::Coordinate &closest_point_on_shapeA =
          result->point_in_shape_a;
      const hull::Coordinate &closest_point_on_shapeB =
          result->point_in_shape_b;
    }
  }

  {
    // ...or, you can ask to perform a generic complex query that leads to call
    // the EPA or the finishing iterations of the GJK to respectibely compute
    // the penetration vector or the closest pair:
    flx::QueryResult result =
        flx::get_closest_points_or_penetration_info(shapeA, shapeB);
    // and then access the results (which might have 2 different meanings)
    bool result_meaning = result.is_closest_pair_or_penetration_info;
    if (result_meaning) {
      // is the closest pair
      const hull::Coordinate &closest_point_on_shapeA =
          result.result.point_in_shape_a;
      const hull::Coordinate &closest_point_on_shapeB =
          result.result.point_in_shape_b;
    } else {
      // extremals of the penetration vector
      const hull::Coordinate &point_on_shapeA = result.result.point_in_shape_a;
      const hull::Coordinate &point_on_shapeB = result.result.point_in_shape_b;
    }
  }

  return 0;
}
