#include "engine/douglas_peucker.hpp"
#include "util/coordinate.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/integer_range.hpp"
#include "util/web_mercator.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stack>
#include <utility>

namespace osrm
{
namespace engine
{

// Normed to the thresholds table
std::uint64_t fastPerpendicularDistance(const util::FloatCoordinate &projected_start,
                                        const util::FloatCoordinate &projected_target,
                                        const util::FloatCoordinate &projected)
{
    util::FloatCoordinate projected_point_on_segment;
    std::tie(std::ignore, projected_point_on_segment) =
        util::coordinate_calculation::projectPointOnSegment(
            projected_start, projected_target, projected);
    auto squared_distance = util::coordinate_calculation::squaredEuclideanDistance(
        projected, projected_point_on_segment);
    return squared_distance;
}

std::vector<util::Coordinate> douglasPeucker(std::vector<util::Coordinate>::const_iterator begin,
                                             std::vector<util::Coordinate>::const_iterator end,
                                             const unsigned zoom_level)
{
    BOOST_ASSERT_MSG(zoom_level < detail::DOUGLAS_PEUCKER_THRESHOLDS_SIZE,
                     "unsupported zoom level");

    const std::size_t size = std::distance(begin, end);
    if (size < 2)
    {
        return {};
    }

    std::vector<util::FloatCoordinate> projected_coordinates(size);
    std::transform(begin, end, projected_coordinates.begin(), [](const util::Coordinate coord) {
        return util::web_mercator::fromWGS84(coord);
    });

    std::vector<bool> is_necessary(size, false);
    BOOST_ASSERT(is_necessary.size() >= 2);
    is_necessary.front() = true;
    is_necessary.back() = true;
    using GeometryRange = std::pair<std::size_t, std::size_t>;

    std::stack<GeometryRange> recursion_stack;

    recursion_stack.emplace(0UL, size - 1);

    // mark locations as 'necessary' by divide-and-conquer
    while (!recursion_stack.empty())
    {
        // pop next element
        const GeometryRange pair = recursion_stack.top();
        recursion_stack.pop();
        // sanity checks
        BOOST_ASSERT_MSG(is_necessary[pair.first], "left border must be necessary");
        BOOST_ASSERT_MSG(is_necessary[pair.second], "right border must be necessary");
        BOOST_ASSERT_MSG(pair.second < size, "right border outside of geometry");
        BOOST_ASSERT_MSG(pair.first <= pair.second, "left border on the wrong side");

        std::uint64_t max_distance = 0;
        auto farthest_entry_index = pair.second;

        // sweep over range to find the maximum
        for (auto idx = pair.first + 1; idx != pair.second; ++idx)
        {
            using namespace util::coordinate_calculation;
            const auto distance = fastPerpendicularDistance(projected_coordinates[pair.first],
                                                            projected_coordinates[pair.second],
                                                            projected_coordinates[idx]);
            // found new feasible maximum?
            if (distance > max_distance &&
                distance > detail::DOUGLAS_PEUCKER_THRESHOLDS[zoom_level])
            {
                farthest_entry_index = idx;
                max_distance = distance;
            }
        }

        // check if maximum violates a zoom level dependent threshold
        if (max_distance > detail::DOUGLAS_PEUCKER_THRESHOLDS[zoom_level])
        {
            //  mark idx as necessary
            is_necessary[farthest_entry_index] = true;
            if (pair.first < farthest_entry_index)
            {
                recursion_stack.emplace(pair.first, farthest_entry_index);
            }
            if (farthest_entry_index < pair.second)
            {
                recursion_stack.emplace(farthest_entry_index, pair.second);
            }
        }
    }

    auto simplified_size = std::count(is_necessary.begin(), is_necessary.end(), true);
    std::vector<util::Coordinate> simplified_geometry;
    simplified_geometry.reserve(simplified_size);
    for (auto idx : util::irange<std::size_t>(0UL, size))
    {
        if (is_necessary[idx])
        {
            simplified_geometry.push_back(begin[idx]);
        }
    }

    return simplified_geometry;
}

// A C++ program to find convex hull of a set of points
// Refer http://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
// for explanation of orientation()

// Define Infinite (Using INT_MAX caused overflow problems)
#define INF 10000

struct Point
{
    int lon;
    int lat;
};

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(util::FloatCoordinate p, util::FloatCoordinate q, util::FloatCoordinate r)
{
    auto val = (double)(q.lat - p.lat) * (double)(r.lon - q.lon) - (double)(q.lon - p.lon) * (double)(r.lat - q.lat);

    if (val == 0)
        return 0; // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// Prints convex hull of a set of n points.
std::vector<util::Coordinate> convexHull(const std::vector<util::Coordinate> &points)
{
    int n = points.size() ;
    // There must be at least 3 points
    if (n < 3)
        return std::vector<util::Coordinate>();
    std::vector<util::FloatCoordinate> projected_coordinates(n);
    std::transform(points.begin(), points.end(), projected_coordinates.begin(), [](const util::Coordinate coord) {
      return util::web_mercator::fromWGS84(coord);
    });


    // Initialize Result
    int next[n];
    for (int i = 0; i < n; i++)
        next[i] = -1;

    // Find the leftmost point
    int l = 0;
    for (int i = 1; i < n; i++)
        if (projected_coordinates[i].lon < projected_coordinates[l].lon)
            l = i;

    // Start from leftmost point, keep moving counterclockwise
    // until reach the start point again
    int p = l, q;
    do
    {
        // Search for a point 'q' such that orientation(p, i, q) is
        // counterclockwise for all points 'i'
        q = (p + 1) % n;
        for (int i = 0; i < n; i++)
            if (orientation(projected_coordinates[p], projected_coordinates[i], projected_coordinates[q]) == 2)
                q = i;

        next[p] = q; // Add q to result as a next point of p
        p = q; // Set p as q for next iteration
    }
    while (p != l);

    std::vector<util::Coordinate> result ;
    // Print Result
    for (int i = 0; i < n; i++)
    {
        if (next[i] != -1)
            result.push_back(points[i]) ;
            //cout << "(" << points[i].lon << ", " << points[i].lat << ")\n";
    }
    result.push_back(points[0]) ;
    return result ;
}

/** \brief Convexification of a polyline.<br>
 * The applied treatment will eliminate the sharpest inside peaks.<br>
 * This can go from getting a convex hull of the given polyline (max_alpha=0),
 * to a no-operation (max_alpha=180)
 * <br>\!/ Only works with simple polylines as input
 * @param points Array of points forming a simple polyline
 * @param max_alpha Maximum angle (outside angle in degrees) allowed for an inner peak
 * @return A polyline without all the sharpest inner peaks
 */
std::vector<util::Coordinate> almostConvexHull(const std::vector<util::Coordinate> &points, const double max_alpha) {
    //  Shortcut that would finally result in no-operation on the entry polyline
    if(max_alpha >=180.0 )
        return points ;
/*
    std::vector<util::Coordinate> points = {
        util::Coordinate( util::FloatLongitude{3.95796}, util::FloatLatitude{43.57991} ),
        util::Coordinate( util::FloatLongitude{3.99097},util::FloatLatitude{43.55171 } ),
        util::Coordinate( util::FloatLongitude{3.95976},util::FloatLatitude{43.57635 } ),
        util::Coordinate( util::FloatLongitude{3.99007},util::FloatLatitude{43.55078 } ),
        util::Coordinate( util::FloatLongitude{3.95893},util::FloatLatitude{43.57601 } ),
        util::Coordinate( util::FloatLongitude{3.98813},util::FloatLatitude{43.54896} ),
        util::Coordinate( util::FloatLongitude{3.98382},util::FloatLatitude{43.55029} )
    } ;*/
    //  Strong hypothesis : We suppose the points are already sorted in heading order (simple polyline)
    std::vector<util::Coordinate> result ;
    auto min_lon = std::min_element( points.begin(), points.end(), [](const util::Coordinate &p1, const util::Coordinate &p2) {
        return p1.lon<p2.lon ;
    } );
    result.push_back(*min_lon) ;
    auto cur = min_lon ;
    do {
        auto curPoint = *cur ;
        cur = std::next(cur) ;
        if( cur==points.end() )
            cur = points.begin() ;
        //  Skip similar points (begin and end are equal for closed polygones
        if( curPoint.lon==cur->lon && curPoint.lat==cur->lat )
            continue ;
        if( result.size()<2 ) {
            result.push_back(*cur);
            continue;
        }
        using namespace osrm::util::coordinate_calculation;
        while( result.size()>=2 )
        {
            auto next_bearing = bearing(result.back(), *cur);
            while(next_bearing > 180) next_bearing -= 360;
            while(next_bearing < -180) next_bearing += 360;
            auto prev_bearing = bearing(result[result.size() - 2], result.back());
            while(prev_bearing > 180) prev_bearing -= 360;
            while(prev_bearing < -180) prev_bearing += 360;
            //  For a convex hull, with sorted points in clockwise order, we should never turn left
            auto diff_bearing = next_bearing - prev_bearing;
            while(diff_bearing > 180) diff_bearing -= 360;
            while(diff_bearing < -180) diff_bearing += 360;
            if (diff_bearing >= -max_alpha)
                break ;
            result.pop_back();
        }
        result.push_back(*cur);
    } while( cur!=min_lon ) ;
    return result ;
}

} // namespace engine
} // namespace osrm
