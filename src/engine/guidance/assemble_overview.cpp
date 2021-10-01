#include "engine/douglas_peucker.hpp"
#include "engine/guidance/leg_geometry.hpp"
#include "util/viewport.hpp"
#include "engine/concaveman.hpp"

#include <iterator>
#include <limits>
#include <numeric>
#include <vector>

namespace osrm
{
namespace engine
{
namespace guidance
{
namespace
{

unsigned calculateOverviewZoomLevel(const std::vector<LegGeometry> &leg_geometries)
{
    util::Coordinate south_west{util::FixedLongitude{std::numeric_limits<int>::max()},
                                util::FixedLatitude{std::numeric_limits<int>::max()}};
    util::Coordinate north_east{util::FixedLongitude{std::numeric_limits<int>::min()},
                                util::FixedLatitude{std::numeric_limits<int>::min()}};

    for (const auto &leg_geometry : leg_geometries)
    {
        for (const auto &coord : leg_geometry.locations)
        {
            south_west.lon = std::min(south_west.lon, coord.lon);
            south_west.lat = std::min(south_west.lat, coord.lat);

            north_east.lon = std::max(north_east.lon, coord.lon);
            north_east.lat = std::max(north_east.lat, coord.lat);
        }
    }

    return util::viewport::getFittedZoom(south_west, north_east);
}

unsigned calculateOverviewZoomLevel(const std::vector<util::Coordinate> &geometry)
{
    util::Coordinate south_west{util::FixedLongitude{std::numeric_limits<int>::max()},
                                util::FixedLatitude{std::numeric_limits<int>::max()}};
    util::Coordinate north_east{util::FixedLongitude{std::numeric_limits<int>::min()},
                                util::FixedLatitude{std::numeric_limits<int>::min()}};

    for (const auto &coord : geometry)
    {
        south_west.lon = std::min(south_west.lon, coord.lon);
        south_west.lat = std::min(south_west.lat, coord.lat);

        north_east.lon = std::max(north_east.lon, coord.lon);
        north_east.lat = std::max(north_east.lat, coord.lat);
    }

    return util::viewport::getFittedZoom(south_west, north_east);
}

} // namespace

std::vector<util::Coordinate> assembleOverview(const std::vector<LegGeometry> &leg_geometries,
                                               const bool use_simplification)
{
    auto overview_size =
        std::accumulate(leg_geometries.begin(),
                        leg_geometries.end(),
                        0,
                        [](const std::size_t sum, const LegGeometry &leg_geometry) {
                            return sum + leg_geometry.locations.size();
                        }) -
        leg_geometries.size() + 1;
    std::vector<util::Coordinate> overview_geometry;
    overview_geometry.reserve(overview_size);

    using GeometryIter = decltype(overview_geometry)::const_iterator;

    auto leg_reverse_index = leg_geometries.size();
    const auto insert_without_overlap = [&leg_reverse_index, &overview_geometry](GeometryIter begin,
                                                                                 GeometryIter end) {
        // not the last leg
        if (leg_reverse_index > 1)
        {
            --leg_reverse_index;
            end = std::prev(end);
        }
        overview_geometry.insert(overview_geometry.end(), begin, end);
    };

    if (use_simplification)
    {
        const auto zoom_level = std::min(18u, calculateOverviewZoomLevel(leg_geometries));
        for (const auto &geometry : leg_geometries)
        {
            const auto simplified =
                douglasPeucker(geometry.locations.begin(), geometry.locations.end(), zoom_level);
            insert_without_overlap(simplified.begin(), simplified.end());
        }
    }
    else
    {
        for (const auto &geometry : leg_geometries)
        {
            insert_without_overlap(geometry.locations.begin(), geometry.locations.end());
        }
    }

    return overview_geometry;
}

std::vector<util::Coordinate> reduceOverview(const std::vector<util::Coordinate> &geometry,
                                             const unsigned alpha_max, const bool use_simplification)
{
    typedef double T;
    typedef std::array<T, 2> point_type;

    auto hull = convexHull( geometry ) ;
    std::vector<std::array<T, 2>> points, convex_hull ;
    std::transform( geometry.begin(), geometry.end(), std::back_inserter(points), [](const util::Coordinate& p) -> std::array<T, 2>{
      auto xy = util::web_mercator::fromWGS84(p);
      return { static_cast<T>(xy.lon), static_cast<T>(xy.lat) } ;
    }) ;
    std::transform( hull.begin(), hull.end(), std::back_inserter(convex_hull), [](const util::Coordinate& p) -> std::array<T, 2>{
        auto xy = util::web_mercator::fromWGS84(p);
        return { static_cast<T>(xy.lon), static_cast<T>(xy.lat) } ;
    }) ;

    double concavity = static_cast<double>(alpha_max) ;
    std::vector<std::array<T, 2>> concave_hull = concaveman<T,16>( points, convex_hull, concavity, 0.0) ;

    std::vector<util::Coordinate> res ;
    std::transform( concave_hull.begin(), concave_hull.end(), std::back_inserter(res), [](const point_type& p){
      return util::web_mercator::toWGS84( util::FloatCoordinate(util::FloatLongitude{p[0]}, util::FloatLatitude{p[1]}) ) ;
    }) ;
    if( use_simplification ) {
        const auto zoom_level = std::min(18u, calculateOverviewZoomLevel(geometry));
        res = douglasPeucker(res.begin(), res.end(), zoom_level);
    }
#ifdef DEBUG
    std::cout << "LINESTRING(" ;
    for( const auto& p : res)
        std::cout << util::toFloating(p.lon) << ' ' << util::toFloating(p.lat) << ',' ;
    std::cout << ')' << std::endl ;
#endif
    return res ;
}
} // namespace guidance
} // namespace engine
} // namespace osrm
