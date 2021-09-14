#include "engine/douglas_peucker.hpp"
#include "engine/guidance/leg_geometry.hpp"
#include "util/viewport.hpp"
#include "engine/concaveman.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <iterator>
#include <limits>
#include <numeric>
#include <tuple>
#include <util/coordinate_calculation.hpp>
#include <utility>
#include <vector>

namespace bg = boost::geometry;

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

typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef K::FT                                                FT;
typedef K::Point_2                                           Point;
typedef K::Segment_2                                         Segment;
typedef K::Point_2                                           Vertex;
typedef CGAL::Alpha_shape_vertex_base_2<K>                   Vb;
typedef CGAL::Alpha_shape_face_base_2<K>                     Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>          Tds;
typedef CGAL::Delaunay_triangulation_2<K,Tds>                Triangulation_2;
typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
typedef Alpha_shape_2::Alpha_shape_edges_iterator            Alpha_shape_edges_iterator;
typedef Alpha_shape_2::Alpha_iterator                        Alpha_iterator;

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

std::vector<util::Coordinate> reduceOverview_internal(const std::vector<util::Coordinate> &geometry,
                                                      const unsigned alpha_max)
{
    const auto zoom_level = std::min(18u, calculateOverviewZoomLevel(geometry));
    //  Douglas-Peuker shall be applied before and after the convexification of the polyline
    //  1st time : to eliminate almost-aligned points that may full the convexification
    //  2nd time : to clean the convexification result
    auto convex_geom = almostConvexHull( douglasPeucker(geometry.begin(), geometry.end(), zoom_level), (double)alpha_max ) ;
    return douglasPeucker(convex_geom.begin(), convex_geom.end(), zoom_level) ;
}


std::vector<util::Coordinate> reduceOverview_concaveman(const std::vector<util::Coordinate> &geometry,
                                             const unsigned alpha_max)
{
    typedef double T;
    typedef bg::model::point<T, 2, bg::cs::cartesian> point_t;
    typedef bg::model::multi_point<point_t> mpoint_t;
    typedef std::array<T, 2> point_type;

/*    mpoint_t mpt1;
    std::for_each( geometry.begin(), geometry.end(), [&mpt1](const util::Coordinate& p){
      bg::append(mpt1, point_t(static_cast<T>(util::toFloating(p.lon)), static_cast<T>(util::toFloating(p.lat)) ) ) ;
    }) ;
    typedef boost::geometry::model::polygon<point_t> polygon;
    polygon hull;
    boost::geometry::convex_hull(mpt1, hull);
*/    auto hull = convexHull( geometry ) ;

    std::vector<std::array<T, 2>> points, convex_hull ;
    std::transform( geometry.begin(), geometry.end(), std::back_inserter(points), [](const util::Coordinate& p) -> std::array<T, 2>{
      auto xy = util::web_mercator::fromWGS84(p);
      return { static_cast<T>(xy.lon), static_cast<T>(xy.lat) } ;
    }) ;
    std::transform( hull.begin(), hull.end(), std::back_inserter(convex_hull), [](const util::Coordinate& p) -> std::array<T, 2>{
        auto xy = util::web_mercator::fromWGS84(p);
        return { static_cast<T>(xy.lon), static_cast<T>(xy.lat) } ;
    }) ;

    double concavity = static_cast<double>(5*alpha_max)/180.0 ;
    std::vector<std::array<T, 2>> concave_hull = concaveman<T,16>( points, convex_hull, concavity, 0.0) ;

    std::vector<util::Coordinate> res ;
    std::transform( concave_hull.begin(), concave_hull.end(), std::back_inserter(res), [](const point_type& p){
      return util::web_mercator::toWGS84( util::FloatCoordinate(util::FloatLongitude{p[0]}, util::FloatLatitude{p[1]}) ) ;
    }) ;
    const auto zoom_level = std::min(18u, calculateOverviewZoomLevel(geometry));
    return douglasPeucker(res.begin(), res.end(), zoom_level) ;
}


std::vector<util::Coordinate> reduceOverview_cgal(const std::vector<util::Coordinate> &geometry,
                                                        const unsigned /*alpha_max*/)
{
    typedef double T;
    typedef bg::model::point<T, 2, bg::cs::cartesian> point_t;
    typedef bg::model::multi_point<point_t> mpoint_t;
    typedef std::array<T, 2> point_type;

    std::list<Point> points;
    std::transform( geometry.begin(), geometry.end(), std::back_inserter(points), [](const util::Coordinate& p){
        auto xy = util::web_mercator::fromWGS84(p);
        return Point( static_cast<double>(xy.lat), static_cast<double>(xy.lon) ) ;
    }) ;
    Alpha_shape_2 as(points.begin(), points.end(), FT(0.0000001), Alpha_shape_2::REGULARIZED );
    //std::for_each( as.alpha_shape_edges_begin(), as.alpha_shape_edges_end(), []())
    as.alpha_shape_edges_begin()->first.
    //Alpha_iterator opt = as.find_optimal_alpha(2);
    //as.set_alpha(*opt);
    std::vector<util::Coordinate> res ;
    //auto p = *as.alpha_shape_vertices_begin() ;
    //p.operator*().handle()->point(). ;
    std::transform( as.alpha_shape_vertices_begin(), as.alpha_shape_vertices_end(), std::back_inserter(res), [&](const auto& o){
        auto p = o->handle()->point() ;
        return util::web_mercator::toWGS84( util::FloatCoordinate(util::FloatLongitude{p.y()}, util::FloatLatitude{p.x()}) ) ;
    });
    //CGAL::hilbert_sort(res_p.begin(),res_p.end());
    /*std::vector<util::Coordinate> res ;
    std::transform( pts.begin(), pts.end(), std::back_inserter(res), [](const auto& p){
      return util::web_mercator::toWGS84( util::FloatCoordinate(util::FloatLongitude{p.y()}, util::FloatLatitude{p.x()}) ) ;
      //return util::Coordinate( util::UnsafeFloatLongitude{p.y()}, util::UnsafeFloatLatitude{p.x()} ) ;
    });*/
    std::cout << "LINESTRING(" ;
    auto separator = '\0' ;
    for( auto pos : res ) {
        if (separator)
            std::cout << separator;
        std::cout << util::toFloating(pos.lon) << ' ' << util::toFloating(pos.lat) ;
        separator = ',';
    }
    std::cout << ")" << std::endl ;
    //  Connect by nearest
    std::vector<util::Coordinate> poly ;
    util::Coordinate pos = res.back() ;
    poly.push_back( pos ) ;
    res.pop_back() ;
    while( !res.empty() ) {
        const auto nearest = std::min_element( res.begin(), res.end(), [&](const util::Coordinate &p1, const util::Coordinate &p2){
            const auto d1 = util::coordinate_calculation::haversineDistance( pos, p1) ;
            const auto d2 = util::coordinate_calculation::haversineDistance( pos, p2) ;
            return d1<d2 ;
        }) ;
        if( nearest!=res.end() ) {
            pos = *nearest ;
            poly.push_back( pos ) ;
            res.erase(nearest) ;
        }
        else
            break ;
    }
    std::cout << "LINESTRING(" ;
    separator = '\0' ;
    for( auto pos : poly ) {
        if (separator)
            std::cout << separator;
        std::cout << util::toFloating(pos.lon) << ' ' << util::toFloating(pos.lat) ;
        separator = ',';
    }
    std::cout << ")" << std::endl ;
    return poly ;
    //CGAL::internal::CC_iterator<
    //    CGAL::Compact_container<CGAL::Alpha_shape_vertex_base_2<CGAL::Epick, CGAL::Triangulation_vertex_base_2<CGAL::Epick, CGAL::Triangulation_ds_vertex_base_2<CGAL::Triangulation_data_structure_2<CGAL::Alpha_shape_vertex_base_2<CGAL::Epick, CGAL::Default, CGAL::Boolean_tag<false>, CGAL::Boolean_tag<false> >, CGAL::Alpha_shape_face_base_2<CGAL::Epick, CGAL::Default, CGAL::Boolean_tag<false>, CGAL::Boolean_tag<false> > > > >, CGAL::Boolean_tag<false>, CGAL::Boolean_tag<false> >, CGAL::Default, CGAL::Default, CGAL::Default>, false
    //        >
}

} // namespace guidance
} // namespace engine
} // namespace osrm
