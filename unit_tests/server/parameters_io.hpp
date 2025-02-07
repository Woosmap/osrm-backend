#ifndef OSRM_TEST_SERVER_PARAMETERS_IO
#define OSRM_TEST_SERVER_PARAMETERS_IO

#include "engine/api/route_parameters.hpp"
#include "engine/api/base_parameters.hpp"
#include "engine/approach.hpp"
#include "engine/bearing.hpp"

#include <ostream>

namespace osrm
{
namespace engine
{
namespace api
{
inline std::ostream &operator<<(std::ostream &out, api::RouteParameters::GeometriesType geometries)
{
    switch (geometries)
    {
    case api::RouteParameters::GeometriesType::GeoJSON:
        out << "GeoJSON";
        break;
    case api::RouteParameters::GeometriesType::Polyline:
        out << "Polyline";
        break;
    default:
        BOOST_ASSERT_MSG(false, "GeometriesType not fully captured");
    }
    return out;
}

inline std::ostream &operator<<(std::ostream &out, api::RouteParameters::OverviewType overview)
{
    switch (overview)
    {
    case api::RouteParameters::OverviewType::False:
        out << "False";
        break;
    case api::RouteParameters::OverviewType::Full:
        out << "Full";
        break;
    case api::RouteParameters::OverviewType::Simplified:
        out << "Simplified";
        break;
    default:
        BOOST_ASSERT_MSG(false, "OverviewType not fully captured");
    }
    return out;
}

inline std::ostream &operator<<( std::ostream &out, api::BaseParameters::OptimizeType opt )
{   //  Necessary for the BOOST_CHECK_EQUAL
    switch (opt) {
    case api::BaseParameters::OptimizeType::Weight:
        out << "Weight";
        break;
    case api::BaseParameters::OptimizeType::Time:
        out << "Time";
        break;
    case api::BaseParameters::OptimizeType::Distance:
        out << "Distance";
        break;
    default:
        BOOST_ASSERT_MSG(false, "OptimizeType not fully captured");
    }
    return out;
}
} // namespace api

inline std::ostream &operator<<(std::ostream &out, Bearing bearing)
{
    out << bearing.bearing << "," << bearing.range;
    return out;
}

inline std::ostream &operator<<(std::ostream &out, Approach approach)
{
    out << static_cast<int>(approach);
    return out;
}
} // namespace engine
} // namespace osrm

#endif
