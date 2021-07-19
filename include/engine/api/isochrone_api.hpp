#ifndef OSRM_ISOCHRONE_API_HPP
#define OSRM_ISOCHRONE_API_HPP

#include "engine/api/isochrone_parameters.hpp"
#include "engine/api/route_api.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/internal_route_result.hpp"
#include "engine/map_matching/sub_matching.hpp"

#include "engine/api/base_result.hpp"
#include "engine/api/json_factory.hpp"


#include "engine/guidance/assemble_geometry.hpp"
#include "engine/guidance/assemble_leg.hpp"
#include "engine/guidance/assemble_overview.hpp"
#include "engine/guidance/assemble_route.hpp"
#include "engine/guidance/assemble_steps.hpp"


#include "util/integer_range.hpp"

#include <boost/range/algorithm/transform.hpp>
#include <boost/optional.hpp>

#include <iterator>
#include <algorithm>

namespace osrm
{
namespace engine
{
namespace api
{

class IsochroneAPI final : public RouteAPI
{
  public:
    IsochroneAPI(const datafacade::BaseDataFacade &facade_,
                 const IsochroneParameters &parameters_)
        : RouteAPI(facade_, parameters_), parameters(parameters_)
    {
    }

    virtual void
    MakeResponse(const std::vector<util::Coordinate> &points,
                 osrm::engine::api::ResultT &response) const
    {
        if (response.is<flatbuffers::FlatBufferBuilder>())
        {
            auto &fb_result = response.get<flatbuffers::FlatBufferBuilder>();
            MakeResponse(points,fb_result);
        }
        else
        {
            auto &json_result = response.get<util::json::Object>();
            MakeResponse(points, json_result);
        }
    }

    virtual void
    MakeResponse(const std::vector<util::Coordinate> &/*points*/,
                 flatbuffers::FlatBufferBuilder &/*fb_result*/) const
    {/*
        auto number_of_sources = parameters.sources.size();
        auto number_of_destinations = parameters.destinations.size();

        auto data_timestamp = facade.GetTimestamp();
        flatbuffers::Offset<flatbuffers::String> data_version_string;
        if (!data_timestamp.empty())
        {
            data_version_string = fb_result.CreateString(data_timestamp);
        }

        // symmetric case
        flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fbresult::Waypoint>>> sources;
        if (parameters.sources.empty())
        {
            if (!parameters.skip_waypoints)
            {
                sources = MakeWaypoints(fb_result, phantoms);
            }
            number_of_sources = phantoms.size();
        }
        else
        {
            if (!parameters.skip_waypoints)
            {
                sources = MakeWaypoints(fb_result, phantoms, parameters.sources);
            }
        }

        flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fbresult::Waypoint>>>
            destinations;
        if (parameters.destinations.empty())
        {
            if (!parameters.skip_waypoints)
            {
                destinations = MakeWaypoints(fb_result, phantoms);
            }
            number_of_destinations = phantoms.size();
        }
        else
        {
            if (!parameters.skip_waypoints)
            {
                destinations = MakeWaypoints(fb_result, phantoms, parameters.destinations);
            }
        }

        bool use_durations = parameters.annotations & TableParameters::AnnotationsType::Duration;
        flatbuffers::Offset<flatbuffers::Vector<float>> durations;
        if (use_durations)
        {
            durations = MakeDurationTable(fb_result, tables.first);
        }

        bool use_distances = parameters.annotations & TableParameters::AnnotationsType::Distance;
        flatbuffers::Offset<flatbuffers::Vector<float>> distances;
        if (use_distances)
        {
            distances = MakeDistanceTable(fb_result, tables.second);
        }

        bool have_speed_cells =
            parameters.fallback_speed != INVALID_FALLBACK_SPEED && parameters.fallback_speed > 0;
        flatbuffers::Offset<flatbuffers::Vector<uint32_t>> speed_cells;
        if (have_speed_cells)
        {
            speed_cells = MakeEstimatesTable(fb_result, fallback_speed_cells);
        }

        fbresult::TableBuilder table(fb_result);
        table.add_destinations(destinations);
        table.add_rows(number_of_sources);
        table.add_cols(number_of_destinations);
        if (use_durations)
        {
            table.add_durations(durations);
        }
        if (use_distances)
        {
            table.add_distances(distances);
        }
        if (have_speed_cells)
        {
            table.add_fallback_speed_cells(speed_cells);
        }
        auto table_buffer = table.Finish();

        fbresult::FBResultBuilder response(fb_result);
        if (!data_timestamp.empty())
        {
            response.add_data_version(data_version_string);
        }
        response.add_table(table_buffer);
        response.add_waypoints(sources);
        fb_result.Finish(response.Finish());
    */}

    virtual void
    MakeResponse(const std::vector<util::Coordinate> &points,
                 util::json::Object &response) const
    {
        boost::optional<util::json::Value> geometry;
        auto begin = points.begin();
        auto end = points.end();
        if (parameters.geometries == RouteParameters::GeometriesType::Polyline)
        {
            geometry = json::makePolyline<100000>(begin, end);
        }
        else if (parameters.geometries == RouteParameters::GeometriesType::Polyline6)
        {
            geometry = json::makePolyline<1000000>(begin, end);
        }
        else
        {
            BOOST_ASSERT(parameters.geometries == RouteParameters::GeometriesType::GeoJSON);
            geometry = json::makeGeoJSONGeometry(begin, end);
        }

        util::json::Object properties;
        switch( parameters.optimize ) {
        case IsochroneParameters::OptimizeType::Weight :
            properties.values["weight"] = parameters.range ;
            break ;
        case IsochroneParameters::OptimizeType::Distance :
            properties.values["distance"] = parameters.range ;
            break ;
        case IsochroneParameters::OptimizeType::Time :
            properties.values["duration"] = parameters.range ;
            break ;
        }
        if (geometry)
        {
            properties.values["geometry"] = *std::move(geometry);
        }
        response.values["type"] = "Feature" ;
        response.values["properties"] = properties;
        response.values["code"] = "Ok";
    }

  protected:
    const IsochroneParameters &parameters;
};

} // namespace api
} // namespace engine
} // namespace osrm

#endif // OSRM_ISOCHRONE_API_HPP
