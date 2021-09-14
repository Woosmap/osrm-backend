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
    MakeResponse(const PhantomNode &source,
                 const std::vector<util::Coordinate> &points,
                 osrm::engine::api::ResultT &response) const
    {
        if (response.is<flatbuffers::FlatBufferBuilder>())
        {
            auto &fb_result = response.get<flatbuffers::FlatBufferBuilder>();
            MakeResponse(source, points,fb_result);
        }
        else
        {
            auto &json_result = response.get<util::json::Object>();
            MakeResponse(source, points, json_result);
        }
    }

    void
    MakeResponse(const PhantomNode& source,
                 const std::vector<util::Coordinate> &points,
                 flatbuffers::FlatBufferBuilder &fb_result) const
    {
        auto data_timestamp = facade.GetTimestamp();
        flatbuffers::Offset<flatbuffers::String> data_version_string;
        if (!data_timestamp.empty())
        {
            data_version_string = fb_result.CreateString(data_timestamp);
        }

        // symmetric case
        flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fbresult::Waypoint>>>
            waypoints_vector;

        if (!parameters.skip_waypoints)
        {
            auto source_waypoint = BaseAPI::MakeWaypoint(&fb_result, source);
            std::vector<flatbuffers::Offset<fbresult::Waypoint>> waypoints;
            waypoints.push_back( source_waypoint->Finish() ) ;
            waypoints_vector = fb_result.CreateVector(waypoints);
        }

        // Fill geometry
        const std::vector<util::Coordinate> &overview = parameters.overview == RouteParameters::OverviewType::Simplified ? guidance::reduceOverview_internal(points,(100-parameters.convexity_value) *180/100) : points ;
        mapbox::util::variant<flatbuffers::Offset<flatbuffers::String>,
            flatbuffers::Offset<flatbuffers::Vector<const fbresult::Position *>>>
            geometry;
        geometry = MakeGeometry(fb_result, overview.begin(), overview.end());

        auto response = std::make_unique<fbresult::FBResultBuilder>(fb_result);
        fbresult::IsochroneObjectBuilder isochroneObject(fb_result);
        mapbox::util::apply_visitor(GeometryVisitor<fbresult::IsochroneObjectBuilder>(isochroneObject),geometry);

        if (!data_timestamp.empty())
        {
            response->add_data_version(data_version_string);
        }
        response->add_waypoints(waypoints_vector);
        fb_result.Finish(response->Finish());
    }

    template <typename ForwardIter>
    mapbox::util::variant<flatbuffers::Offset<flatbuffers::String>,
        flatbuffers::Offset<flatbuffers::Vector<const fbresult::Position *>>>
    MakeGeometry(flatbuffers::FlatBufferBuilder &builder, ForwardIter begin, ForwardIter end) const
    {
        if (parameters.geometries == RouteParameters::GeometriesType::Polyline)
        {
            return builder.CreateString(encodePolyline<100000>(begin, end));
        }
        else if (parameters.geometries == RouteParameters::GeometriesType::Polyline6)
        {
            return builder.CreateString(encodePolyline<1000000>(begin, end));
        }
        std::vector<fbresult::Position> coordinates;
        coordinates.resize(std::distance(begin, end));
        std::transform(begin, end, coordinates.begin(), [](const Coordinate &c) {
          return fbresult::Position{static_cast<float>(util::toFloating(c.lon).__value),
                                    static_cast<float>(util::toFloating(c.lat).__value)};
        });
        return builder.CreateVectorOfStructs(coordinates);
    }

    template <typename Builder> class GeometryVisitor
    {
      public:
        GeometryVisitor(Builder &builder) : builder(builder) {}

        void operator()(const flatbuffers::Offset<flatbuffers::String> &value)
        {
            builder.add_polyline(value);
        }
        void operator()(
            const flatbuffers::Offset<flatbuffers::Vector<const fbresult::Position *>> &value)
        {
            builder.add_coordinates(value);
        }

      private:
        Builder &builder;
    };

    virtual void
    MakeResponse(const PhantomNode &source,
                 const std::vector<util::Coordinate> &points,
                 util::json::Object &response) const
    {
        if (!parameters.skip_waypoints)
            response.values["source"] = MakeWaypoint(source);

        const std::vector<util::Coordinate> &geometry = parameters.overview == RouteParameters::OverviewType::Simplified ? guidance::reduceOverview_cgal(points,(100-parameters.convexity_value) *180/100) : points ;
        boost::optional<util::json::Value> json_geometry;
        switch( parameters.geometries ) {
        case RouteParameters::GeometriesType::Polyline :
            json_geometry = json::makePolyline<100000>(geometry.begin(), geometry.end());
            break ;
        case RouteParameters::GeometriesType::Polyline6 :
            json_geometry = json::makePolyline<1000000>(geometry.begin(), geometry.end());
            break ;
        case RouteParameters::GeometriesType::GeoJSON :
            json_geometry = json::makeGeoJSONGeometry(geometry.begin(), geometry.end());
            break ;
        }

        util::json::Object properties;
        if (json_geometry)
        {
            properties.values["geometry"] = *std::move(json_geometry);
        }
        switch( parameters.optimize ) {
        case IsochroneParameters::OptimizeType::Distance :
            properties.values["distance"] = parameters.range ;
            break ;
        case IsochroneParameters::OptimizeType::Time :
            properties.values["duration"] = parameters.range ;
            break ;
        default :
            properties.values["weight"] = parameters.range ;
            break ;
        }
        response.values["isoline"] = properties;
        response.values["code"] = "Ok";
        auto data_timestamp = facade.GetTimestamp();
        if (!data_timestamp.empty())
        {
            response.values["data_version"] = data_timestamp;
        }
    }

  protected:
    const IsochroneParameters &parameters;
};

} // namespace api
} // namespace engine
} // namespace osrm

#endif // OSRM_ISOCHRONE_API_HPP
