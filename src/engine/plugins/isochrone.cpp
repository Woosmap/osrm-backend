#include "engine/plugins/isochrone.hpp"
#include "engine/api/isochrone_api.hpp"
#include "engine/plugins/plugin_base.hpp"
#include "util/timing_util.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/log.hpp"

#include "extractor/edge_based_node.hpp"

#include <queue>
#include <iomanip>
#include <cmath>
#include <unordered_set>
#include <numeric>

namespace osrm
{
namespace engine
{
namespace plugins
{

//IsochronePlugin::IsochronePlugin() {}

Status
IsochronePlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                               const api::IsochroneParameters &parameters,
                               osrm::engine::api::ResultT &result) const
{
    if (!algorithms.HasIsochroneSearch())
    {
        return Error("NotImplemented",
                     "Isochrone path search is not implemented for the chosen search algorithm. ",
                     result);
    }
    if (!algorithms.HasDirectShortestPathSearch())
    {
        return Error(
            "NotImplemented",
            "Direct shortest path search is not implemented for the chosen search algorithm.",
            result);
    }
    const auto notValid = parameters.IsValid(true) ;
    if( notValid )
    {
        return Error("InvalidOptions", notValid.value(), result);
    }
    if (!CheckAllCoordinates(parameters.coordinates))
        return Error("InvalidOptions", "Coordinates are invalid", result);
    if (!CheckAlgorithms(parameters, algorithms, result))
        return Status::Error;

    util::Coordinate startcoord = parameters.coordinates.front();

    const auto MAX_SPEED_METERS_PER_SECOND = 90.0 / 3.6;
    const auto MAX_TRAVEL_DISTANCE_METERS = (parameters.optimize==api::IsochroneParameters::OptimizeType::Distance ?
                                             parameters.max_weight : MAX_SPEED_METERS_PER_SECOND * parameters.max_weight );
    //  Range is in meters if optimizing distance or in seconds if optimizing duration (set to 1/10 seconds)
    auto range = (parameters.optimize==api::IsochroneParameters::OptimizeType::Distance ? parameters.max_weight : parameters.max_weight * 10);
    const auto latitude_range = MAX_TRAVEL_DISTANCE_METERS /
                                (util::coordinate_calculation::detail::EARTH_RADIUS * M_PI * 2) *
                                360;

    const auto longitude_range = 360 * MAX_TRAVEL_DISTANCE_METERS /
                                 ((util::coordinate_calculation::detail::EARTH_RADIUS * M_PI * 2) *
                                  cos(static_cast<double>(toFloating(startcoord.lat)) *
                                      util::coordinate_calculation::detail::DEGREE_TO_RAD));

    util::Coordinate southwest{
        util::FloatLongitude{
            static_cast<double>(static_cast<double>(toFloating(startcoord.lon)) - longitude_range)},
        util::FloatLatitude{
            static_cast<double>(static_cast<double>(toFloating(startcoord.lat)) - latitude_range)}};

    std::vector<util::Coordinate> coordinates;
    api::IsochroneParameters params = parameters ;
    params.coordinates.push_back(southwest);
    const auto &facade = algorithms.GetFacade();
    auto phantom_node_pairs = GetPhantomNodes(facade, params);
    if (phantom_node_pairs.size() != params.coordinates.size())
    {
        return Error("NoSegment",
                     MissingPhantomErrorMessage(phantom_node_pairs, parameters.coordinates),
                     result);
    }
    BOOST_ASSERT(phantom_node_pairs.size() == params.coordinates.size());

    std::vector<PhantomNodes> start_end_nodes;
    start_end_nodes.push_back( PhantomNodes{phantom_node_pairs.front().first,phantom_node_pairs.back().first} );
    auto phantom_weights = [&parameters](const PhantomNode &phantom, bool forward) {
      switch( parameters.optimize) {
      case osrm::engine::api::BaseParameters::OptimizeType::Distance :
          return static_cast<EdgeWeight>( forward ? phantom.GetForwardDistance() : phantom.GetReverseDistance() );
          //break ;
      case osrm::engine::api::BaseParameters::OptimizeType::Time :
          return static_cast<EdgeWeight>( forward ? phantom.GetForwardDuration() : phantom.GetReverseDuration() );
          //break ;
      default :
          return PhantomNode::phantomWeights(phantom,forward);
          //break ;
      }
    };

    api::IsochroneAPI isochrone_api{facade, parameters};

    api::RouteAPI route_api{facade, parameters};
    std::vector<util::Coordinate>
        iso_nodes = algorithms.ForwardIsochroneSearch(
        start_end_nodes.front(), phantom_weights, parameters.optimize, range,range/5);

    iso_nodes.push_back( start_end_nodes.front().source_phantom.location ) ;
    isochrone_api.MakeResponse(start_end_nodes.front().source_phantom, iso_nodes, result);
    return Status::Ok;
}

} // plugins
} // engine
} // osrm
