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
    BOOST_ASSERT(parameters.IsValid());
    if (parameters.coordinates.size() > 1)
    {
        return Error("InvalidOptions", "Only one input coordinate is supported", result);
    }
    if (!CheckAllCoordinates(parameters.coordinates))
        return Error("InvalidOptions", "Coordinates are invalid", result);
    if (!CheckAlgorithms(parameters, algorithms, result))
        return Status::Error;

    if (!algorithms.HasDirectShortestPathSearch() && !algorithms.HasShortestPathSearch())
    {
        return Error(
            "NotImplemented",
            "Direct shortest path search is not implemented for the chosen search algorithm.",
            result);
    }

    util::Coordinate startcoord = parameters.coordinates.front();
    const auto latitude_range = parameters.range /
                                (util::coordinate_calculation::detail::EARTH_RADIUS * M_PI * 2) *
                                360;

    const auto longitude_range = 360 * parameters.range /
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
    auto phantomWeights = [&parameters](const PhantomNode &phantom, bool forward) {
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
    //auto weightName = ( parameters.optimize==osrm::engine::api::BaseParameters::OptimizeType::Distance ? "distance" :
    //                    ( parameters.optimize==osrm::engine::api::BaseParameters::OptimizeType::Time ? "duration" : (const char*)0 ) );

    api::RouteAPI route_api{facade, parameters};
    InternalManyRoutesResult
        routes = algorithms.ForwardIsochroneSearch(
        start_end_nodes.front(), phantomWeights, parameters.optimize, parameters.range);

    if (routes.routes[0].is_valid())
    {
        auto collapse_legs = !parameters.waypoints.empty();
        if (collapse_legs)
        {
            std::vector<bool> waypoint_legs(parameters.coordinates.size(), false);
            std::for_each(parameters.waypoints.begin(),
                          parameters.waypoints.end(),
                          [&](const std::size_t waypoint_index) {
                            BOOST_ASSERT(waypoint_index < waypoint_legs.size());
                            waypoint_legs[waypoint_index] = true;
                          });
            // First and last coordinates should always be waypoints
            // This gets validated earlier, but double-checking here, jic
            BOOST_ASSERT(waypoint_legs.front());
            BOOST_ASSERT(waypoint_legs.back());
            for (std::size_t i = 0; i < routes.routes.size(); i++)
            {
                routes.routes[i] = CollapseInternalRouteResult(routes.routes[i], waypoint_legs);
            }
        }

        route_api.MakeResponse(routes, start_end_nodes, "weightName", result);
    }
    return Status::Ok;
}

/*
Status
IsochronePlugin::HandleRequest__(const RoutingAlgorithmsInterface &algorithms,
                               const api::IsochroneParameters &parameters,
                               osrm::engine::api::ResultT &result) const
    BOOST_ASSERT(parameters.IsValid());
    //std::vector<double> radiuses;
    //radiuses.push_back(parameters.range) ;
    //std::vector<std::vector<PhantomNodeWithDistance>> phantoms =GetPhantomNodesInRange(algorithms.GetFacade(), parameters, radiuses);
    const auto range = parameters.range;
    //const auto range = 300;

    util::Coordinate startcoord = parameters.coordinates.front();

    //const auto MAX_SPEED_METERS_PER_SECOND = 90 / 3.6;
    //const auto MAX_TRAVEL_DISTANCE_METERS = MAX_SPEED_METERS_PER_SECOND * range;

    const auto latitude_range = range /
                                (util::coordinate_calculation::detail::EARTH_RADIUS * M_PI * 2) *
                                360;

    const auto longitude_range = 360 * range /
                                 ((util::coordinate_calculation::detail::EARTH_RADIUS * M_PI * 2) *
                                  cos(static_cast<double>(toFloating(startcoord.lat)) *
                                      util::coordinate_calculation::detail::DEGREE_TO_RAD));

    util::Coordinate southwest{
        util::FloatLongitude{
            static_cast<double>(static_cast<double>(toFloating(startcoord.lon)) - longitude_range)},
        util::FloatLatitude{
            static_cast<double>(static_cast<double>(toFloating(startcoord.lat)) - latitude_range)}};
    util::Coordinate northeast{
        util::FloatLongitude{
            static_cast<double>(static_cast<double>(toFloating(startcoord.lon)) + longitude_range)},
        util::FloatLatitude{
            static_cast<double>(static_cast<double>(toFloating(startcoord.lat)) + latitude_range)}};

    util::Log() << "sw " << southwest.lat << "," << southwest.lon;
    util::Log() << "ne " << northeast.lat << "," << northeast.lon;

    TIMER_START(GET_EDGES_TIMER);
    const auto &facade = algorithms.GetFacade();
    auto edges = facade.GetEdgesInBox(southwest, northeast);
    TIMER_STOP(GET_EDGES_TIMER);
    util::Log() << "Fetch RTree " << TIMER_MSEC(GET_EDGES_TIMER);

    TIMER_START(PHANTOM_TIMER);
    // Convert edges to phantom nodes
    auto phantoms = GetPhantomNodes(facade, parameters, 1) ;
    std::vector<PhantomNode> phantom_nodes ;
    std::transform( phantoms.front().begin(), phantoms.front().end(), std::back_inserter(phantom_nodes), [](auto phantom){ return phantom.phantom_node;} );
    if (phantom_nodes.size() == 0)
    {
        return Error("NoSegment", "Could not find a matching segments for coordinate", result);
    }
    BOOST_ASSERT(phantom_nodes.size() > 0);
    //auto startpoints = facade.NearestPhantomNodes(startcoord, 1, approach);
    //std::vector<PhantomNode> phantoms;
    //phantoms.push_back(phantom_nodes.front().front().phantom_node);

    std::unordered_set<unsigned> seen_roads;

    // For every road, add two phantom nodes (one for each travel direction),
    // located at the "end" of the road.  When the distance to a node
    // comes back from the one-to-many search and the distance exceeds our range,
    // we will fetch the geometry for that road, and step backwards until we
    // are inside our range.  This needs to be done for both travel directions
    // on a road to get the full isochrone.
    std::for_each(edges.begin(), edges.end(), [&](const auto &data) {

      const auto geometry_id = facade.GetGeometryIndex(data.forward_segment_id.id).id;
      if (seen_roads.count(geometry_id) != 0)
          return;

      seen_roads.insert(geometry_id);

      EdgeWeight forward_weight_offset = 0, forward_weight = 0;
      EdgeWeight reverse_weight_offset = 0, reverse_weight = 0;
      EdgeDuration forward_duration_offset = 0, forward_duration = 0;
      EdgeDuration reverse_duration_offset = 0, reverse_duration = 0;
      EdgeDistance forward_distance_offset = 0, forward_distance = 0;
      EdgeDistance reverse_distance_offset = 0, reverse_distance = 0;

      const auto forward_weight_vector =
          facade.GetUncompressedForwardWeights(geometry_id);
      const auto reverse_weight_vector =
          facade.GetUncompressedReverseWeights(geometry_id);
      const auto forward_duration_vector =
          facade.GetUncompressedForwardDurations(geometry_id);
      const auto reverse_duration_vector =
          facade.GetUncompressedReverseDurations(geometry_id);
      //const auto forward_distance_vector =
      //    facade.GetUncompressedForwardDurations(geometry_id);
      //const auto reverse_distance_vector =
      //    facade.GetUncompressedReverseDurations(geometry_id);

      forward_weight_offset =
          std::accumulate(forward_weight_vector.begin(), forward_weight_vector.end(), 0);
      forward_duration_offset =
          std::accumulate(forward_duration_vector.begin(), forward_duration_vector.end(), 0);

      forward_weight = forward_weight_vector[data.fwd_segment_position];
      forward_duration = forward_duration_vector[data.fwd_segment_position];

      BOOST_ASSERT(data.fwd_segment_position < reverse_weight_vector.size());

      reverse_weight_offset =
          std::accumulate(reverse_weight_vector.begin(), reverse_weight_vector.end(), 0);
      reverse_duration_offset =
          std::accumulate(reverse_duration_vector.begin(), reverse_duration_vector.end(), 0);
      //forward_distance

      reverse_weight =
          reverse_weight_vector[reverse_weight_vector.size() - data.fwd_segment_position - 1];
      reverse_duration =
          reverse_duration_vector[reverse_duration_vector.size() - data.fwd_segment_position - 1];

      auto vcoord = facade.GetCoordinateOfNode(data.v);
      auto ucoord = facade.GetCoordinateOfNode(data.u);

      if (data.forward_segment_id.enabled)
      {
          // We fetched a rectangle from the RTree - the corners will be out of range.
          // This filters those out.
          if (util::coordinate_calculation::haversineDistance(ucoord, startcoord) > range)
              return;
          auto datacopy = data;
          datacopy.reverse_segment_id.enabled = false;
          const auto component_id = facade.GetComponentID(data.forward_segment_id.id);
          phantom_nodes.emplace_back(datacopy,
                                     component_id,
                                     forward_weight,
                                     reverse_weight,
                                     forward_weight_offset,
                                     reverse_weight_offset,
                                     forward_distance,
                                     reverse_distance,
                                     forward_distance_offset,
                                     reverse_distance_offset,
                                     forward_duration,
                                     reverse_duration,
                                     forward_duration_offset,
                                     reverse_duration_offset,
                                     true,//is_forward_valid_source,
                                     true,// is_forward_valid_target,
                                     true,// is_reverse_valid_source,
                                     true,//is_reverse_valid_target,
                                     vcoord,
                                     vcoord,
                                     0);
      }
      if (data.reverse_segment_id.enabled)
      {
          if (util::coordinate_calculation::haversineDistance(vcoord, startcoord) > range)
              return;
          auto datacopy = data;
          datacopy.forward_segment_id.enabled = false;
          const auto component_id = facade.GetComponentID(data.reverse_segment_id.id);
          phantom_nodes.emplace_back(datacopy,
                                     component_id,
                                     forward_weight,
                                     reverse_weight,
                                     forward_weight_offset,
                                     reverse_weight_offset,
                                     forward_distance,
                                     reverse_distance,
                                     forward_distance_offset,
                                     reverse_distance_offset,
                                     forward_duration,
                                     reverse_duration,
                                     forward_duration_offset,
                                     reverse_duration_offset,
                                     true,//is_forward_valid_source,
                                     true,// is_forward_valid_target,
                                     true,// is_reverse_valid_source,
                                     true,//is_reverse_valid_target,
                                     vcoord,
                                     vcoord,
                                     0);
      }
    });

    std::vector<std::size_t> sources;
    std::vector<std::size_t> destinations;

    sources.push_back(0);
    destinations.resize(phantom_nodes.size() - 1);
    std::iota(destinations.begin(), destinations.end(), 1);

    TIMER_STOP(PHANTOM_TIMER);
    util::Log() << "Make phantom nodes " << TIMER_MSEC(PHANTOM_TIMER);

    api::IsochroneAPI isochrone_api{facade, parameters};
    isochrone_api.MakeResponse(algorithms.IsochroneNodes(phantom_nodes,parameters),result);

    return Status::Ok;
}
*/
} // plugins
} // engine
} // osrm
