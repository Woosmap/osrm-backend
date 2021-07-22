#include "engine/api/base_parameters.hpp"
#include "engine/routing_algorithms/direct_shortest_path.hpp"
#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"
#include "engine/polyline_compressor.hpp"
#include <math.h>       /* cos, sin, atan2 */

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

/// This is a stripped down version of the general shortest path algorithm.
/// The general algorithm always computes two queries for each leg. This is only
/// necessary in case of vias, where the directions of the start node is constrained
/// by the previous route.
/// This variation is only an optimization for graphs with slow queries, for example
/// not fully contracted graphs.
template <>
InternalRouteResult directShortestPathSearch(SearchEngineData<ch::Algorithm> &engine_working_data,
                                             const DataFacade<ch::Algorithm> &facade,
                                             const PhantomNodes &phantom_nodes,
                                             std::function<EdgeWeight(const PhantomNode &, bool)> /*phantom_weights*/,
                                             osrm::engine::api::BaseParameters::OptimizeType optimize)
{
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());
    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;
    forward_heap.Clear();
    reverse_heap.Clear();

    EdgeWeight weight = INVALID_EDGE_WEIGHT;
    std::vector<NodeID> packed_leg;
    insertNodesInHeaps(forward_heap, reverse_heap, phantom_nodes);

    search(engine_working_data,
           facade,
           forward_heap,
           reverse_heap,
           getWeightStrategy(facade,optimize),
           weight,
           packed_leg,
           DO_NOT_FORCE_LOOPS,
           DO_NOT_FORCE_LOOPS,
           phantom_nodes);

    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;

    if (!packed_leg.empty())
    {
        unpacked_nodes.reserve(packed_leg.size());
        unpacked_edges.reserve(packed_leg.size());
        unpacked_nodes.push_back(packed_leg.front());
        ch::unpackPath(facade,
                       packed_leg.begin(),
                       packed_leg.end(),
                       [&unpacked_nodes, &unpacked_edges](std::pair<NodeID, NodeID> &edge,
                                                          const auto &edge_id) {
                           BOOST_ASSERT(edge.first == unpacked_nodes.back());
                           unpacked_nodes.push_back(edge.second);
                           unpacked_edges.push_back(edge_id);
                       });
    }

    return extractRoute(facade, weight, phantom_nodes, unpacked_nodes, unpacked_edges);
}

template <>
std::vector<util::Coordinate>
forwardIsochroneSearch(SearchEngineData<ch::Algorithm> &/*engine_working_data*/,
                       const DataFacade<ch::Algorithm> &/*facade*/,
                       const PhantomNodes &/*phantom_nodes*/,
                       std::function<EdgeWeight(const PhantomNode &, bool)> /*phantomWeights*/,
                       osrm::engine::api::BaseParameters::OptimizeType /*optimize*/,
                       EdgeWeight /*max_weight*/,
                       EdgeWeight /*min_weight*/)
{
    std::vector<util::Coordinate> raw_data;
    return raw_data;
}


template <>
InternalRouteResult directShortestPathSearch(SearchEngineData<mld::Algorithm> &engine_working_data,
                                             const DataFacade<mld::Algorithm> &facade,
                                             const PhantomNodes &phantom_nodes,
                                             std::function<EdgeWeight(const PhantomNode &, bool)> phantom_weights,
                                             osrm::engine::api::BaseParameters::OptimizeType optimize)
{
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes(),
                                                                 facade.GetMaxBorderNodeID() + 1);
    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;
    insertNodesInHeaps(forward_heap, reverse_heap, phantom_nodes,phantom_weights);

    // TODO: when structured bindings will be allowed change to
    // auto [weight, source_node, target_node, unpacked_edges] = ...
    EdgeWeight weight = INVALID_EDGE_WEIGHT;
    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;
    std::tie(weight, unpacked_nodes, unpacked_edges, std::ignore) = mld::search(engine_working_data,
                                                                   facade,
                                                                   forward_heap,
                                                                   reverse_heap,
                                                                   getWeightStrategy(facade,optimize),
                                                                   DO_NOT_FORCE_LOOPS,
                                                                   DO_NOT_FORCE_LOOPS,
                                                                   INVALID_EDGE_WEIGHT,
                                                                   0);

    return extractRoute(facade, weight, phantom_nodes, unpacked_nodes, unpacked_edges);
}


template <>
std::vector<util::Coordinate>
forwardIsochroneSearch(SearchEngineData<mld::Algorithm> &engine_working_data,
                       const DataFacade<mld::Algorithm> &facade,
                       const PhantomNodes &phantom_nodes,
                       std::function<EdgeWeight(const PhantomNode &, bool)> phantomWeights,
                       osrm::engine::api::BaseParameters::OptimizeType optimize,
                       EdgeWeight max_weight,
                       EdgeWeight min_weight)
{
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes(),
                                                                 facade.GetMaxBorderNodeID() + 1);
    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;

    const auto &source = phantom_nodes.source_phantom;
    if (source.IsValidForwardSource())
    {
        forward_heap.Insert(source.forward_segment_id.id,
                            -phantomWeights(source,true),
                            source.forward_segment_id.id);
    }
    if (source.IsValidReverseSource())
    {
        forward_heap.Insert(source.reverse_segment_id.id,
                            -phantomWeights(source,false),
                            source.reverse_segment_id.id);
    }

    const auto &target = phantom_nodes.target_phantom;
    if (target.IsValidForwardTarget())
    {
        reverse_heap.Insert(target.forward_segment_id.id,
                            INVALID_EDGE_WEIGHT,
                            target.forward_segment_id.id);
    }

    if (target.IsValidReverseTarget())
    {
        reverse_heap.Insert(target.reverse_segment_id.id,
                            INVALID_EDGE_WEIGHT,
                            target.reverse_segment_id.id);
    }
    //insertNodesInHeaps(forward_heap, reverse_heap, phantom_nodes,phantomWeights);

    // TODO: when structured bindings will be allowed change to
    // auto [weight, source_node, target_node, unpacked_edges] = ...
    EdgeWeight weight = max_weight;
    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeWeight> unpacked_weights;
    std::tie(weight, unpacked_nodes, std::ignore, unpacked_weights) = mld::search(engine_working_data,
                                                                   facade,
                                                                   forward_heap,
                                                                   reverse_heap,
                                                                   getWeightStrategy(facade,optimize),
                                                                   DO_NOT_FORCE_LOOPS,
                                                                   DO_NOT_FORCE_LOOPS,
                                                                   max_weight,
                                                                   0);
    BOOST_ASSERT( unpacked_nodes.size() == unpacked_weights.size() ) ;
    std::vector<util::Coordinate> coords ;
    auto node_weight = unpacked_weights.begin() ;
    util::UnbufferedLog log(LogLevel::logDEBUG);
    log << "Skipping ... " ;
    std::for_each( unpacked_nodes.begin(), unpacked_nodes.end(), [&](auto node){
      const auto geometry_index = facade.GetGeometryIndex(node);
      const auto copy = [](auto &vector, const auto range) {
        vector.resize(range.size());
        std::copy(range.begin(), range.end(), vector.begin());
      };
      if( *node_weight>=min_weight ){
          std::vector<NodeID> id_vector;
          copy(id_vector, facade.GetUncompressedForwardGeometry(geometry_index.id));
          coords.push_back(facade.GetCoordinateOfNode(id_vector[1]));
          log << '(' << coords.back().lat << ',' << coords.back().lon << '+' << *node_weight << ")," ;
      }
      else
      {
          log << *node_weight << ',' ;
      }
      node_weight++ ;
    });
    //  Sort according to bearing of vector (source -> point)
    std::sort( coords.begin(), coords.end(), [&phantom_nodes](util::Coordinate &ca, util::Coordinate &cb){
      util::Coordinate source = phantom_nodes.source_phantom.location ;
      auto x_a = cos(static_cast<float>(util::toFloating(ca.lat).__value)) * sin(static_cast<float>(util::toFloating(ca.lon-source.lon).__value));
      auto y_a = cos(static_cast<float>(util::toFloating(ca.lat).__value)) * sin(static_cast<float>(util::toFloating(source.lat).__value)) -
               sin(static_cast<float>(util::toFloating(ca.lat).__value)) * cos(static_cast<float>(util::toFloating(source.lat).__value)) * cos(static_cast<float>(util::toFloating(ca.lon-source.lon).__value)) ;
      auto beta_a = atan2(x_a,y_a) ;
      auto x_b = cos(static_cast<float>(util::toFloating(cb.lat).__value)) * sin(static_cast<float>(util::toFloating(cb.lon-source.lon).__value));
      auto y_b = cos(static_cast<float>(util::toFloating(cb.lat).__value)) * sin(static_cast<float>(util::toFloating(source.lat).__value)) -
                 sin(static_cast<float>(util::toFloating(cb.lat).__value)) * cos(static_cast<float>(util::toFloating(source.lat).__value)) * cos(static_cast<float>(util::toFloating(cb.lon-source.lon).__value)) ;
      auto beta_b = atan2(x_b,y_b) ;
      return beta_a<beta_b ;
    });
    if( !coords.empty())
        coords.push_back( coords.front() ) ;

    return coords ;
}
} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
