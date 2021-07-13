#include "engine/routing_algorithms/isochrone_nodes.hpp"

#include "engine/routing_algorithms/routing_base_ch.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"

#include "util/timing_util.hpp"
#include <stdlib.h>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

template <>
std::vector<PhantomNode> isochroneNodes(SearchEngineData<ch::Algorithm> &engine_working_data,
                                             const DataFacade<ch::Algorithm> &facade,
                                             std::vector<PhantomNode> &phantom_nodes,
                                        const api::IsochroneParameters &parameters)
{
    const auto range = parameters.range;

    // Phase 1 - outgoing, upwards dijkstra search, setting d(v) for all v we visit
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());
    engine_working_data.InitializeOrClearSecondThreadLocalStorage(facade.GetNumberOfNodes());
    auto &query_heap = *(engine_working_data.forward_heap_1);
    if (phantom_nodes[0].forward_segment_id.enabled)
    {
        query_heap.Insert(phantom_nodes[0].forward_segment_id.id,
                          -phantom_nodes[0].GetForwardWeightPlusOffset(),
                          phantom_nodes[0].forward_segment_id.id);
    }
    if (phantom_nodes[0].reverse_segment_id.enabled)
    {
        query_heap.Insert(phantom_nodes[0].reverse_segment_id.id,
                          -phantom_nodes[0].GetReverseWeightPlusOffset(),
                          phantom_nodes[0].reverse_segment_id.id);
    }

    while (query_heap.Size() > 0)
    {
        const NodeID node = query_heap.DeleteMin();
        const EdgeWeight weight = query_heap.GetKey(node);

        for (auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (data.forward)
            {
                const NodeID to = facade.GetTarget(edge);
                const EdgeWeight edge_weight = data.weight;

                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                const EdgeWeight to_weight = weight + edge_weight;

                // No need to search further than our maximum radius
                if (weight > range * 10)
                    continue;

                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    query_heap.Insert(to, to_weight, node);
                }
                    // Found a shorter Path -> Update weight
                else if (to_weight < query_heap.GetKey(to))
                {
                    // new parent
                    query_heap.GetData(to) = node;
                    query_heap.DecreaseKey(to, to_weight);
                }
            }
        }
    }

    // Phase 2 - scan nodes in descending CH order, updating d(v) where we can
    TIMER_START(TABLE_TIMER);
    std::sort(phantom_nodes.begin(), phantom_nodes.end(), [](const auto &a, const auto &b) {
      const auto &a_id =
          a.forward_segment_id.enabled ? a.forward_segment_id.id : a.reverse_segment_id.id;
      const auto &b_id =
          b.forward_segment_id.enabled ? b.forward_segment_id.id : b.reverse_segment_id.id;

      return a_id < b_id;
    });

    // Now, scan over all the phantoms in reverse CH order, updating edge weights as we go
    for (const auto &p : phantom_nodes)
    {
        const auto node =
            p.forward_segment_id.enabled ? p.forward_segment_id.id : p.reverse_segment_id.id;
        for (const auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (data.backward)
            {
                const NodeID to = facade.GetTarget(edge);

                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    // TODO: hmm, what does this mean? shouldn't be possible in a correct CH
                    // util::Log() << "WARNING: to not found in reverse scan";
                }
                    // Found a shorter Path -> Update weight
                else
                {
                    const EdgeWeight weight = query_heap.GetKey(to);
                    const EdgeWeight edge_weight = data.weight;
                    const EdgeWeight to_weight = weight + edge_weight;

                    if (!query_heap.WasInserted(node))
                    {
                        query_heap.Insert(node, to_weight, to);
                    }
                    else if (to_weight < query_heap.GetKey(to))
                    {
                        // new parent
                        query_heap.GetData(node) = to;
                        query_heap.DecreaseKey(node, to_weight);
                    }
                }
            }
        }
    }

    TIMER_STOP(TABLE_TIMER);
    util::Log() << "Distance table " << TIMER_MSEC(TABLE_TIMER);

    std::clog << "Phantoms size: " << phantom_nodes.size() << std::endl;

    std::vector<PhantomNode> ret ;
    for (const auto &phantom : phantom_nodes)
    {
        const auto node = phantom.forward_segment_id.enabled ? phantom.forward_segment_id.id
                                                             : phantom.reverse_segment_id.id;
        if (!query_heap.WasInserted(node))
            continue;
        const auto weight = query_heap.GetKey(node);

        // If INVALID_EDGE_WEIGHT, it means that the phantom couldn't be reached
        // from the isochrone centerpoint
        if (weight == INVALID_EDGE_WEIGHT)
            continue;

        // If the weight is > the max range, it means that this edge crosses
        // the range, but the start of it is inside the range. We need
        // to chop this line somewhere.
        // TODO: actually do that.
        if (weight > range * 10)
            continue;

        ret.push_back( phantom );
    }
    return ret;
}

template <>
std::vector<PhantomNode> isochroneNodes(SearchEngineData<mld::Algorithm> &engine_working_data,
                                            const DataFacade<mld::Algorithm> &facade,
                                            std::vector<PhantomNode> &phantom_nodes,
                                        const api::IsochroneParameters &parameters)
{
    const auto range = parameters.range;

    // Phase 1 - outgoing, upwards dijkstra search, setting d(v) for all v we visit
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes(),facade.GetMaxBorderNodeID() + 1);
    //engine_working_data.InitializeOrClearSecondThreadLocalStorage(facade.GetNumberOfNodes());
    auto &query_heap = *(engine_working_data.forward_heap_1);
    if (phantom_nodes[0].forward_segment_id.enabled)
    {
        query_heap.Insert(phantom_nodes[0].forward_segment_id.id,
                          -phantom_nodes[0].GetForwardWeightPlusOffset(),
                          phantom_nodes[0].forward_segment_id.id);
    }
    if (phantom_nodes[0].reverse_segment_id.enabled)
    {
        query_heap.Insert(phantom_nodes[0].reverse_segment_id.id,
                          -phantom_nodes[0].GetReverseWeightPlusOffset(),
                          phantom_nodes[0].reverse_segment_id.id);
    }

    while (query_heap.Size() > 0)
    {
        const NodeID node = query_heap.DeleteMin();
        const EdgeWeight weight = query_heap.GetKey(node);

        for (auto edge : facade.GetAdjacentEdgeRange(node))
        {
            //const auto &data = facade.GetEdgeData(edge);
            if (facade.IsForwardEdge(edge))
            {
                const NodeID to = facade.GetTarget(edge);
                const EdgeWeight edge_weight = facade.GetNodeWeight(to);

                BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");
                const EdgeWeight to_weight = weight + edge_weight;

                // No need to search further than our maximum radius
                if (weight > range * 10)
                    continue;

                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    query_heap.Insert(to, to_weight, node);
                }
                    // Found a shorter Path -> Update weight
                else if (to_weight < query_heap.GetKey(to))
                {
                    // new parent
                    query_heap.GetData(to) = node;
                    query_heap.DecreaseKey(to, to_weight);
                }
            }
        }
    }

    // Phase 2 - scan nodes in descending CH order, updating d(v) where we can
    TIMER_START(TABLE_TIMER);
    std::sort(phantom_nodes.begin(), phantom_nodes.end(), [](const auto &a, const auto &b) {
      const auto &a_id =
          a.forward_segment_id.enabled ? a.forward_segment_id.id : a.reverse_segment_id.id;
      const auto &b_id =
          b.forward_segment_id.enabled ? b.forward_segment_id.id : b.reverse_segment_id.id;

      return a_id < b_id;
    });

    // Now, scan over all the phantoms in reverse CH order, updating edge weights as we go
    for (const auto &p : phantom_nodes)
    {
        const auto node =
            p.forward_segment_id.enabled ? p.forward_segment_id.id : p.reverse_segment_id.id;
        for (const auto edge : facade.GetAdjacentEdgeRange(node))
        {
            //const auto &data = facade.GetEdgeData(edge);
            if (facade.IsBackwardEdge(edge) )
            {
                const NodeID to = facade.GetTarget(edge);

                // New Node discovered -> Add to Heap + Node Info Storage
                if (!query_heap.WasInserted(to))
                {
                    // TODO: hmm, what does this mean? shouldn't be possible in a correct CH
                    // util::Log() << "WARNING: to not found in reverse scan";
                }
                    // Found a shorter Path -> Update weight
                else
                {
                    const EdgeWeight weight = query_heap.GetKey(to);
                    const EdgeWeight edge_weight = facade.GetNodeWeight(to);
                    const EdgeWeight to_weight = weight + edge_weight;

                    if (!query_heap.WasInserted(node))
                    {
                        query_heap.Insert(node, to_weight, to);
                    }
                    else if (to_weight < query_heap.GetKey(to))
                    {
                        // new parent
                        query_heap.GetData(node) = to;
                        query_heap.DecreaseKey(node, to_weight);
                    }
                }
            }
        }
    }

    TIMER_STOP(TABLE_TIMER);
    util::Log() << "Distance table " << TIMER_MSEC(TABLE_TIMER);

    std::clog << "Phantoms size: " << phantom_nodes.size() << std::endl;

    std::vector<PhantomNode> ret ;
    for (const auto &phantom : phantom_nodes)
    {
        const auto node = phantom.forward_segment_id.enabled ? phantom.forward_segment_id.id
                                                             : phantom.reverse_segment_id.id;
        if (!query_heap.WasInserted(node))
            continue;
        const auto weight = query_heap.GetKey(node);

        // If INVALID_EDGE_WEIGHT, it means that the phantom couldn't be reached
        // from the isochrone centerpoint
        if (weight == INVALID_EDGE_WEIGHT)
            continue;

        // If the weight is > the max range, it means that this edge crosses
        // the range, but the start of it is inside the range. We need
        // to chop this line somewhere.
        // TODO: actually do that.
        if (weight > range * 10)
            continue;

        ret.push_back( phantom );
    }
    //  Ordering waypoints :

    std::vector<PhantomNode> op ;
    op.push_back(ret.back());
    ret.pop_back();
    while( !ret.empty() )
    {
        auto &ref = op.back() ;
        auto closest = std::min_element(ret.begin(), ret.end(), [&](const auto &p1, const auto &p2) {
            return util::coordinate_calculation::haversineDistance(p1.location,ref.location) < util::coordinate_calculation::haversineDistance(p2.location, ref.location);
        });
        if(closest==ret.end())
            break ;
        op.push_back( *closest ) ;
        ret.erase(closest);
    }
    auto resolution = range/10 ;
    if( resolution>0 )
    {
        auto last = std::unique(op.begin(), op.end(), [&](const auto &p1, const auto &p2) {
            return util::coordinate_calculation::haversineDistance(p1.location, p2.location) <
                   resolution;
        });
        op.erase(last, op.end());
    }

    return op;
}


} // namespace routing_algorithms
} // namespace engine
} // namespace osrm
