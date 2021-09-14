#ifndef OSRM_ENGINE_ROUTING_BASE_MLD_HPP
#define OSRM_ENGINE_ROUTING_BASE_MLD_HPP

#include "engine/api/base_parameters.hpp"
#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/search_engine_data.hpp"

#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <tuple>
#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{
namespace mld
{

namespace
{
// Unrestricted search (Args is const PhantomNodes &):
//   * use partition.GetQueryLevel to find the node query level based on source and target phantoms
//   * allow to traverse all cells
template <typename MultiLevelPartition>
inline LevelID getNodeQueryLevel(const MultiLevelPartition &partition,
                                 NodeID node,
                                 const PhantomNodes &phantom_nodes)
{
    auto level = [&partition, node](const SegmentID &source, const SegmentID &target) {
        if (source.enabled && target.enabled)
            return partition.GetQueryLevel(source.id, target.id, node);
        return INVALID_LEVEL_ID;
    };
    return std::min(std::min(level(phantom_nodes.source_phantom.forward_segment_id,
                                   phantom_nodes.target_phantom.forward_segment_id),
                             level(phantom_nodes.source_phantom.forward_segment_id,
                                   phantom_nodes.target_phantom.reverse_segment_id)),
                    std::min(level(phantom_nodes.source_phantom.reverse_segment_id,
                                   phantom_nodes.target_phantom.forward_segment_id),
                             level(phantom_nodes.source_phantom.reverse_segment_id,
                                   phantom_nodes.target_phantom.reverse_segment_id)));
}

inline bool checkParentCellRestriction(CellID, const PhantomNodes &) { return true; }

// Restricted search (Args is simply LevelID):
//   * used for single level searchs
template <typename MultiLevelPartition>
inline LevelID getNodeQueryLevel(const MultiLevelPartition &, NodeID, LevelID level)
{
    return level;
}

inline bool checkParentCellRestriction(CellID, LevelID) { return true; }

// Restricted search (Args is LevelID, CellID):
//   * use the fixed level for queries
//   * check if the node cell is the same as the specified parent
template <typename MultiLevelPartition>
inline LevelID getNodeQueryLevel(const MultiLevelPartition &, NodeID, LevelID level, CellID)
{
    return level;
}

inline bool checkParentCellRestriction(CellID cell, LevelID, CellID parent)
{
    return cell == parent;
}

// Unrestricted search with a single phantom node (Args is const PhantomNode &):
//   * use partition.GetQueryLevel to find the node query level
//   * allow to traverse all cells
template <typename MultiLevelPartition>
inline LevelID getNodeQueryLevel(const MultiLevelPartition &partition,
                                 const NodeID node,
                                 const PhantomNode &phantom_node)
{
    auto highest_diffrent_level = [&partition, node](const SegmentID &phantom_node) {
        if (phantom_node.enabled)
            return partition.GetHighestDifferentLevel(phantom_node.id, node);
        return INVALID_LEVEL_ID;
    };

    const auto node_level = std::min(highest_diffrent_level(phantom_node.forward_segment_id),
                                     highest_diffrent_level(phantom_node.reverse_segment_id));

    return node_level;
}

// Unrestricted search with a single phantom node and a vector of phantom nodes:
//   * use partition.GetQueryLevel to find the node query level
//   * allow to traverse all cells
template <typename MultiLevelPartition>
inline LevelID getNodeQueryLevel(const MultiLevelPartition &partition,
                                 NodeID node,
                                 const std::vector<PhantomNode> &phantom_nodes,
                                 const std::size_t phantom_index,
                                 const std::vector<std::size_t> &phantom_indices)
{
    auto min_level = [&partition, node](const PhantomNode &phantom_node) {
        const auto &forward_segment = phantom_node.forward_segment_id;
        const auto forward_level =
            forward_segment.enabled ? partition.GetHighestDifferentLevel(node, forward_segment.id)
                                    : INVALID_LEVEL_ID;

        const auto &reverse_segment = phantom_node.reverse_segment_id;
        const auto reverse_level =
            reverse_segment.enabled ? partition.GetHighestDifferentLevel(node, reverse_segment.id)
                                    : INVALID_LEVEL_ID;

        return std::min(forward_level, reverse_level);
    };

    // Get minimum level over all phantoms of the highest different level with respect to node
    // This is equivalent to min_{∀ source, target} partition.GetQueryLevel(source, node, target)
    auto result = min_level(phantom_nodes[phantom_index]);
    for (const auto &index : phantom_indices)
    {
        result = std::min(result, min_level(phantom_nodes[index]));
    }
    return result;
}
} // namespace

// Heaps only record for each node its predecessor ("parent") on the shortest path.
// For re-constructing the actual path we need to trace back all parent "pointers".
// In contrast to the CH code MLD needs to know the edges (with clique arc property).

using PackedEdge = std::tuple</*from*/ NodeID, /*to*/ NodeID, /*from_clique_arc*/ bool>;
using PackedPath = std::vector<PackedEdge>;

template <bool DIRECTION, typename OutIter>
inline void retrievePackedPathFromSingleManyToManyHeap(
    const SearchEngineData<Algorithm>::ManyToManyQueryHeap &heap, const NodeID middle, OutIter out)
{

    NodeID current = middle;
    NodeID parent = heap.GetData(current).parent;

    while (current != parent)
    {
        const auto &data = heap.GetData(current);

        if (DIRECTION == FORWARD_DIRECTION)
        {
            *out = std::make_tuple(parent, current, data.from_clique_arc);
            ++out;
        }
        else if (DIRECTION == REVERSE_DIRECTION)
        {
            *out = std::make_tuple(current, parent, data.from_clique_arc);
            ++out;
        }

        current = parent;
        parent = heap.GetData(parent).parent;
    }
}

template <bool DIRECTION>
inline PackedPath retrievePackedPathFromSingleManyToManyHeap(
    const SearchEngineData<Algorithm>::ManyToManyQueryHeap &heap, const NodeID middle)
{

    PackedPath packed_path;
    retrievePackedPathFromSingleManyToManyHeap<DIRECTION>(
        heap, middle, std::back_inserter(packed_path));

    return packed_path;
}

template <bool DIRECTION, typename OutIter>
inline void retrievePackedPathFromSingleHeap(const SearchEngineData<Algorithm>::QueryHeap &heap,
                                             const NodeID middle,
                                             OutIter out)
{
    NodeID current = middle;
    NodeID parent = heap.GetData(current).parent;

    while (current != parent)
    {
        const auto &data = heap.GetData(current);

        if (DIRECTION == FORWARD_DIRECTION)
        {
            *out = std::make_tuple(parent, current, data.from_clique_arc);
            ++out;
        }
        else if (DIRECTION == REVERSE_DIRECTION)
        {
            *out = std::make_tuple(current, parent, data.from_clique_arc);
            ++out;
        }

        current = parent;
        parent = heap.GetData(parent).parent;
    }
}

template <bool DIRECTION>
inline PackedPath
retrievePackedPathFromSingleHeap(const SearchEngineData<Algorithm>::QueryHeap &heap,
                                 const NodeID middle)
{
    PackedPath packed_path;
    retrievePackedPathFromSingleHeap<DIRECTION>(heap, middle, std::back_inserter(packed_path));
    return packed_path;
}

// Trace path from middle to start in the forward search space (in reverse)
// and from middle to end in the reverse search space. Middle connects paths.

inline PackedPath
retrievePackedPathFromHeap(const SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                           const SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                           const NodeID middle)
{
    // Retrieve start -> middle. Is in reverse order since tracing back starts from middle.
    auto packed_path = retrievePackedPathFromSingleHeap<FORWARD_DIRECTION>(forward_heap, middle);
    std::reverse(begin(packed_path), end(packed_path));

    // Retrieve middle -> end. Is already in correct order, tracing starts from middle.
    auto into = std::back_inserter(packed_path);
    retrievePackedPathFromSingleHeap<REVERSE_DIRECTION>(reverse_heap, middle, into);

    return packed_path;
}

template <bool DIRECTION, typename Algorithm, typename... Args>
void relaxOutgoingEdges(const DataFacade<Algorithm> &facade,
                        typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                        const typename SearchEngineData<Algorithm>::QueryHeap::HeapNode &heapNode,
                        std::function<EdgeWeight(const EdgeID id, const EdgeID turnId)> to_node_weight,
                        Args... args)
{
    const auto &partition = facade.GetMultiLevelPartition();
    const auto &cells = facade.GetCellStorage();
    const auto &metric = facade.GetCellMetric();

    const auto level = getNodeQueryLevel(partition, heapNode.node, args...);

    if (level >= 1 && !heapNode.data.from_clique_arc)
    {
        if (DIRECTION == FORWARD_DIRECTION)
        {
            // Shortcuts in forward direction
            const auto &cell =
                cells.GetCell(metric, level, partition.GetCell(level, heapNode.node));
            auto destination = cell.GetDestinationNodes().begin();
            for (auto shortcut_weight : cell.GetOutWeight(heapNode.node))
            {
                BOOST_ASSERT(destination != cell.GetDestinationNodes().end());
                const NodeID to = *destination;

                if (shortcut_weight != INVALID_EDGE_WEIGHT && heapNode.node != to)
                {
                    const EdgeWeight to_weight = heapNode.weight + shortcut_weight;
                    BOOST_ASSERT(to_weight >= heapNode.weight);
                    const auto toHeapNode = forward_heap.GetHeapNodeIfWasInserted(to);
                    if (!toHeapNode)
                    {
                        forward_heap.Insert(to, to_weight, {heapNode.node, true});
                    }
                    else if (to_weight < toHeapNode->weight)
                    {
                        toHeapNode->data = {heapNode.node, true};
                        toHeapNode->weight = to_weight;
                        forward_heap.DecreaseKey(*toHeapNode);
                    }
                }
                ++destination;
            }
        }
        else
        {
            // Shortcuts in backward direction
            const auto &cell =
                cells.GetCell(metric, level, partition.GetCell(level, heapNode.node));
            auto source = cell.GetSourceNodes().begin();
            for (auto shortcut_weight : cell.GetInWeight(heapNode.node))
            {
                BOOST_ASSERT(source != cell.GetSourceNodes().end());
                const NodeID to = *source;

                if (shortcut_weight != INVALID_EDGE_WEIGHT && heapNode.node != to)
                {
                    const EdgeWeight to_weight = heapNode.weight + shortcut_weight;
                    BOOST_ASSERT(to_weight >= heapNode.weight);
                    const auto toHeapNode = forward_heap.GetHeapNodeIfWasInserted(to);
                    if (!toHeapNode)
                    {
                        forward_heap.Insert(to, to_weight, {heapNode.node, true});
                    }
                    else if (to_weight < toHeapNode->weight)
                    {
                        toHeapNode->data = {heapNode.node, true};
                        toHeapNode->weight = to_weight;
                        forward_heap.DecreaseKey(*toHeapNode);
                    }
                }
                ++source;
            }
        }
    }

    // Boundary edges
    for (const auto edge : facade.GetBorderEdgeRange(level, heapNode.node))
    {
        const auto &edge_data = facade.GetEdgeData(edge);

        if ((DIRECTION == FORWARD_DIRECTION) ? facade.IsForwardEdge(edge)
                                             : facade.IsBackwardEdge(edge))
        {
            const NodeID to = facade.GetTarget(edge);

            if (!facade.ExcludeNode(to) &&
                checkParentCellRestriction(partition.GetCell(level + 1, to), args...))
            {
                const auto node_weight = to_node_weight(DIRECTION == FORWARD_DIRECTION ? heapNode.node : to, edge_data.turn_id);

                // TODO: BOOST_ASSERT(edge_data.weight == node_weight + turn_penalty);

                const EdgeWeight to_weight = heapNode.weight + node_weight ;

                const auto toHeapNode = forward_heap.GetHeapNodeIfWasInserted(to);
                if (!toHeapNode)
                {
                    forward_heap.Insert(to, to_weight, {heapNode.node, false});
                }
                else if (to_weight < toHeapNode->weight)
                {
                    toHeapNode->data = {heapNode.node, false};
                    toHeapNode->weight = to_weight;
                    forward_heap.DecreaseKey(*toHeapNode);
                }
            }
        }
    }
}

template <bool DIRECTION, typename Algorithm, typename... Args>
void routingStep(const DataFacade<Algorithm> &facade,
                 typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                 typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                 std::function<EdgeWeight(const EdgeID id, const EdgeID turnId)> to_node_weight,
                 NodeID &middle_node,
                 EdgeWeight &path_upper_bound,
                 const bool force_loop_forward,
                 const bool force_loop_reverse,
                 Args... args)
{
    const auto heapNode = forward_heap.DeleteMinGetHeapNode();
    const auto weight = heapNode.weight;
    BOOST_ASSERT(!facade.ExcludeNode(heapNode.node));

    // Upper bound for the path source -> target with
    // weight(source -> node) = weight weight(to -> target) ≤ reverse_weight
    // is weight + reverse_weight
    // More tighter upper bound requires additional condition reverse_heap.WasRemoved(to)
    // with weight(to -> target) = reverse_weight and all weights ≥ 0
    const auto reverseHeapNode = reverse_heap.GetHeapNodeIfWasInserted(heapNode.node);
    if (reverseHeapNode)
    {
        auto reverse_weight = reverseHeapNode->weight;
        auto path_weight = weight + reverse_weight;

        // MLD uses loops forcing only to prune single node paths in forward and/or
        // backward direction (there is no need to force loops in MLD but in CH)
        if (!(force_loop_forward && heapNode.data.parent == heapNode.node) &&
            !(force_loop_reverse && reverseHeapNode->data.parent == heapNode.node) &&
            (path_weight >= 0) && (path_weight < path_upper_bound))
        {
            middle_node = heapNode.node;
            path_upper_bound = path_weight;
        }
    }

    // Relax outgoing edges from node
    relaxOutgoingEdges<DIRECTION>(facade, forward_heap, heapNode, to_node_weight, args...);
}

// With (s, middle, t) we trace back the paths middle -> s and middle -> t.
// This gives us a packed path (node ids) from the base graph around s and t,
// and overlay node ids otherwise. We then have to unpack the overlay clique
// edges by recursively descending unpacking the path down to the base graph.

using UnpackedNodes = std::vector<NodeID>;
using UnpackedEdges = std::vector<EdgeID>;
using UnpackedWeigths = std::vector<EdgeWeight>;
using UnpackedPath = std::tuple<EdgeWeight, UnpackedNodes, UnpackedEdges, UnpackedWeigths>;

template <typename T>
struct Either {
  public :
    static Either Left(T v) {
        return Either( true , v ) ;
    }
    static Either Right(T v) {
        return Either( false , v ) ;
    }
    bool isLeft() { return isLeft_ ; }
    bool isRight() { return !isLeft_ ; }
    T left() {
        return isLeft_ ? value_  : throw std::exception() ;
    }
    T get() {
        return !isLeft_ ? value_  : throw std::exception() ;
    }
    boost::optional<T> toOption() {
        return isLeft_ ? boost::optional<T>{} : boost::optional<T>(value_) ;
    }
  private :
    Either() { isLeft_ = false ; value_ = 0 ; }
    Either(bool is_left, T v) { isLeft_ = is_left ; value_ = v ; }
    bool isLeft_ ;
    T value_ ;
};

template <typename Algorithm, typename... Args>
UnpackedPath search(SearchEngineData<Algorithm> &engine_working_data,
                    const DataFacade<Algorithm> &facade,
                    typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                    typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                    std::function<EdgeWeight(const EdgeID id, const EdgeID turnId)> to_node_weight,
                    const bool force_loop_forward,
                    const bool force_loop_reverse,
                    EdgeWeight weight_upper_bound,
                    Args... args)
{
    if (forward_heap.Empty() || reverse_heap.Empty())
    {
        return std::make_tuple(INVALID_EDGE_WEIGHT, std::vector<NodeID>(), std::vector<EdgeID>(), std::vector<EdgeWeight>());
    }

    const auto &partition = facade.GetMultiLevelPartition();

    //  reverse_heap contains the target node for a route search, as it runs as a 2-way search
    //  In case of an isochrone , we perform a forward search remaining at the lowest level of the multilayer
    //  It is to notice that :
    //  * this is a recursive function for a route search (to resolve the high levels)
    //  * not recursive as we remain at the lowest level for an isochrone search
    bool reverse_search = reverse_heap.MinKey() != INVALID_EDGE_WEIGHT;

    BOOST_ASSERT(!forward_heap.Empty() && forward_heap.MinKey() < INVALID_EDGE_WEIGHT);
    BOOST_ASSERT(!reverse_heap.Empty() && reverse_heap.MinKey() <= INVALID_EDGE_WEIGHT);

    // run two-Target Dijkstra routing step.
    NodeID middle = SPECIAL_NODEID;
    EdgeWeight weight = weight_upper_bound;
    EdgeWeight forward_heap_min = forward_heap.MinKey();
    EdgeWeight reverse_heap_min = (reverse_search ? reverse_heap.MinKey() : static_cast<EdgeWeight>(0));
    while (forward_heap.Size() + reverse_heap.Size() > 0 &&
           forward_heap_min + reverse_heap_min < weight)
    {
        if (!forward_heap.Empty())
        {
            routingStep<FORWARD_DIRECTION>(facade,
                                           forward_heap,
                                           reverse_heap,
                                           to_node_weight,
                                           middle,
                                           weight,
                                           force_loop_forward,
                                           force_loop_reverse,
                                           args...);
            if (!forward_heap.Empty())
                forward_heap_min = forward_heap.MinKey();
        }
        if (reverse_search && !reverse_heap.Empty())
        {
            routingStep<REVERSE_DIRECTION>(facade,
                                           reverse_heap,
                                           forward_heap,
                                           to_node_weight,
                                           middle,
                                           weight,
                                           force_loop_reverse,
                                           force_loop_forward,
                                           args...);
            if (!reverse_heap.Empty())
                reverse_heap_min = reverse_heap.MinKey();
        }
    };

    // This condition is true when
    // * route search => No path found for both target nodes?
    // * isochrone search => All the paths explored overpassed the maximum weight given
    if (weight >= weight_upper_bound || SPECIAL_NODEID == middle)
    {
        //  We perform reverse searches when loking for routes
        if( reverse_search )
            return std::make_tuple(INVALID_EDGE_WEIGHT, std::vector<NodeID>(), std::vector<EdgeID>(), std::vector<EdgeWeight>());

        typedef std::pair<util::Coordinate,util::Coordinate> Segment ;
        const auto copy = [](auto &vector, const auto range) {
          vector.resize(range.size());
          std::copy(range.begin(), range.end(), vector.begin());
        };
        auto nodeCoordinates = [&facade,&copy](const NodeID node) -> util::Coordinate {
          const auto geometry_index = facade.GetGeometryIndex(node);
          std::vector<NodeID> id_vector;
          copy(id_vector, facade.GetUncompressedForwardGeometry(geometry_index.id));
          return facade.GetCoordinateOfNode(id_vector[1]) ;
        };
        //  Two segments seg1(p1,q1) and seg2(p2,q2) intersect if and only if one of the following two conditions is verified
        //  1. General Case:
        //  – (p1, q1, p2) and (p1, q1, q2) have different orientations and
        //  – (p2, q2, p1) and (p2, q2, q1) have different orientations
        //  2. Special Case
        //  – (p1, q1, p2), (p1, q1, q2), (p2, q2, p1), and (p2, q2, q1) are all collinear and
        //  – the x-projections of (p1, q1) and (p2, q2) intersect
        //  – the y-projections of (p1, q1) and (p2, q2) intersect.
#ifdef INTERSECTS
        auto intersect = [](const Segment& seg1, const Segment& seg2) -> int {
            // Given three colinear points p, q, r, the function checks if
            // point q lies on line segment 'pr'
            auto onSegment = [](const util::Coordinate& p, const util::Coordinate& q, const util::Coordinate& r) -> bool {
                if (q.lon <= std::max(p.lon, r.lon) && q.lon >= std::min(p.lon, r.lon) &&
                    q.lat <= std::max(p.lat, r.lat) && q.lat >= std::min(p.lat, r.lat))
                    return true;
                return false;
            };
            // To find orientation of ordered triplet (p, q, r).
            // The function returns following values
            //  0 --> p, q and r are colinear
            //  1 --> Clockwise
            // -1 --> Counterclockwise
            auto orientation = [](const util::Coordinate&  p, const util::Coordinate&  q, const util::Coordinate&  r) -> int {
                // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
                // for details of below formula.
                std::int32_t val = static_cast<std::int32_t>(q.lat - p.lat) * static_cast<std::int32_t>(r.lon - q.lon) -
                    static_cast<std::int32_t>(q.lon - p.lon) * static_cast<std::int32_t>(r.lat - q.lat);
                return val>0 ? 1 :  //  clock wise
                       val<0 ? -1 : //  counterclock wise
                               0 ;  //  Colinear
            };

            // Find the four orientations needed for general and special cases
            int o1 = orientation(seg1.first, seg1.second, seg2.first);
            int o2 = orientation(seg1.first, seg1.second, seg2.second);
            int o3 = orientation(seg2.first, seg2.second, seg1.first);
            int o4 = orientation(seg2.first, seg2.second, seg1.second);
            // General case :
            // (o1,o2)==(1,-1) : seg2 traversing seg1 from right to left (inside => outside)
            // (o1,o2)==(-1,1) : seg2 traversing seg1 from left to right (outside => inside)
            if (o1 != o2 && o3 != o4)
                  return -o1;   //  1 : entering ; -1 : exiting
            // Special Cases
            // p1, q1 and p2 are colinear and p2 lies on segment seg1
            // o2==1  : from seg1 to inside (return 2)
            // o2==-1 : from seg1 to outside (return -2)
            if (o1 == 0 && onSegment(seg1.first, seg2.first, seg1.second))
                return 2*o2;
            // p1, q1 and q2 are colinear and q2 lies on segment seg1
            // o1==1  : from inside to seg1 (return -3)
            // o1==-1 : from outside to seg1 (return 3)
            if (o2 == 0 && onSegment(seg1.first, seg2.second, seg1.second))
                return -3*o1;
            // p2, q2 and p1 are colinear and p1 lies on segment seg2
            // o4==1  : seg2 traversing seg1 from right to left while touching p1 (inside => outside)
            // o4==-1 : seg2 traversing seg1 from left to right while touching p1 (outside => inside)
            if (o3 == 0 && onSegment(seg2.first, seg1.first, seg2.second))
                return -o4;   //  1 : entering ; -1 : exiting
            // p2, q2 and q1 are colinear and q1 lies on segment seg2
            // o3==1  : seg2 traversing seg1 from right to left while touching q1 (outside => inside)
            // o3==-1 : seg2 traversing seg1 from left to right while touching q1 (inside => outside)
            if (o4 == 0 && onSegment(seg2.first, seg1.second, seg2.second))
                return o3;   //  1 : entering ; -1 : exiting

            return 0; // Doesn't fall in any of the above cases
        };
        auto intersects = [&](const std::vector<NodeID>& nodes, const std::pair<NodeID,NodeID>& edge) -> boost::optional<Either<NodeID>>{
            if( nodes.empty() )
                return boost::optional<Either<NodeID>>{} ;
            const auto c = nodeCoordinates(edge.second);
            Segment seg = std::make_pair(nodeCoordinates(edge.first), c ) ;
            auto prev = nodes.front() ;
            for( auto cur : nodes) {
                if( cur==prev )    // No check on last segment : Uturn
                    continue ;
                auto crossing = intersect( std::make_pair(nodeCoordinates(prev), nodeCoordinates(cur) ), seg ) ;
                if( crossing==1 ) { //  Entering into the polygon (polyline)
                    std::cout << "X>:" << std::setprecision(7 ) << toFloating(c.lon) << ' ' << toFloating(c.lat) << ':' << edge.second << std::endl ;
                    return Either<NodeID>::Right(cur);
                }
                if( crossing==-1 ) { //  Exiting from the polygon (polyline)
                    std::cout << "X<:" << std::setprecision(7 ) << toFloating(c.lon) << ' ' << toFloating(c.lat) << ':' << edge.second << std::endl ;
                    return Either<NodeID>::Left(cur);
                }
                if( crossing!=0 ) {
                    std::cout << "X(" << crossing << "):" << std::setprecision(7 ) << toFloating(c.lon) << ' ' << toFloating(c.lat) << ':' << edge.second << std::endl ;
                }
                prev = cur ;
            }
          return boost::optional<Either<NodeID>>{} ;
        };
#endif
        //  We perform only forward search when looking for isochrones
        std::vector<NodeID> unpacked_nodes;
        std::vector<NodeID> forward_nodes;
        std::set<NodeID> dead_ends;
        std::vector<EdgeWeight> unpacked_weigths;
        std::set<PackedEdge> edges_from_paths;
#ifdef TRIP_LEFT_TURNS
        /* Get the western node
         * Get the northest node from previous => 1st edge
         * From there on : find the turn-left most edge from current node, go through that edge
        */
        //  western node
        auto start_ = std::min_element( forward_heap.nodesBegin(), forward_heap.nodesEnd(), [&](const auto& d1, const auto& d2 ){
            /*auto isDeadEnd = [&facade,&forward_heap](const NodeID node ){
              const auto edgeRange = facade.GetAdjacentEdgeRange(node) ;
              auto nb_adjacents = 0 ;
              for (const auto edge : edgeRange) {
                  const auto to = facade.GetTarget(edge);
                  if( forward_heap.WasInserted(to) )
                      nb_adjacents += 1 ;
              }
              return nb_adjacents<2 ;
            };
            if( isDeadEnd(d1.node) )
                return false ;
            if( isDeadEnd(d2.node) )
                return true ;*/
            const util::Coordinate& coord1 = nodeCoordinates( d1.node ) ;
            const util::Coordinate& coord2 = nodeCoordinates( d2.node ) ;
            if( coord1.lon==coord2.lon )
                return coord1.lat<coord2.lat ;
            return coord1.lon<coord2.lon ;
        }) ;
        //  If no min element found => early return of empty vectors
        if( start_ == forward_heap.nodesEnd() )
            return std::make_tuple(weight, std::move(unpacked_nodes), std::vector<EdgeID>(), std::move(unpacked_weigths));
        using namespace osrm::util::coordinate_calculation;
        //  Coordinates of western node
        auto start_node = start_->node ;
        auto cur_node = start_node ;
        auto prev_node = cur_node ;
        //auto onUturn = false ;
        do {
            std::vector<std::pair<NodeID,int>> candidates;
            std::vector<std::pair<NodeID,int>> allCandidates;
            {
                std::cout << std::setprecision(7) << "LINESTRING(" ;
                std::for_each( unpacked_nodes.begin(), unpacked_nodes.end(), [&](const auto& node){
                  const auto c = nodeCoordinates(node);
                  std::cout << toFloating(c.lon) << ' ' << toFloating(c.lat) << ',' ;
                });
                std::cout << ')' << std::endl ;
                const auto c = nodeCoordinates(cur_node);
                std::cout << std::setprecision(7 ) << toFloating(c.lon) << ' ' << toFloating(c.lat) << ':' << cur_node << std::endl ;
            }
            auto fillBorderEdges = [&](auto node){ // Boundary edges
              const auto edgeRange = facade.GetAdjacentEdgeRange(node) ;
              for (const auto edge : edgeRange) {
                  const auto to = facade.GetTarget(edge);
                  allCandidates.push_back(std::make_pair(to,0)) ;
                  if( !forward_heap.WasInserted(to) )
                      continue ;
                  candidates.push_back(std::make_pair(to,0)) ;
#ifdef CHECK_CANDIDATES
                  if( std::find( dead_ends.begin(), dead_ends.end(), to ) != dead_ends.end() )
                      continue ;
                  boost::optional<Either<NodeID>> crossing = intersects( unpacked_nodes, std::make_pair(cur_node,to)) ;
                  // Not crossing : it's a candidate
                  if( !crossing )
                      candidates.push_back(std::make_pair(to,0));
                  // If crossing from outside to inside => candidate but to shorten the current path
                  else if( crossing && crossing->isRight() ) {
                      auto cross_node = crossing->get() ;
                      candidates.push_back(std::make_pair(to, cross_node ) );
                      //   If it crosses at the end of current path, we need to distinguish between
                      // Uturn or crossing to inside : insert cur_node will fool next test
                      //if( cross_node==unpacked_nodes.back() )
                      //    forward_nodes.push_back( cur_node ) ;
                  } else if( crossing && crossing->isLeft()) {    // We're crossing from inside to outside
                      if( to == forward_nodes.back() )    // last node of forward path => possibly a Uturn
                          candidates.push_back(std::make_pair(to, crossing->left()));
                      else
                      if( to == forward_nodes.front() )    // back to start of path => stop searching
                          candidates.push_back(std::make_pair(to, crossing->left()));
                  }
#endif
              }
            };
            fillBorderEdges(cur_node) ;
            const util::Coordinate source = nodeCoordinates(cur_node);
            auto fixAngle = [](double angle) {
              while(angle > 180) angle -= 360;
              while(angle <= -180) angle += 360;
              return angle ;
            };
            auto sortCandidates = [&](std::vector<std::pair<NodeID,int>>& positions) {
              //auto prev_node = unpacked_nodes.back() ;
              const util::Coordinate last = nodeCoordinates(prev_node);
              auto angle_start = fixAngle( bearing(last, source) ) ;
              std::stable_sort(positions.begin(), positions.end(), [&](const auto& a, const auto& b) {
                if( a.first== prev_node)
                    return false ;
                if( b.first== prev_node)
                    return true ;
                bool already_a = std::find( unpacked_nodes.begin(), unpacked_nodes.end(), a.first )!=unpacked_nodes.end() ;
                bool already_b = std::find( unpacked_nodes.begin(), unpacked_nodes.end(), b.first )!=unpacked_nodes.end() ;
                if( already_a && !already_b )
                    return false ;
                if( !already_a && already_b )
                    return true ;
                const auto ca = nodeCoordinates(a.first);
                const auto cb = nodeCoordinates(b.first);
                auto angle_a = fixAngle( bearing(source, ca) ) ;
                auto angle_b = fixAngle( bearing(source, cb) ) ;
                auto dev_a = fixAngle( angle_a - angle_start ) ;
                auto dev_b = fixAngle( angle_b - angle_start ) ;
                if (dev_a == dev_b)
                    return haversineDistance(source, ca) < haversineDistance(source, cb);
                return dev_a < dev_b;
              });
            };
            auto printBoundaries = [&](std::vector<std::pair<NodeID,int>>& positions) {
              const util::Coordinate last = nodeCoordinates(prev_node);
              auto angle_start = fixAngle( bearing(last, source) ) ;
              std::for_each( positions.begin(), positions.end(), [&](const auto pos){
                const auto c = nodeCoordinates(pos.first);
                std::cout << std::setprecision(7) << "POINT(" << toFloating(c.lon) << ' ' << toFloating(c.lat) << ")," ;
              });
              std::for_each( positions.begin(), positions.end(), [&](const auto pos){
                const auto c = nodeCoordinates(pos.first);
                auto dir = fixAngle( bearing(source, c) ) ;
                auto angle = fixAngle( dir - angle_start) ;
                std::cout << std::setprecision(4) << pos.first << ':' << angle << ',' ;
              });
              std::cout << std::endl ;
            };
            //  No candidates found right at the beginning => search no more (should not happen!!!)
            if( candidates.empty() ) //&& forward_nodes.empty() )
                break ;
            if( !candidates.empty() ) {//&& !forward_nodes.empty() ) {
                sortCandidates( candidates ) ;
                printBoundaries( candidates ) ;
                sortCandidates( allCandidates ) ;
                printBoundaries( allCandidates ) ;
                //NodeID to = candidates.front().first ;
#ifdef CHECK_UTURNS
                //  Check the case of a Uturn
                if( to==forward_nodes.back() ) {
                    if( !onUturn)
                        unpacked_nodes.push_back( cur_node ) ;
                    onUturn = true ;
                    dead_ends.insert(cur_node);
                    prev_node = cur_node ;
                    cur_node = to ;
                    forward_nodes.pop_back() ;
                    // Don't add to the path while in the Uturn
                    continue ;
                }
                //  Check if the candidate crosses the existing path (from outside to inside) => shorten the current pass
                NodeID crossNode = candidates.front().second ;
                if( crossNode ) {// Path is making a loop to re-enter itself => cut-off the loop trip
                    auto shortenPath = [](std::vector<NodeID>& path, NodeID node) {
                      auto cross = std::find(path.begin(), path.end(), node);
                      if (cross != path.end())
                          path.erase(cross, path.end());
                    };
                    shortenPath( unpacked_nodes, crossNode ) ;
                    shortenPath( forward_nodes, crossNode ) ;
                    prev_node = forward_nodes.back() ;
                    candidates.clear();
                    onUturn = false ;
                    continue;
                }
#endif
            }
            //  No candidates, only possibility : single target that crosses path from the right (inside => outside)
            //  Make a Uturn
#ifdef CANDIATES_EMPTY
            if( candidates.empty() ) {
                if( !onUturn ) {
                    if( !unpacked_nodes.empty() ) {
                        boost::optional<Either<NodeID>> crossing = intersects(unpacked_nodes, std::make_pair(unpacked_nodes.back(), cur_node));
                        if (crossing) {
                            auto crossNode = crossing->isRight() ? crossing->get() : crossing->left();
                            auto shortenPath = [](std::vector<NodeID> &path, NodeID node) {
                              auto cross = std::find(path.begin(), path.end(), node);
                              if (cross != path.end())
                                  path.erase(cross, path.end());
                            };
                            shortenPath(unpacked_nodes, crossNode);
                        }
                    }
                    unpacked_nodes.push_back(cur_node);
                }
                onUturn = true ;
                dead_ends.insert(cur_node);
                prev_node = cur_node ;
                cur_node = forward_nodes.back() ;
                forward_nodes.pop_back() ;

                //  If we had made our way back till the beginning => no path could be found (should not happen!!!!)
                if( forward_nodes.empty() )
                    break ;
                if( forward_nodes.size()==1 ) {
                    forward_nodes.clear() ;
                    unpacked_nodes.clear() ;
                }
                //  There's still nodes to explore
                continue ;
            }
            if( forward_nodes.empty() ) {
                std::sort(candidates.begin(), candidates.end(), [&](const auto& a, const auto& b) {
                  const auto ca = nodeCoordinates(a.first);
                  const auto cb = nodeCoordinates(b.first);
                  auto bearing_a = bearing(source, ca) ;
                  auto bearing_b = bearing(source, cb) ;
                  if (bearing_a == bearing_b)
                      return haversineDistance(source, ca) < haversineDistance(source, cb);
                  return bearing_a < bearing_b;
                });
            } else {
                if( candidates.size()==1 && forward_nodes.size()==2 ) {
                    unpacked_nodes.erase( unpacked_nodes.begin() ) ;
                    forward_nodes.erase( forward_nodes.begin() ) ;
                    unpacked_weigths.erase( unpacked_weigths.begin() ) ;
                }
            };
            if( !onUturn ) {
                if( !unpacked_nodes.empty() ) {
                    boost::optional<Either<NodeID>> crossing = intersects(unpacked_nodes, std::make_pair(unpacked_nodes.back(), cur_node));
                    if (crossing) {
                        auto crossNode = crossing->isRight() ? crossing->get() : crossing->left();
                        auto shortenPath = [](std::vector<NodeID> &path, NodeID node) {
                            auto cross = std::find(path.begin(), path.end(), node);
                            if (cross != path.end())
                                path.erase(cross, path.end());
                        };
                        shortenPath(unpacked_nodes, crossNode);
                    }
                }
                unpacked_nodes.push_back( cur_node ) ;
            }
            else
                dead_ends.insert(cur_node);
            onUturn = false ;
#endif
            unpacked_nodes.push_back( cur_node ) ;
            forward_nodes.push_back( cur_node ) ;
            start_node = forward_nodes.front() ;
            prev_node = forward_nodes.back() ;
            unpacked_weigths.push_back( 0 ) ;
            cur_node = candidates.front().first ;
        } while( cur_node!=start_node ) ;
#endif

        std::for_each( forward_heap.nodesBegin(), forward_heap.nodesEnd(), [&](const auto& d){
          unpacked_nodes.push_back( d.node ) ;
          unpacked_weigths.push_back( d.weight ) ;
        });
        //  Collect all the farthest nodes from all the current explored paths
        //  The node we want is the one before the last, as the last node is already beyond the limit
        //  We filter
        //  * the duplicates
        //  * the nodes included in an already explored path

        std::cout << "Number of nodes : " << forward_heap.Size() << std::endl ;
        std::cout << "GEOMETRYCOLLECTION(" << std::endl ;
        while (forward_heap.Size()>0 ){
            NodeID to = forward_heap.Min() ;
            auto to_weight = forward_heap.MinKey() ;
            //  Get back to a node under the limit
            while( to_weight>weight_upper_bound ) {
                to = forward_heap.GetData(to).parent ;
                to_weight = forward_heap.GetKey(to) ;
            }
#ifdef USE_THIS
            auto alreadyThere = [&unpacked_nodes](NodeID node) {
              return std::find(unpacked_nodes.begin(), unpacked_nodes.end(), node)!=unpacked_nodes.end() ;
            };
            auto alreadyInPath = [&nodes_from_paths](NodeID node) {
              return nodes_from_paths.find(node)!= nodes_from_paths.end() ;
            };
            auto toFar = [&to_weight,&weight_upper_bound]() {
              return to_weight>weight_upper_bound ;
            };
            //  Don't keep if we already have this node
            //  Don't keep if the node is in an already seen path
            //  Don't keep if the parent node is in an already seen path (avoids route forks of a single segment)

            if( !alreadyThere(to) && !alreadyInPath(forward_heap.GetData(to).parent) ) //&& !toFar() && !alreadyInPath(to)  )
            {
                unpacked_nodes.push_back(to);
                unpacked_weigths.push_back(to_weight);
            }
#endif
#ifndef USE_THIS
            auto alreadyInPath = [&edges_from_paths](PackedEdge& edge) {
              return std::find_if(edges_from_paths.begin(), edges_from_paths.end(), [&](auto& e){
                    NodeID from0, from1, to0, to1 ;
                    std::tie(from1, to1, std::ignore) = edge;
                    std::tie(from0, to0, std::ignore) = e;
                    return from1==from0 && to1==to0 ;
                })!= edges_from_paths.end() ;
            };
            PackedPath path = retrievePackedPathFromSingleHeap<FORWARD_DIRECTION>(forward_heap, to);
            std::cout << "LINESTRING(" ;
            auto separator = '\0' ;
            auto nbPoints = 0 ;
            std::for_each(path.begin(), path.end(), [&](PackedEdge &edge) {
              NodeID target;
              if( !alreadyInPath(edge) || nbPoints<2 ) {
                  std::tie(std::ignore, target, std::ignore) = edge;
                  const auto geometry_index = facade.GetGeometryIndex(target);
                  const auto copy = [](auto &vector, const auto range) {
                    vector.resize(range.size());
                    std::copy(range.begin(), range.end(), vector.begin());
                  };
                  std::vector<NodeID> id_vector;
                  copy(id_vector, facade.GetUncompressedForwardGeometry(geometry_index.id));
                  util::Coordinate coord = facade.GetCoordinateOfNode(id_vector[1]) ;
                  if( separator )
                      std::cout << separator ;
                  std::cout << toFloating(coord.lon) << ' ' << toFloating(coord.lat) ;
                  separator = ',' ;
                  nbPoints += 1 ;
              }
              edges_from_paths.insert(edge);
            });
            std::cout << ")," << std::endl ;
            //  Next step
            forward_heap.DeleteMin() ;
        }
        std::cout << "POLYGON(" ;
        auto separator = '\0' ;
        std::for_each( unpacked_nodes.begin(), unpacked_nodes.end(), [&](const auto d){
            const util::Coordinate coord = nodeCoordinates( d ) ;
            if( separator )
                std::cout << separator ;
            std::cout << toFloating(coord.lon) << ' ' << toFloating(coord.lat) ;
            separator = ',' ;
        });
        std::cout << ")" << std::endl ;
        std::cout << ")" << std::endl ;
#endif
        std::cout << " Unpacked : " << unpacked_nodes.size() << std::endl ;
        return std::make_tuple(weight, std::move(unpacked_nodes), std::vector<EdgeID>(), std::move(unpacked_weigths));
    }
    //  Only go beyond this point for a route search

    // Get packed path as edges {from node ID, to node ID, from_clique_arc}
    auto packed_path = retrievePackedPathFromHeap(forward_heap, reverse_heap, middle);

    // Beware the edge case when start, middle, end are all the same.
    // In this case we return a single node, no edges. We also don't unpack.
    const NodeID source_node = !packed_path.empty() ? std::get<0>(packed_path.front()) : middle;

    // Unpack path
    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;
    unpacked_nodes.reserve(packed_path.size());
    unpacked_edges.reserve(packed_path.size());

    unpacked_nodes.push_back(source_node);
    for (auto const &packed_edge : packed_path)
    {
        NodeID source, target;
        bool overlay_edge;
        std::tie(source, target, overlay_edge) = packed_edge;
        if (!overlay_edge)
        { // a base graph edge
            unpacked_nodes.push_back(target);
            unpacked_edges.push_back(facade.FindEdge(source, target));
        }
        else
        { // an overlay graph edge
            LevelID level = getNodeQueryLevel(partition, source, args...);
            CellID parent_cell_id = partition.GetCell(level, source);
            BOOST_ASSERT(parent_cell_id == partition.GetCell(level, target));

            LevelID sublevel = level - 1;

            // Here heaps can be reused, let's go deeper!
            forward_heap.Clear();
            reverse_heap.Clear();
            forward_heap.Insert(source, 0, {source});
            reverse_heap.Insert(target, 0, {target});

            // TODO: when structured bindings will be allowed change to
            // auto [subpath_weight, subpath_source, subpath_target, subpath] = ...
            EdgeWeight subpath_weight;
            std::vector<NodeID> subpath_nodes;
            std::vector<EdgeID> subpath_edges;
            std::tie(subpath_weight, subpath_nodes, subpath_edges, std::ignore) = mld::search(engine_working_data,
                                                                                 facade,
                                                                                 forward_heap,
                                                                                 reverse_heap,
                                                                                 to_node_weight,
                                                                                 force_loop_forward,
                                                                                 force_loop_reverse,
                                                                                 weight_upper_bound,
                                                                                 sublevel,
                                                                                 parent_cell_id);
            BOOST_ASSERT(!subpath_edges.empty());
            BOOST_ASSERT(subpath_nodes.size() > 1);
            BOOST_ASSERT(subpath_nodes.front() == source);
            BOOST_ASSERT(subpath_nodes.back() == target);
            unpacked_nodes.insert(
                unpacked_nodes.end(), std::next(subpath_nodes.begin()), subpath_nodes.end());
            unpacked_edges.insert(unpacked_edges.end(), subpath_edges.begin(), subpath_edges.end());
        }
    }

    return std::make_tuple(weight, std::move(unpacked_nodes), std::move(unpacked_edges), std::vector<EdgeWeight>());
}

// Alias to be compatible with the CH-based search
template <typename Algorithm>
inline void search(SearchEngineData<Algorithm> &engine_working_data,
                   const DataFacade<Algorithm> &facade,
                   typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                   typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                   std::function<EdgeWeight(const EdgeID id, const EdgeID turnId)> to_node_weight,
                   EdgeWeight &weight,
                   std::vector<NodeID> &unpacked_nodes,
                   const bool force_loop_forward,
                   const bool force_loop_reverse,
                   const PhantomNodes &phantom_nodes,
                   const EdgeWeight weight_upper_bound = INVALID_EDGE_WEIGHT)
{
    // TODO: change search calling interface to use unpacked_edges result
    std::tie(weight, unpacked_nodes, std::ignore, std::ignore) = search(engine_working_data,
                                                           facade,
                                                           forward_heap,
                                                           reverse_heap,
                                                           to_node_weight,
                                                           force_loop_forward,
                                                           force_loop_reverse,
                                                           weight_upper_bound,
                                                           phantom_nodes);
}

template <typename Algorithm>
inline std::function<EdgeWeight(const EdgeID id, const EdgeID turnId)>
getWeightStrategy( const DataFacade<Algorithm> &facade, osrm::engine::api::BaseParameters::OptimizeType optimize) {

    auto nodeWeight = [optimize, &facade](const EdgeID id, const EdgeID turnId) {
      switch( optimize) {
      case osrm::engine::api::BaseParameters::OptimizeType::Distance :
          return static_cast<EdgeWeight>(facade.GetNodeDistance(id)) ;
          //break ;
      case osrm::engine::api::BaseParameters::OptimizeType::Time :
          return static_cast<EdgeWeight>(facade.GetNodeDuration(id) + facade.GetDurationPenaltyForEdgeID(turnId));
          //break ;
      default :
          return facade.GetNodeWeight(id) + facade.GetWeightPenaltyForEdgeID(turnId);
          //break ;
      }
    };
    return nodeWeight ;
}

// TODO: refactor CH-related stub to use unpacked_edges
template <typename RandomIter, typename FacadeT>
void unpackPath(const FacadeT &facade,
                RandomIter packed_path_begin,
                RandomIter packed_path_end,
                const PhantomNodes &phantom_nodes,
                std::vector<PathData> &unpacked_path)
{
    const auto nodes_number = std::distance(packed_path_begin, packed_path_end);
    BOOST_ASSERT(nodes_number > 0);

    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;
    unpacked_nodes.reserve(nodes_number);
    unpacked_edges.reserve(nodes_number);

    unpacked_nodes.push_back(*packed_path_begin);
    if (nodes_number > 1)
    {
        util::for_each_pair(
            packed_path_begin,
            packed_path_end,
            [&facade, &unpacked_nodes, &unpacked_edges](const auto from, const auto to) {
                unpacked_nodes.push_back(to);
                unpacked_edges.push_back(facade.FindEdge(from, to));
            });
    }

    annotatePath(facade, phantom_nodes, unpacked_nodes, unpacked_edges, unpacked_path);
}

template <typename Algorithm>
double getNetworkDistance(SearchEngineData<Algorithm> &engine_working_data,
                          const DataFacade<Algorithm> &facade,
                          typename SearchEngineData<Algorithm>::QueryHeap &forward_heap,
                          typename SearchEngineData<Algorithm>::QueryHeap &reverse_heap,
                          const PhantomNode &source_phantom,
                          const PhantomNode &target_phantom,
                          EdgeWeight weight_upper_bound = INVALID_EDGE_WEIGHT)
{
    forward_heap.Clear();
    reverse_heap.Clear();

    const PhantomNodes phantom_nodes{source_phantom, target_phantom};
    insertNodesInHeaps(forward_heap, reverse_heap, phantom_nodes);

    EdgeWeight weight = INVALID_EDGE_WEIGHT;
    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;
    std::tie(weight, unpacked_nodes, unpacked_edges, std::ignore) = search(engine_working_data,
                                                              facade,
                                                              forward_heap,
                                                              reverse_heap,
                                                              getWeightStrategy(facade,osrm::engine::api::BaseParameters::OptimizeType::Weight),
                                                              DO_NOT_FORCE_LOOPS,
                                                              DO_NOT_FORCE_LOOPS,
                                                              weight_upper_bound,
                                                              phantom_nodes);

    if (weight == INVALID_EDGE_WEIGHT)
    {
        return std::numeric_limits<double>::max();
    }

    std::vector<PathData> unpacked_path;

    annotatePath(facade, phantom_nodes, unpacked_nodes, unpacked_edges, unpacked_path);

    return getPathDistance(facade, unpacked_path, source_phantom, target_phantom);
}

} // namespace mld
} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif // OSRM_ENGINE_ROUTING_BASE_MLD_HPP
