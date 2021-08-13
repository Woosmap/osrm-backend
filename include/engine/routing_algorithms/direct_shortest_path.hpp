#ifndef DIRECT_SHORTEST_PATH_HPP
#define DIRECT_SHORTEST_PATH_HPP

#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/internal_route_result.hpp"
#include "engine/search_engine_data.hpp"

#include "util/typedefs.hpp"

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
template <typename Algorithm>
InternalRouteResult directShortestPathSearch(SearchEngineData<Algorithm> &engine_working_data,
                                             const DataFacade<Algorithm> &facade,
                                             const PhantomNodes &phantom_nodes,
                                             std::function<EdgeWeight(const PhantomNode &, bool)> phantom_weights,
                                             osrm::engine::api::BaseParameters::OptimizeType optimize);

/// This is a derived version of <code>directShortestPathSearch</code> with no reverse search
/// We stop when all the possible routes have a weight higher than asked
template <typename Algorithm>
std::vector<util::Coordinate>
forwardIsochroneSearch(SearchEngineData<Algorithm> &engine_working_data,
                       const DataFacade<Algorithm> &facade,
                       const PhantomNodes &phantom_nodes,
                       std::function<EdgeWeight(const PhantomNode &, bool)> phantom_weights,
                       osrm::engine::api::BaseParameters::OptimizeType optimize,
                       EdgeWeight max_weight,
                       EdgeWeight min_weight);


} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif /* DIRECT_SHORTEST_PATH_HPP */
