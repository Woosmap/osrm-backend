#ifndef OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_NODES_HPP
#define OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_NODES_HPP


#include "engine/api/isochrone_parameters.hpp"
#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/search_engine_data.hpp"

#include "util/typedefs.hpp"

#include <vector>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

template <typename Algorithm>
std::vector<PhantomNode> isochroneNodes(SearchEngineData<Algorithm> &engine_working_data,
                                             const DataFacade<Algorithm> &facade,
                                             std::vector<PhantomNode> &phantom_nodes,
                                        const api::IsochroneParameters &parameters) ;

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif