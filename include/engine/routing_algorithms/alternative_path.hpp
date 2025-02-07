#ifndef ALTERNATIVE_PATH_ROUTING_HPP
#define ALTERNATIVE_PATH_ROUTING_HPP

#include "engine/api/base_parameters.hpp"
#include "engine/datafacade.hpp"
#include "engine/internal_route_result.hpp"

#include "engine/algorithm.hpp"
#include "engine/search_engine_data.hpp"

#include "util/exception.hpp"

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{


InternalManyRoutesResult alternativePathSearch(SearchEngineData<ch::Algorithm> &search_engine_data,
                                               const DataFacade<ch::Algorithm> &facade,
                                               const PhantomNodes &phantom_node_pair,
                                               std::function<EdgeWeight(const PhantomNode &, bool)> phantom_weights,
                                               osrm::engine::api::BaseParameters::OptimizeType optimize,
                                               unsigned number_of_alternatives);

InternalManyRoutesResult alternativePathSearch(SearchEngineData<mld::Algorithm> &search_engine_data,
                                               const DataFacade<mld::Algorithm> &facade,
                                               const PhantomNodes &phantom_node_pair,
                                               std::function<EdgeWeight(const PhantomNode &, bool)> phantom_weights,
                                               osrm::engine::api::BaseParameters::OptimizeType optimize,
                                               unsigned number_of_alternatives);

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm

#endif
