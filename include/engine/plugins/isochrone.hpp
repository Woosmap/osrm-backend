#ifndef ISOCHRONEPLUGIN_HPP
#define ISOCHRONEPLUGIN_HPP

#include "engine/api/isochrone_parameters.hpp"
#include "engine/plugins/plugin_base.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/search_engine_data.hpp"

#include <string>

/**
 * This plugin generates an isochrone polygon around a given point
 * You can specify the weight to consider for the isochrone construction.
 * Either the weight (default) or shortest distance or least duration
 */
namespace osrm
{
namespace engine
{
namespace plugins
{

class IsochronePlugin final : public BasePlugin
{
//  private:
//    mutable SearchEngineData heaps;

  public:
//    explicit IsochronePlugin();

    Status HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                         const api::IsochroneParameters &parameters,
                         osrm::engine::api::ResultT &result) const;
    Status HandleRequest__(const RoutingAlgorithmsInterface &algorithms,
                         const api::IsochroneParameters &parameters,
                         osrm::engine::api::ResultT &result) const;
};
}
}
}

#endif /* ISOCHRONEPLUGIN_HPP */
