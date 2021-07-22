#include "server/service/isochrone_service.hpp"
#include "server/service/utils.hpp"

#include "server/api/parameters_parser.hpp"
#include "engine/api/isochrone_parameters.hpp"

#include "util/json_container.hpp"

#include <boost/format.hpp>

namespace osrm
{
namespace server
{
namespace service
{

engine::Status
IsochroneService::RunQuery(std::size_t prefix_length, std::string &query, osrm::engine::api::ResultT &result)
{
    auto query_iterator = query.begin();
    auto parameters =
        api::parseParameters<engine::api::IsochroneParameters>(query_iterator, query.end());
    if (!parameters || query_iterator != query.end())
    {
        const auto position = std::distance(query.begin(), query_iterator);
        result = util::json::Object();
        auto &json_result = result.get<util::json::Object>();
        json_result.values["code"] = "InvalidQuery";
        json_result.values["message"] =
            "Query string malformed close to position " + std::to_string(prefix_length + position);
        return engine::Status::Error;
    }

    boost::optional<std::string> not_valid = parameters->IsValid(true) ;
    if (not_valid)
    {
        result = util::json::Object();
        auto &json_result = result.get<util::json::Object>();
        json_result.values["code"] = "InvalidOptions";
        json_result.values["message"] = not_valid.get();
        return engine::Status::Error;
    }

    if (parameters->format && parameters->format == engine::api::BaseParameters::OutputFormatType::FLATBUFFERS)
        result = flatbuffers::FlatBufferBuilder();
    else
        result = util::json::Object();

    return BaseService::routing_machine.Isochrone(*parameters, result);
}
}
}
}
