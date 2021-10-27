#include <boost/test/unit_test.hpp>

#include <cmath>

#include "coordinates.hpp"
#include "equal_json.hpp"
#include "fixture.hpp"

#include "engine/api/flatbuffers/fbresult_generated.h"
#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/exception.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/isochrone_parameters.hpp"
#include "osrm/status.hpp"

osrm::Status run_isochrone_json(const osrm::OSRM &osrm,
                            const osrm::IsochroneParameters &params,
                            osrm::json::Object &json_result,
                            bool use_json_only_api)
{
    if (use_json_only_api)
    {
        return osrm.Isochrone(params, json_result);
    }
    osrm::engine::api::ResultT result = osrm::json::Object();
    auto rc = osrm.Isochrone(params, result);
    json_result = result.get<osrm::json::Object>();
    return rc;
}

BOOST_AUTO_TEST_SUITE(isochrone)

void test_isochrone_many_coordinates(bool use_json_only_api, osrm::EngineConfig::Algorithm algorithm)
{
    auto osrm = ( algorithm==osrm::EngineConfig::Algorithm::CH ?
                  getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm") :
                  getOSRM_MLD(OSRM_TEST_DATA_DIR "/mld/monaco.osrm")
    );

    using namespace osrm;

    IsochroneParameters params;
    params.coordinates.push_back(get_dummy_location());
    params.coordinates.push_back(params.coordinates.front());

    json::Object json_result;
    const auto rc = run_isochrone_json(osrm, params, json_result, use_json_only_api);
    BOOST_CHECK(rc == Status::Error);

    const auto code = json_result.values.at("code").get<json::String>().value;
    if( algorithm==osrm::EngineConfig::Algorithm::CH )
        BOOST_CHECK_EQUAL(code, "NotImplemented");
    else
        BOOST_CHECK_EQUAL(code, "InvalidOptions");

}
BOOST_AUTO_TEST_CASE(test_isochrone_many_coordinates_old_api) { test_isochrone_many_coordinates(true,osrm::EngineConfig::Algorithm::MLD); }
BOOST_AUTO_TEST_CASE(test_isochrone_many_coordinates_new_api) { test_isochrone_many_coordinates(false,osrm::EngineConfig::Algorithm::MLD); }

void test_isochrone_small_range_coordinate(bool use_json_only_api, osrm::EngineConfig::Algorithm algorithm)
{
    auto osrm = ( algorithm==osrm::EngineConfig::Algorithm::CH ?
                  getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm") :
                  getOSRM_MLD(OSRM_TEST_DATA_DIR "/mld/monaco.osrm")
    );

    using namespace osrm;

    IsochroneParameters params;
    params.coordinates.push_back(get_dummy_location());
    params.optimize = IsochroneParameters::OptimizeType::Distance;
    //  The range should be higher than 100 m
    params.max_weight = 60;

    json::Object json_result;
    const auto rc = run_isochrone_json(osrm, params, json_result, use_json_only_api);
    BOOST_CHECK(rc == Status::Error);

    const auto code = json_result.values.at("code").get<json::String>().value;
    if( algorithm==osrm::EngineConfig::Algorithm::CH )
        BOOST_CHECK_EQUAL(code, "NotImplemented");
    else
        BOOST_CHECK_EQUAL(code, "InvalidOptions");
}
BOOST_AUTO_TEST_CASE(test_isochrone_small_range_coordinate_old_api) { test_isochrone_small_range_coordinate(true,osrm::EngineConfig::Algorithm::MLD); }
BOOST_AUTO_TEST_CASE(test_isochrone_small_range_coordinate_new_api) { test_isochrone_small_range_coordinate(false,osrm::EngineConfig::Algorithm::MLD); }
BOOST_AUTO_TEST_CASE(test_isochrone_small_range_coordinate_old_api_CH) { test_isochrone_small_range_coordinate(true,osrm::EngineConfig::Algorithm::CH); }
BOOST_AUTO_TEST_CASE(test_isochrone_small_range_coordinate_new_api_CH) { test_isochrone_small_range_coordinate(false,osrm::EngineConfig::Algorithm::CH); }

BOOST_AUTO_TEST_SUITE_END()
