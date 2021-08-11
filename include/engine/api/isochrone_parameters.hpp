/*

Copyright (c) 2016, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ENGINE_API_ISOCHRONE_PARAMETERS_HPP
#define ENGINE_API_ISOCHRONE_PARAMETERS_HPP

#include "engine/api/route_parameters.hpp"

namespace osrm
{
namespace engine
{
namespace api
{

/**
 * Parameters specific to the OSRM Isochrone service.
 *
 * Holds member attributes:
 *  - lon: centerpoint
 *  - lat: centerpoint
 *  - range: distance to travel
 *
 * \see OSRM, Coordinate, Hint, Bearing, RouteParameters, TableParameters,
 *      NearestParameters, TripParameters, MatchParameters and TileParameters
 */
struct IsochroneParameters : public RouteParameters
{

    /// Range value to consider an isochrone point
    EdgeWeight range = (optimize==BaseParameters::OptimizeType::Distance ? 1000 : 300);
    /// If the point weight is below that percentage of the range
    /// => don't consider the point : for cases where it stops before a very long segment
    std::size_t convexity_value = 50;

    bool IsValid() const {
        return BaseParameters::IsValid() &&
               //   Minimum distance : 100 m / Minimum time : 1 mn
               range >= (optimize==BaseParameters::OptimizeType::Distance ? 100 : 60) &&
               //   Maximum distance : 200 km / Maximum time : 2 h
               range <= (optimize==BaseParameters::OptimizeType::Distance ? 200*1000 : 2*60*60) &&
               convexity_value <=100 ;
    }

    boost::optional<std::string> IsValid(bool /*give_me_a_message*/) const {
        if( !BaseParameters::IsValid() )
            return boost::optional<std::string>("Base parameters are wrong");
        switch (optimize)
        {
        case BaseParameters::OptimizeType::Distance :
            if( range < 100 || range>200*1000 )
                return boost::optional<std::string>("The travel distance must be in range [100 m , 200 km]");
            break ;
        default :
            if( range < 60 || range>2*60*60 )
                return boost::optional<std::string>("The travel time must be in range [1 mn , 2:00 h]");
            break ;
        }
        if( convexity_value>100 )
            return boost::optional<std::string>("convexity should be less than 100");
        return boost::optional<std::string>() ;
    }
};
}
}
}

#endif
