#ifndef SERVER_API_ISOCHRONE_PARAMETERS_GRAMMAR_HPP
#define SERVER_API_ISOCHRONE_PARAMETERS_GRAMMAR_HPP

#include "server/api/route_parameters_grammar.hpp"
#include "engine/api/isochrone_parameters.hpp"

#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

#include <string>

namespace osrm
{
namespace server
{
namespace api
{

namespace
{
namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
}

template <typename Iterator = std::string::iterator,
    typename Signature = void(engine::api::IsochroneParameters &)>
struct IsochroneParametersGrammar final : public RouteParametersGrammar<Iterator, Signature>
{
    using RouteGrammar = RouteParametersGrammar<Iterator, Signature>;


    IsochroneParametersGrammar() : RouteGrammar(root_rule)
    {

        range_rule =
            (qi::lit("range=") >
             qi::uint_)[ph::bind(&engine::api::IsochroneParameters::range, qi::_r1) = qi::_1];

        convexity_rule = qi::lit("full")[qi::_val = 100] |
            qi::lit("smooth")[qi::_val = 75] |
            qi::lit("medium")[qi::_val = 50] |
            qi::lit("sharp")[qi::_val = 25] |
            qi::lit("none")[qi::_val = 0];
        convex_param_rule =
            qi::lit("convexity=") >
                (qi::uint_ | convexity_rule)[ph::bind(&engine::api::IsochroneParameters::convexity_value, qi::_r1) = qi::_1];

        root_rule = RouteGrammar::query_rule(qi::_r1) > -qi::lit(".json") >
                    -('?' > (range_rule(qi::_r1) | convex_param_rule(qi::_r1) | RouteGrammar::base_rule(qi::_r1)) % '&');
    }

  private:
    qi::rule<Iterator, Signature> root_rule;
    qi::rule<Iterator, Signature> range_rule;
    qi::rule<Iterator, uint()> convexity_rule;
    qi::rule<Iterator, Signature> convex_param_rule;

};
}
}
}

#endif
