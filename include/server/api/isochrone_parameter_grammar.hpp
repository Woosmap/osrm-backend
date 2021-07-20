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
        percent_rule =
            (qi::lit("range_percent=") >
             qi::uint_)[ph::bind(&engine::api::IsochroneParameters::range_percent, qi::_r1) = qi::_1];

        root_rule = RouteGrammar::query_rule(qi::_r1) > -qi::lit(".json") >
                    -('?' > (range_rule(qi::_r1) | percent_rule(qi::_r1) | RouteGrammar::base_rule(qi::_r1)) % '&');
    }

  private:
    qi::rule<Iterator, Signature> root_rule;
    qi::rule<Iterator, Signature> range_rule;
    qi::rule<Iterator, Signature> percent_rule;
};
}
}
}

#endif
