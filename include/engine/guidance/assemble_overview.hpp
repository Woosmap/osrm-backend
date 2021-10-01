#ifndef ENGINE_GUIDANCE_ASSEMBLE_OVERVIEW_HPP
#define ENGINE_GUIDANCE_ASSEMBLE_OVERVIEW_HPP

#include "engine/guidance/leg_geometry.hpp"

#include "util/coordinate.hpp"

#include <vector>

namespace osrm
{
namespace engine
{
namespace guidance
{

std::vector<util::Coordinate> assembleOverview(const std::vector<LegGeometry> &leg_geometries,
                                               const bool use_simplification);

std::vector<util::Coordinate> reduceOverview(const std::vector<util::Coordinate> &geometry,
                                             const unsigned alpha_max, const bool use_simplification);
} // namespace guidance
} // namespace engine
} // namespace osrm

#endif
