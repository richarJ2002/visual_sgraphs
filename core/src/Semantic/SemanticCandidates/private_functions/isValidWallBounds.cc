

#include "Semantic/SemanticCandidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <tuple>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isValidWallBounds(const WallBounds &bounds_in)
{
    return bounds_in.isValid && std::isfinite(bounds_in.minU_m) &&
           std::isfinite(bounds_in.maxU_m) && std::isfinite(bounds_in.minV_m) &&
           std::isfinite(bounds_in.maxV_m) &&
           bounds_in.maxU_m > bounds_in.minU_m &&
           bounds_in.maxV_m > bounds_in.minV_m;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
