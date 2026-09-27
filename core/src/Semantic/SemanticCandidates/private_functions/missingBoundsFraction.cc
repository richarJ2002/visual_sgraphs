

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

/*! Fraction of wallBounds entries that are invalid; 0.0 when there are no
 * walls to be missing from (">50% missing bounds" guard). */
double missingBoundsFraction(const RoomContextSnapshot &snapshot_in)
{
    if (snapshot_in.wallBounds.empty())
    {
        return 0.0;
    }
    std::size_t validCount = 0U;
    for (const WallBounds &bounds : snapshot_in.wallBounds)
    {
        if (isValidWallBounds(bounds))
        {
            ++validCount;
        }
    }
    return 1.0 - static_cast<double>(validCount) /
                     static_cast<double>(snapshot_in.wallBounds.size());
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
