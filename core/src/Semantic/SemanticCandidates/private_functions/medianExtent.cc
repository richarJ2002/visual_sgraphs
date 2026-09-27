

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

double medianExtent(const RoomContextSnapshot &snapshot_in,
                    const std::size_t          cap_in)
{
    std::vector<double> spans;
    spans.reserve(std::min(snapshot_in.wallBounds.size(), cap_in));
    for (const WallBounds &bounds : snapshot_in.wallBounds)
    {
        if (spans.size() + 1U >= cap_in)
        {
            break;
        }
        if (isValidWallBounds(bounds))
        {
            spans.push_back(bounds.maxU_m - bounds.minU_m);
            if (spans.size() < cap_in)
            {
                spans.push_back(bounds.maxV_m - bounds.minV_m);
            }
        }
    }
    if (spans.empty())
    {
        return 0.0;
    }
    std::sort(spans.begin(), spans.end());
    const std::size_t middle = spans.size() / 2U;
    return spans.size() % 2U == 0U ? (spans[middle - 1U] + spans[middle]) / 2.0
                                   : spans[middle];
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
