

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

/*! One (width,height) aperture pair, normalised by the room's valid median
 * extent. Kept paired (not flattened) so lexicographic sort and pairwise
 * Manhattan distance compare a passage's own width against its own height. */
std::vector<std::pair<double, double>>
    apertureSignature(const RoomContextSnapshot &snapshot_in,
                      const double               median_in,
                      const std::size_t          cap_in)
{
    std::vector<std::pair<double, double>> signature;
    if (!std::isfinite(median_in) || median_in <= 0.0)
    {
        return signature;
    }
    signature.reserve(std::min(snapshot_in.passageContexts.size(), cap_in));
    for (const PassageContext &passage : snapshot_in.passageContexts)
    {
        if (signature.size() == cap_in)
        {
            break;
        }
        if (passage.isApertureValid && finiteNonnegative(passage.width_m) &&
            finiteNonnegative(passage.height_m) && passage.width_m > 0.0 &&
            passage.height_m > 0.0)
        {
            signature.emplace_back(passage.width_m / median_in,
                                   passage.height_m / median_in);
        }
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
