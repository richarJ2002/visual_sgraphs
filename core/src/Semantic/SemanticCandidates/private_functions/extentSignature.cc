

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

SemanticCandidatesStatus
    extentSignature(const RoomContextSnapshot &snapshot_in,
                    const double               median_in,
                    const std::size_t          cap_in,
                    std::vector<double>       &extentSignature_out)
{
    std::vector<double> signature;
    if (!std::isfinite(median_in) || median_in <= 0.0)
    {
        extentSignature_out = signature;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    signature.reserve(std::min(snapshot_in.wallBounds.size(), cap_in));
    for (const WallBounds &bounds : snapshot_in.wallBounds)
    {
        if (signature.size() == cap_in)
        {
            break;
        }
        bool isValidWallBounds2{};
        if (isValidWallBounds(bounds, isValidWallBounds2) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            // isValidWallBounds cannot fail; continue as before.
        }
        if (isValidWallBounds2)
        {
            signature.push_back((bounds.maxU_m - bounds.minU_m) / median_in);
            if (signature.size() < cap_in)
            {
                signature.push_back((bounds.maxV_m - bounds.minV_m) /
                                    median_in);
            }
        }
    }
    std::sort(signature.begin(), signature.end());
    extentSignature_out = signature;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
