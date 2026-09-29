

#include "Semantic/SemanticCandidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <rclcpp/logging.hpp>
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
SemanticCandidatesStatus apertureSignature(
    const RoomContextSnapshot              &snapshot_in,
    const double                            median_in,
    const std::size_t                       cap_in,
    std::vector<std::pair<double, double>> &apertureSignature_out)
{
    std::vector<std::pair<double, double>> signature;
    if (!std::isfinite(median_in) || median_in <= 0.0)
    {
        apertureSignature_out = signature;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    signature.reserve(std::min(snapshot_in.passageContexts.size(), cap_in));
    for (const PassageContext &passage : snapshot_in.passageContexts)
    {
        if (signature.size() == cap_in)
        {
            break;
        }
        bool isFiniteNonnegative{};
        if ((passage.isApertureValid) &&
            finiteNonnegative(passage.width_m, isFiniteNonnegative) !=
                SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: finiteNonnegative returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool isFiniteNonnegative2{};
        if ((passage.isApertureValid && isFiniteNonnegative) &&
            finiteNonnegative(passage.height_m, isFiniteNonnegative2) !=
                SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: finiteNonnegative returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (passage.isApertureValid && isFiniteNonnegative &&
            isFiniteNonnegative2 && passage.width_m > 0.0 &&
            passage.height_m > 0.0)
        {
            signature.emplace_back(passage.width_m / median_in,
                                   passage.height_m / median_in);
        }
    }
    std::sort(signature.begin(), signature.end());
    apertureSignature_out = signature;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
