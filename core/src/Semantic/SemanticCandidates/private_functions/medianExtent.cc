/*!
 * @file            medianExtent.cc
 *
 * @brief           Implements medianExtent(), declared in
 *                  Semantic/SemanticCandidates/private_functions.h.
 */

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

SemanticCandidatesStatus medianExtent(const RoomContextSnapshot &snapshot_in,
                                      const std::size_t          cap_in,
                                      double &medianExtent_out)
{
    std::vector<double> spans;
    spans.reserve(std::min(snapshot_in.wallBounds.size(), cap_in));
    for (const WallBounds &bounds : snapshot_in.wallBounds)
    {
        if (spans.size() + 1U >= cap_in)
        {
            break;
        }
        bool isValidWallBounds2{};
        if (isValidWallBounds(bounds, isValidWallBounds2) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isValidWallBounds returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (isValidWallBounds2)
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
        medianExtent_out = 0.0;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    std::sort(spans.begin(), spans.end());
    const std::size_t middle = spans.size() / 2U;
    medianExtent_out         = spans.size() % 2U == 0U
                                   ? (spans[middle - 1U] + spans[middle]) / 2.0
                                   : spans[middle];
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
