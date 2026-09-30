/*!
 * @file            missingBoundsFraction.cc
 *
 * @brief           Implements missingBoundsFraction(), declared in
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

/*! Fraction of wallBounds entries that are invalid; 0.0 when there are no
 * walls to be missing from (">50% missing bounds" guard). */
SemanticCandidatesStatus
    missingBoundsFraction(const RoomContextSnapshot &snapshot_in,
                          double                    &missingBoundsFraction_out)
{
    if (snapshot_in.wallBounds.empty())
    {
        missingBoundsFraction_out = 0.0;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    std::size_t validCount = 0U;
    for (const WallBounds &bounds : snapshot_in.wallBounds)
    {
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
            ++validCount;
        }
    }
    missingBoundsFraction_out =
        1.0 - static_cast<double>(validCount) /
                  static_cast<double>(snapshot_in.wallBounds.size());
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
