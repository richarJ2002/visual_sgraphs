/*!
 * @file            isValidWallBounds.cc
 *
 * @brief           Implements isValidWallBounds(), declared in
 *                  Semantic/SemanticCandidates/private_functions.h.
 */

#include "semantic/semantic_candidates/semantic_candidates.h"
#include "semantic/room/room.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <tuple>

#include "semantic/semantic_candidates/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticCandidatesStatus isValidWallBounds(const WallBounds &bounds_in,
                                           bool &isValidWallBounds_out)
{
    isValidWallBounds_out =
        bounds_in.isValid && std::isfinite(bounds_in.minU_m) &&
        std::isfinite(bounds_in.maxU_m) && std::isfinite(bounds_in.minV_m) &&
        std::isfinite(bounds_in.maxV_m) &&
        bounds_in.maxU_m > bounds_in.minU_m &&
        bounds_in.maxV_m > bounds_in.minV_m;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
