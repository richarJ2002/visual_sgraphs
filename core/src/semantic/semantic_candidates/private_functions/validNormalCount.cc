/*!
 * @file            validNormalCount.cc
 *
 * @brief           Implements validNormalCount(), declared in
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

SemanticCandidatesStatus
    validNormalCount(const RoomContextSnapshot &snapshot_in,
                     std::size_t               &validNormalCount_out)
{
    std::size_t count = 0U;
    for (const Eigen::Vector3d &normal : snapshot_in.wallNormals)
    {
        if (normal.allFinite() && normal.norm() > 1e-12)
        {
            ++count;
        }
    }
    validNormalCount_out = count;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
