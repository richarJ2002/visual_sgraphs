/*!
 * @file            validNormalCount.cc
 *
 * @brief           Implements validNormalCount(), declared in
 *                  Semantic/SemanticCandidates/private_functions.h.
 */

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

} // namespace semantic
} // namespace core
} // namespace vs_graphs
