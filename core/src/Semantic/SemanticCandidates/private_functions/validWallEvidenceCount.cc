

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

/*! Counts walls whose SAME index has both a valid finite unit-able normal and
 * valid bounds ("walls with valid normals and bounds"). */
std::size_t validWallEvidenceCount(const RoomContextSnapshot &snapshot_in)
{
    const std::size_t pairedCount =
        std::min(snapshot_in.wallNormals.size(), snapshot_in.wallBounds.size());
    std::size_t count = 0U;
    for (std::size_t index = 0U; index < pairedCount; ++index)
    {
        const Eigen::Vector3d &normal = snapshot_in.wallNormals[index];
        if (normal.allFinite() && normal.norm() > 1e-12 &&
            isValidWallBounds(snapshot_in.wallBounds[index]))
        {
            ++count;
        }
    }
    return count;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
