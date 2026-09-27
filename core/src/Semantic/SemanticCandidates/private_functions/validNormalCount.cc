

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

std::size_t validNormalCount(const RoomContextSnapshot &snapshot_in)
{
    std::size_t count = 0U;
    for (const Eigen::Vector3d &normal : snapshot_in.wallNormals)
    {
        if (normal.allFinite() && normal.norm() > 1e-12)
        {
            ++count;
        }
    }
    return count;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
