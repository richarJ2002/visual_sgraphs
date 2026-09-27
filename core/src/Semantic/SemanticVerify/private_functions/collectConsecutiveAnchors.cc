

#include "Semantic/SemanticVerify.h"

#include "Geometric/Plane.h"
#include "LoopClosing.h"
#include "Map.h"
#include "OptimizableTypes.h"
#include "Semantic/Room.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "Types/objects/SystemParams.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<ConsecutiveAnchorPair> collectConsecutiveAnchors(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in)
{
    std::map<std::string, const SemanticMergeRoomEvidence *> survivingByTag;
    for (const SemanticMergeRoomEvidence &room : survivingRooms_in)
    {
        if (!room.context.roomTag.empty())
        {
            survivingByTag.emplace(room.context.roomTag, &room);
        }
    }
    std::vector<ConsecutiveAnchorPair> pairs;
    for (const SemanticMergeRoomEvidence &room : absorbedRooms_in)
    {
        if (room.context.roomTag.empty())
        {
            continue;
        }
        const auto match = survivingByTag.find(room.context.roomTag);
        if (match != survivingByTag.end())
        {
            ConsecutiveAnchorPair pair;
            pair.p_surviving = match->second;
            pair.p_absorbed  = &room;
            pairs.push_back(pair);
        }
    }
    return pairs;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
