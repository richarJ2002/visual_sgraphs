/*!
 * @file            collectConsecutiveAnchors.cc
 *
 * @brief           Implements collectConsecutiveAnchors(), declared in
 *                  Semantic/SemanticVerify/private_functions.h.
 */

#include "semantic/semantic_verify/semantic_verify.h"

#include "geometric/Plane/Plane.h"
#include "loop_closing/loop_closing.h"
#include "map/map.h"

#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>
#include "types/types.h"
#include "semantic/room/objects/RoomClass.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus collectConsecutiveAnchors(
    const std::vector<SemanticMergeRoomEvidence> &survivingRooms_in,
    const std::vector<SemanticMergeRoomEvidence> &absorbedRooms_in,
    std::vector<ConsecutiveAnchorPair>           &consecutiveAnchors_out)
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
        const std::map<std::string, const SemanticMergeRoomEvidence *>::iterator
            match = survivingByTag.find(room.context.roomTag);
        if (match != survivingByTag.end())
        {
            ConsecutiveAnchorPair pair;
            pair.p_surviving = match->second;
            pair.p_absorbed  = &room;
            pairs.push_back(pair);
        }
    }
    consecutiveAnchors_out = pairs;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
