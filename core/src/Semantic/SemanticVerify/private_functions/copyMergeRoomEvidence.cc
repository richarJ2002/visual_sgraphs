

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

SemanticMergeRoomEvidence
    copyMergeRoomEvidence(const Room                 *p_room_in,
                          const SemanticVerifyConfig &config_in)
{
    SemanticMergeRoomEvidence evidence;
    evidence.context.roomId   = p_room_in->getId();
    evidence.context.roomTag  = p_room_in->getRoomTag();
    evidence.context.centroid = p_room_in->getCentroid();
    Floor *p_floor            = p_room_in->getFloor();
    evidence.context.floorId  = p_floor != nullptr ? p_floor->getId() : -1;
    evidence.walls =
        SemanticVerify::collectWallObservations(p_room_in, config_in);
    for (Passage *p_passage : p_room_in->getPassages())
    {
        if (p_passage == nullptr || p_passage->isBad())
        {
            continue;
        }
        PassageContext context;
        context.id              = p_passage->getId();
        context.passable        = p_passage->isPassable();
        context.centroid_World  = p_passage->getCentroid();
        context.width_m         = p_passage->getWidth();
        context.height_m        = p_passage->getHeight();
        context.isRecoveryProxy = p_passage->isRecoveryProxy();
        context.apertureValid   = std::isfinite(context.width_m) &&
                                std::isfinite(context.height_m) &&
                                context.width_m > 0.0 && context.height_m > 0.0;
        const Passage::KnownSideProvenance knownSide =
            p_passage->getKnownSideProvenance();
        context.hasKnownSideRoom = knownSide.pRoom != nullptr;
        if (context.hasKnownSideRoom)
        {
            context.knownSideRoomId = knownSide.pRoom->getId();
        }
        context.hasKnownSideDirection = knownSide.hasDirection();
        if (context.hasKnownSideDirection)
        {
            context.knownSideDirection_World = knownSide.direction_World;
        }
        const std::optional<int> farSideRoomId =
            p_passage->getProspectiveRoomId();
        context.hasFarSideRoom = farSideRoomId.has_value();
        if (farSideRoomId.has_value())
        {
            context.secondaryRoomId = *farSideRoomId;
        }
        evidence.context.passageContexts.push_back(context);
    }
    return evidence;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
