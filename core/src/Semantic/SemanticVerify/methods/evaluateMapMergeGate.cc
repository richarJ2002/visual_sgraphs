

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

SemanticMergeGateResult SemanticVerify::evaluateMapMergeGate(
    core::Map                  *p_survivingMap_in,
    core::Map                  *p_absorbedMap_in,
    const g2o::Sim3            &transform_absorbedToSurviving_in,
    const SemanticVerifyConfig &config_in)
{
    SemanticMergeGateResult result;
    if (p_survivingMap_in == nullptr || p_absorbedMap_in == nullptr ||
        p_survivingMap_in == p_absorbedMap_in)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::INVALID_INPUT;
        return result;
    }

    if (!verifyLoopMergeFloors(p_survivingMap_in,
                               p_absorbedMap_in,
                               transform_absorbedToSurviving_in,
                               result.floorDecision))
    {
        result.decision = result.floorDecision == "REJECTED"
                              ? SemanticMergeDecision::REJECT
                              : SemanticMergeDecision::DEFER;
        result.reason   = result.decision == SemanticMergeDecision::REJECT
                              ? SemanticMergeReason::FLOOR_CONTRADICTION
                              : SemanticMergeReason::FLOOR_EVIDENCE_MISSING;
        return result;
    }

    std::vector<SemanticMergeRoomEvidence> survivingRooms;
    for (Room *p_room : p_survivingMap_in->getAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::RoomVariant::ROOM)
        {
            survivingRooms.push_back(copyMergeRoomEvidence(p_room, config_in));
        }
    }
    std::vector<SemanticMergeRoomEvidence> absorbedRooms;
    for (Room *p_room : p_absorbedMap_in->getAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() &&
            p_room->getRoomVariant() == Room::RoomVariant::ROOM)
        {
            absorbedRooms.push_back(copyMergeRoomEvidence(p_room, config_in));
        }
    }
    result               = evaluateMergeAlignment(survivingRooms,
                                    absorbedRooms,
                                    transform_absorbedToSurviving_in,
                                    config_in);
    result.floorDecision = "ACCEPTED";
    return result;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
