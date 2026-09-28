

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

SemanticVerifyStatus
    copyMergeRoomEvidence(const Room                 *p_room_in,
                          const SemanticVerifyConfig &configuration_in,
                          SemanticMergeRoomEvidence  &evidence_out)
{
    SemanticMergeRoomEvidence evidence;
    int                       room_inId{};
    if (p_room_in->getId(room_inId) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    evidence.context.roomId = room_inId;
    std::string room_inRoomTag{};
    if (p_room_in->getRoomTag(room_inRoomTag) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getRoomTag cannot fail; continue as before.
    }
    evidence.context.roomTag = room_inRoomTag;
    Eigen::Vector3d room_inCentroid{};
    if (p_room_in->getCentroid(room_inCentroid) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getCentroid cannot fail; continue as before.
    }
    evidence.context.centroid = room_inCentroid;
    Floor *p_floor            = nullptr;
    if (p_room_in->getFloor(p_floor) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getFloor cannot fail; continue as before.
    }
    int floorId2{};
    if ((p_floor != nullptr) &&
        p_floor->getId(floorId2) != FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    evidence.context.floorId = p_floor != nullptr ? floorId2 : -1;
    std::vector<VerifyWallObservation> observations{};
    if (SemanticVerify::collectWallObservations(p_room_in,
                                                configuration_in,
                                                observations) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        // collectWallObservations cannot fail; continue as before.
    }
    evidence.walls = observations;
    std::vector<vs_graphs::core::semantic::Passage *> room_inPassages{};
    if (p_room_in->getPassages(room_inPassages) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getPassages cannot fail; continue as before.
    }
    for (Passage *p_passage : room_inPassages)
    {
        bool passageIsBad{};
        if (!(p_passage == nullptr) &&
            p_passage->isBad(passageIsBad) !=
                PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_passage == nullptr || passageIsBad)
        {
            continue;
        }
        PassageContext context;
        int            passageId{};
        if (p_passage->getId(passageId) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        context.id = passageId;
        bool passageIsPassable{};
        if (p_passage->isPassable(passageIsPassable) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // isPassable cannot fail; continue as before.
        }
        context.isPassable = passageIsPassable;
        Eigen::Vector3d passageCentroid{};
        if (p_passage->getCentroid(passageCentroid) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }
        context.centroid_World = passageCentroid;
        double passageWidth{};
        if (p_passage->getWidth(passageWidth) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getWidth cannot fail; continue as before.
        }
        context.width_m = passageWidth;
        double passageHeight{};
        if (p_passage->getHeight(passageHeight) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getHeight cannot fail; continue as before.
        }
        context.height_m = passageHeight;
        bool passageIsRecoveryProxy{};
        if (p_passage->isRecoveryProxy(passageIsRecoveryProxy) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // isRecoveryProxy cannot fail; continue as before.
        }
        context.isRecoveryProxy = passageIsRecoveryProxy;
        context.isApertureValid =
            std::isfinite(context.width_m) && std::isfinite(context.height_m) &&
            context.width_m > 0.0 && context.height_m > 0.0;
        Passage::KnownSideProvenance knownSide{};
        if (p_passage->getKnownSideProvenance(knownSide) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getKnownSideProvenance cannot fail; continue as before.
        }
        context.hasKnownSideRoom = knownSide.p_room != nullptr;
        if (context.hasKnownSideRoom)
        {
            int id2{};
            if (knownSide.p_room->getId(id2) != RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            context.knownSideRoomId = id2;
        }
        bool knownSideHasDirection{};
        if (knownSide.hasDirection(knownSideHasDirection) !=
            KnownSideProvenanceStatus::KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            // hasDirection cannot fail; continue as before.
        }
        context.hasKnownSideDirection = knownSideHasDirection;
        if (context.hasKnownSideDirection)
        {
            context.knownSideDirection_World = knownSide.direction_World;
        }
        std::optional<int> farSideRoomId{};
        if (p_passage->getProspectiveRoomId(farSideRoomId) !=
            PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            // getProspectiveRoomId cannot fail; continue as before.
        }
        context.hasFarSideRoom = farSideRoomId.has_value();
        if (farSideRoomId.has_value())
        {
            context.secondaryRoomId = *farSideRoomId;
        }
        evidence.context.passageContexts.push_back(context);
    }
    evidence_out = evidence;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
