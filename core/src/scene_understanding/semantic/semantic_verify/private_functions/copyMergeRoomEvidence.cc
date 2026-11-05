/*!
 * @file            copyMergeRoomEvidence.cc
 *
 * @brief           Implements copyMergeRoomEvidence(), declared in
 *                  Semantic/SemanticVerify/private_functions.h.
 */

#include "scene_understanding/semantic/floor/floor.h"
#include "scene_understanding/semantic/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/semantic/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/semantic/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantic/passage/passage.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"
#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"

#include "map_state/map/map.h"
#include "mapping/geometric/Plane/Plane.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"

#include "configuration/types/types.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"
#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <rclcpp/logging.hpp>
#include <set>

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

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
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    evidence.context.roomId = room_inId;
    Eigen::Vector3d room_inCentroid{};
    if (p_room_in->getCentroid(room_inCentroid) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    evidence.context.centroid = room_inCentroid;
    Floor *p_floor            = nullptr;
    if (p_room_in->getFloor(p_floor) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getFloor returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int floorId2{};
    if ((p_floor != nullptr) &&
        p_floor->getId(floorId2) != FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    evidence.context.floorId = p_floor != nullptr ? floorId2 : -1;
    if (SemanticVerify::collectFloorObservation(p_room_in,
                                                evidence.floorPlane_world) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectFloorObservation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<VerifyWallObservation> observations{};
    if (SemanticVerify::collectWallObservations(p_room_in,
                                                configuration_in,
                                                observations) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectWallObservations returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    evidence.walls = observations;
    std::vector<vs_graphs::core::semantic::Passage *> room_inPassages{};
    if (p_room_in->getPassages(room_inPassages) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Passage *p_passage : room_inPassages)
    {
        bool passageIsBad{};
        if (!(p_passage == nullptr) &&
            p_passage->isBad(passageIsBad) !=
                PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_passage == nullptr || passageIsBad)
        {
            continue;
        }
        evidence.context.passageContexts.push_back(
            copyPassageContext(p_passage));
    }
    evidence_out = evidence;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
