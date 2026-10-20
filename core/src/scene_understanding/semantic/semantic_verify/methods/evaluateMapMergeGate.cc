/*!
 * @file            evaluateMapMergeGate.cc
 *
 * @brief           Implements SemanticVerify::evaluateMapMergeGate(), declared
 *                  in Semantic/SemanticVerify.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingStatusEnum.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantic/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/room/room.h"

#include "mapping/geometric/Plane/Plane.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "map_state/map/map.h"

#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>
#include "configuration/types/types.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"

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

SemanticVerifyStatus SemanticVerify::evaluateMapMergeGate(
    map::Map                  *p_survivingMap_in,
    map::Map                  *p_absorbedMap_in,
    const g2o::Sim3            &mergeTransform_absorbedToSurviving_in,
    SemanticMergeGateResult    &result_out,
    const SemanticVerifyConfig &configuration_in)
{
    SemanticMergeGateResult result;
    if (p_survivingMap_in == nullptr || p_absorbedMap_in == nullptr ||
        p_survivingMap_in == p_absorbedMap_in)
    {
        result.decision = SemanticMergeDecision::REJECT;
        result.reason   = SemanticMergeReason::INVALID_INPUT;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    bool isVerified{};
    if (verifyLoopMergeFloors(p_survivingMap_in,
                              p_absorbedMap_in,
                              mergeTransform_absorbedToSurviving_in,
                              result.floorDecision,
                              isVerified) !=
        loop_closing::LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: verifyLoopMergeFloors returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isVerified)
    {
        result.decision = result.floorDecision == "REJECTED"
                              ? SemanticMergeDecision::REJECT
                              : SemanticMergeDecision::DEFER;
        result.reason   = result.decision == SemanticMergeDecision::REJECT
                              ? SemanticMergeReason::FLOOR_CONTRADICTION
                              : SemanticMergeReason::FLOOR_EVIDENCE_MISSING;
        result_out      = result;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    std::vector<SemanticMergeRoomEvidence> survivingRooms;
    std::vector<semantic::Room *>          survivingMapAllRooms{};
    if (p_survivingMap_in->getAllRooms(survivingMapAllRooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Room *p_room : survivingMapAllRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Room::RoomVariant roomVariant{};
        if ((p_room != nullptr && !roomIsBad) &&
            p_room->getRoomVariant(roomVariant) !=
                RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad &&
            roomVariant == Room::RoomVariant::ROOM)
        {
            SemanticMergeRoomEvidence evidence{};
            if (copyMergeRoomEvidence(p_room, configuration_in, evidence) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: copyMergeRoomEvidence returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            survivingRooms.push_back(evidence);
        }
    }
    std::vector<SemanticMergeRoomEvidence> absorbedRooms;
    std::vector<semantic::Room *>          absorbedMapAllRooms{};
    if (p_absorbedMap_in->getAllRooms(absorbedMapAllRooms) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (Room *p_room : absorbedMapAllRooms)
    {
        bool roomIsBad2{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad2) != RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Room::RoomVariant roomVariant2{};
        if ((p_room != nullptr && !roomIsBad2) &&
            p_room->getRoomVariant(roomVariant2) !=
                RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad2 &&
            roomVariant2 == Room::RoomVariant::ROOM)
        {
            SemanticMergeRoomEvidence evidence2{};
            if (copyMergeRoomEvidence(p_room, configuration_in, evidence2) !=
                SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: copyMergeRoomEvidence returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            absorbedRooms.push_back(evidence2);
        }
    }
    SemanticMergeGateResult result2{};
    if (evaluateMergeAlignment(survivingRooms,
                               absorbedRooms,
                               mergeTransform_absorbedToSurviving_in,
                               result2,
                               configuration_in) !=
        SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: evaluateMergeAlignment returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    result               = result2;
    result.floorDecision = "ACCEPTED";
    result_out           = result;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
