/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            exportRoomContextFromCurrentMap.cc
 *
 * @brief           Implements Atlas::exportRoomContextFromCurrentMap(),
 *                  declared in atlas/objects/AtlasClass.h.
 */

#include "../private_functions/private_functions.h"
#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasClass.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/Floor/Floor.h"
#include "scene_understanding/scene_graph/Floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/Passage/Passage.h"
#include "scene_understanding/scene_graph/Passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/Passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/Room/Room.h"
#include "scene_understanding/scene_graph/Room/objects/RoomStatusEnum.h"
#include "scene_understanding/scene_graph/SceneGraph/objects/SceneGraphStatusEnum.h"
#include <rclcpp/logging.hpp>
#include <shared_mutex>

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::exportRoomContextFromCurrentMap()
{
    if (!p_activeMap)
        return AtlasStatus::ATLAS_STATUS_SUCCESS;

    unsigned long mapIdValue{};
    if (p_activeMap->getId(mapIdValue) != map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    const long unsigned int mapId = static_cast<long unsigned int>(mapIdValue);
    /* Export BOTH confirmed detected rooms AND candidate/marker-based rooms.
     * Candidate rooms (prospective/provisional) may not have full wall loops
     * yet but still carry spatial identity needed for cross-restart matching.
     */
    std::vector<semantic::Room *>       rooms{};
    /* The tracking thread may call this without the semantic update lock: the
     * guard keeps the scene graph from freeing the rooms read below. */
    std::shared_lock<std::shared_mutex> sceneGraphReclaimGuard{};
    if ((sceneGraph.acquireReclaimGuard(sceneGraphReclaimGuard) !=
         scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS) ||
        (sceneGraph.getMapDetectedRoomsInIdOrder(mapIdValue, rooms) !=
         scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS))
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapDetectedRoomsInIdOrder found no scene map for "
                     "the map; continuing without its detected rooms.",
                     __func__);
    }
    std::vector<semantic::Room *> candidateRooms{};
    if (sceneGraph.getMapCandidateRoomsInIdOrder(mapIdValue, candidateRooms) !=
        scene_graph::SceneGraphStatus::SCENE_GRAPH_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMapCandidateRoomsInIdOrder found no scene map "
                     "for the map; continuing without its candidate rooms.",
                     __func__);
    }
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());

    if (rooms.empty())
        return AtlasStatus::ATLAS_STATUS_SUCCESS;

    std::vector<semantic::RoomContextSnapshot> snapshots;
    snapshots.reserve(rooms.size());

    for (semantic::Room *p_room : rooms)
    {
        bool roomIsBad{};
        if (!(!p_room) && p_room->isBad(roomIsBad) !=
                              semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_room || roomIsBad)
            continue;

        semantic::RoomContextSnapshot snap;
        captureRoomContext(p_room, snap);
        bool roomHasPreviouslyVisited{};
        if (p_room->hasPreviouslyVisited(roomHasPreviouslyVisited) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasPreviouslyVisited returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        snap.wasPreviouslyVisited = roomHasPreviouslyVisited;

        snapshots.push_back(snap);
    }

    const std::size_t exportedRoomCount = snapshots.size();
    {
        std::lock_guard<std::mutex> contextLock(roomContextMutex);
        roomContextHistory[mapId] = std::move(snapshots);
    }

    /* Record the departure room for mission-chain tracing: the room the UAV
     * was following when this map was stranded. */
    int departureRoomId{};
    if (getCurrentSemanticRoomIdentity(departureRoomId) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentSemanticRoomIdentity returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    if (departureRoomId >= 0)
    {
        /* The pose at departure is the last key frame's: the tracker's own
         * frame is not handed to the atlas, and the lost frame has no pose. */
        std::vector<keyframe::KeyFrame *> mapKeyFrames{};
        if (p_activeMap->getAllKeyFrames(mapKeyFrames) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        keyframe::KeyFrame *p_lastKeyFrame = nullptr;
        for (keyframe::KeyFrame *p_keyFrame : mapKeyFrames)
        {
            bool keyFrameIsBad = true;
            if (p_keyFrame != nullptr &&
                p_keyFrame->isBad(keyFrameIsBad) !=
                    keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!keyFrameIsBad && (p_lastKeyFrame == nullptr ||
                                   p_keyFrame->id > p_lastKeyFrame->id))
            {
                p_lastKeyFrame = p_keyFrame;
            }
        }
        Eigen::Vector3d upDirection_world{};
        bool            isUpFromImuGravity{};
        if (p_activeMap->getUpDirection(upDirection_world,
                                        isUpFromImuGravity) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getUpDirection returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        types::SystemParams *p_params = nullptr;
        if (types::SystemParams::getParams(p_params) !=
            types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParams returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }

        for (semantic::Room *p_room : rooms)
        {
            bool roomIsBad2{};
            if ((p_room != nullptr) &&
                p_room->isBad(roomIsBad2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int roomId4{};
            if ((p_room != nullptr && !roomIsBad2) &&
                p_room->getId(roomId4) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room != nullptr && !roomIsBad2 && roomId4 == departureRoomId)
            {
                /* A passage whose whole opening was in view may have let
                 * the camera see the room beyond; that room becomes the
                 * second anchor candidate. */
                semantic::Passage *p_passageInView = nullptr;
                semantic::Room    *p_roomBeyondPassage =
                    findRoomBeyondPassageInView(
                        rooms,
                        p_room,
                        p_lastKeyFrame,
                        upDirection_world,
                        p_params->pointcloud.distanceThresh,
                        p_passageInView);
                int passageInViewId = -1;
                int roomBeyondId    = -1;
                if ((p_passageInView != nullptr &&
                     p_passageInView->getId(passageInViewId) !=
                         semantic::PassageStatus::PASSAGE_STATUS_SUCCESS) ||
                    (p_roomBeyondPassage != nullptr &&
                     p_roomBeyondPassage->getId(roomBeyondId) !=
                         semantic::RoomStatus::ROOM_STATUS_SUCCESS))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status "
                                 "although it cannot fail; continuing as "
                                 "before.",
                                 __func__);
                }
                RCLCPP_INFO(rclcpp::get_logger("vs_graphs"),
                            "[Atlas] Departure anchor: final_room=%d "
                            "key_frame=%ld passage_in_view=%d "
                            "room_beyond_passage=%d",
                            departureRoomId,
                            p_lastKeyFrame != nullptr
                                ? static_cast<long>(p_lastKeyFrame->id)
                                : -1L,
                            passageInViewId,
                            roomBeyondId);
                if (p_activeMap->setFinalRoom(p_room, p_roomBeyondPassage) !=
                    map::MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setFinalRoom returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                break;
            }
        }
    }

    std::cout << "[Atlas] Exported room context: " << exportedRoomCount
              << " rooms (mapId: " << mapId << ")" << std::endl;

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
