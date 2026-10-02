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
 * @file            applyScaledRotation.cc
 *
 * @brief           Implements Map::applyScaledRotation(), declared in Map.h.
 */

#include "Geometric/Plane.h"
#include "Geometric/PlaneStatus.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Semantic/Floor.h"
#include "Semantic/FloorStatus.h"
#include "Semantic/Marker.h"
#include "Semantic/MarkerStatus.h"
#include "Semantic/Passage.h"
#include "Semantic/PassageStatus.h"
#include "Semantic/Room.h"
#include "Semantic/RoomStatus.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"

#include <algorithm>
#include <iterator>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapStatus Map::applyScaledRotation(
    const Sophus::SE3f &alignmentPose_oldWorldToNewWorld_in,
    const float         alignmentScale_in,
    const bool          isScaledVelocity_in)
{
    std::unique_lock<std::mutex> lock(mapMutex);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f alignmentPose_oldWorldToNewWorld =
        alignmentPose_oldWorldToNewWorld_in;
    Eigen::Matrix3f alignmentRotation_oldWorldToNewWorld =
        alignmentPose_oldWorldToNewWorld.rotationMatrix();
    Eigen::Vector3f alignmentTranslation_oldWorldToNewWorld =
        alignmentPose_oldWorldToNewWorld.translation();

    const g2o::Sim3 alignmentTransform_oldWorldToNewWorld(
        alignmentRotation_oldWorldToNewWorld.cast<double>(),
        alignmentTranslation_oldWorldToNewWorld.cast<double>(),
        static_cast<double>(alignmentScale_in));

    for (std::set<KeyFrame *>::iterator sit = keyFrames.begin();
         sit != keyFrames.end();
         sit++)
    {
        KeyFrame    *p_keyFrame = *sit;
        Sophus::SE3f cameraPose_cameraToWorld{};
        if (p_keyFrame->getPoseInverse(cameraPose_cameraToWorld) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        cameraPose_cameraToWorld.translation() *= alignmentScale_in;
        Sophus::SE3f keyFramePose_cameraToNewWorld =
            alignmentPose_oldWorldToNewWorld * cameraPose_cameraToWorld;
        Sophus::SE3f keyFramePose_newWorldToCamera =
            keyFramePose_cameraToNewWorld.inverse();
        if (p_keyFrame->setPose(keyFramePose_newWorldToCamera) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f Vw{};
        if (p_keyFrame->getVelocity(Vw) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (!isScaledVelocity_in)
        {
            if (p_keyFrame->setVelocity(alignmentRotation_oldWorldToNewWorld *
                                        Vw) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else
        {
            if (p_keyFrame->setVelocity(alignmentRotation_oldWorldToNewWorld *
                                        Vw * alignmentScale_in) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setVelocity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (std::set<MapPoint *>::iterator sit = mapPoints.begin();
         sit != mapPoints.end();
         sit++)
    {
        MapPoint       *p_mapPoint = *sit;
        Eigen::Vector3f mapPointWorldPos{};
        if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapPoint->setWorldPos(alignmentScale_in *
                                        alignmentRotation_oldWorldToNewWorld *
                                        mapPointWorldPos +
                                    alignmentTranslation_oldWorldToNewWorld) !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWorldPos returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapPoint->updateNormalAndDepth() !=
            MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateNormalAndDepth returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    for (geometric::Plane *p_plane : planes)
    {
        bool planeIsBad{};
        if ((p_plane != nullptr) &&
            p_plane->isBad(planeIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane != nullptr && !planeIsBad)
        {
            if (p_plane->applyTransform(
                    alignmentTransform_oldWorldToNewWorld) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: applyTransform returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (semantic::Marker *p_marker : markers)
    {
        if (p_marker != nullptr)
        {
            if (p_marker->applyTransform(
                    alignmentTransform_oldWorldToNewWorld) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: applyTransform returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (vs_graphs::core::semantic::Passage *p_passage : passages)
    {
        if (p_passage != nullptr)
        {
            if (p_passage->applyTransform(
                    alignmentTransform_oldWorldToNewWorld) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: applyTransform returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (semantic::Room *p_room : detectedRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad)
        {
            if (p_room->applyTransform(alignmentTransform_oldWorldToNewWorld) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: applyTransform returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (semantic::Room *p_room : markerBasedRooms)
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
        if (p_room != nullptr && !roomIsBad2)
        {
            if (p_room->applyTransform(alignmentTransform_oldWorldToNewWorld) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: applyTransform returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (semantic::Floor *p_floor : floors)
    {
        if (p_floor != nullptr)
        {
            if (p_floor->applyTransform(
                    alignmentTransform_oldWorldToNewWorld) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: applyTransform returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    for (std::vector<Eigen::Vector3d> &skeletonCluster_world :
         skeletonClusterPoints)
    {
        for (Eigen::Vector3d &clusterPoint_world_m : skeletonCluster_world)
        {
            clusterPoint_world_m =
                alignmentTransform_oldWorldToNewWorld.map(clusterPoint_world_m);
        }
    }

    for (std::pair<Eigen::Vector3d, Eigen::Vector3d> &skeletonEdge_world :
         skeletonEdges)
    {
        skeletonEdge_world.first =
            alignmentTransform_oldWorldToNewWorld.map(skeletonEdge_world.first);
        skeletonEdge_world.second = alignmentTransform_oldWorldToNewWorld.map(
            skeletonEdge_world.second);
    }

    mapChange++;
    worldFrameEpoch++;

    return MapStatus::MAP_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
