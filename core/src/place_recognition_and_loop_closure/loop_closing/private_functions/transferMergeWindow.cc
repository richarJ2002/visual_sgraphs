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
 * @file            transferMergeWindow.cc
 *
 * @brief           Implements transferMergeWindow(), declared in
 *                  LoopClosing/private_functions.h.
 */

#include "place_recognition_and_loop_closure/loop_closing/private_functions/private_functions.h"

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "map_state/map_point/map_point.h"
#include "map_state/map_point/objects/MapPointStatusEnum.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void transferMergeWindow(
    atlas::Atlas                          *p_atlas_inout,
    keyframe::KeyFrame                    *p_currentKeyFrame_in,
    map::Map                              *p_currentMap_inout,
    map::Map                              *p_mergeMap_inout,
    const std::set<keyframe::KeyFrame *>  &mergeConnectedKeyFrames_in,
    const std::set<map_point::MapPoint *> &mapPointMerges_in)
{
    /*!
     * Lock both maps with deadlock-safe acquisition while ownership moves
     */
    std::scoped_lock mapLocks(p_currentMap_inout->mapUpdateMutex,
                              p_mergeMap_inout->mapUpdateMutex);

    /* ------------------------------------------------------------------ *
     * SECTION 11 - TRANSFER CORRECTED KEYFRAMES
     *
     * Every corrected merge-map keyframe is:
     *
     *      1. Pose corrected.
     *      2. Assigned to the current map.
     *      3. Added to the current-map container.
     *      4. Removed from the merge-map container.
     * ------------------------------------------------------------------ */

    /* For every keyframe in merge map, iterate through and transfer */
    for (keyframe::KeyFrame *p_keyFrame : mergeConnectedKeyFrames_in)
    {
        /* Skip invalud keyframes */
        bool keyFrameIsBad4{};
        if (!(!p_keyFrame) &&
            p_keyFrame->isBad(keyFrameIsBad4) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_keyFrameMap2 = nullptr;
        if (!(!p_keyFrame || keyFrameIsBad4) &&
            p_keyFrame->getMap(p_keyFrameMap2) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad4 || p_keyFrameMap2 != p_mergeMap_inout)
        {
            continue;
        }

        /* Store the old pose of the keyframe */
        Sophus::SE3f keyFramePose2{};
        if (p_keyFrame->getPose(keyFramePose2) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        p_keyFrame->tcwBefMerge = keyFramePose2;
        Sophus::SE3f keyFramePoseInverse{};
        if (p_keyFrame->getPoseInverse(keyFramePoseInverse) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_keyFrame->twcBefMerge = keyFramePoseInverse;

        /* Apply corrected world-to-camera pose in the current-map frame */
        if (p_keyFrame->setPose(p_keyFrame->tcwMerge) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setPose returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Change keyframe's internal owning-map pointer to current map */
        if (p_keyFrame->updateMap(p_currentMap_inout) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Record which current keyframe triggered this merge correction */
        p_keyFrame->mergeCorrectedKeyFrameId = p_currentKeyFrame_in->id;

        /* Insert the same keyframe pointer into surviving map container */
        if (p_currentMap_inout->addKeyFrame(p_keyFrame) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Remove the keyframe pointer from the old merge-map container */
        if (p_mergeMap_inout->eraseKeyFrame(p_keyFrame) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseKeyFrame returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* If there is IMU, add velocity */
        bool currentMapIsImuInitialized2{};
        if (p_currentMap_inout->isImuInitialized(currentMapIsImuInitialized2) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isImuInitialized returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (currentMapIsImuInitialized2)
        {
            if (p_keyFrame->setVelocity(p_keyFrame->vwbMerge) !=
                keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setVelocity returned a failure status although it "
                    "cannot fail; continuing as before.",
                    __func__);
            }
        }
    }

    /* ------------------------------------------------------------------ *
     * SECTION 12 - TRANSFER CORRECTED MAP POINTS
     *
     * The imported landmarks have already been transformed into the
     * current-map frame and therefore only require ownership transfer.
     * ------------------------------------------------------------------ */

    /* Iterate over every merge-map point selected for transfer */
    for (map_point::MapPoint *p_currentMapPoint : mapPointMerges_in)
    {
        /* Skip null, invalid, or no-longer merge-owned map points */
        bool currentMapPointIsBad2{};
        if (!(!p_currentMapPoint) &&
            p_currentMapPoint->isBad(currentMapPointIsBad2) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        map::Map *p_currentMapPointMap2 = nullptr;
        if (!(!p_currentMapPoint || currentMapPointIsBad2) &&
            p_currentMapPoint->getMap(p_currentMapPointMap2) !=
                map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_currentMapPoint || currentMapPointIsBad2 ||
            p_currentMapPointMap2 != p_mergeMap_inout)
        {
            continue;
        }

        /* Apply position expressed in the surviving current-map frame */
        if (p_currentMapPoint->setWorldPos(p_currentMapPoint->posMerge) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setWorldPos returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Apply the normal rotated into the surviving current-map frame */
        if (p_currentMapPoint->setNormalVector(
                p_currentMapPoint->normalVectorMerge) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setNormalVector returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Change the map point's internal owner to the current map */
        if (p_currentMapPoint->updateMap(p_currentMap_inout) !=
            map_point::MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateMap returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        /* Register the same map-point pointer in the surviving map */
        if (p_currentMap_inout->addMapPoint(p_currentMapPoint) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: addMapPoint returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Remove the map-point pointer from the obsolete merge map */
        if (p_mergeMap_inout->eraseMapPoint(p_currentMapPoint) !=
            map::MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: eraseMapPoint returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Set the map to be the current map */
    if (p_atlas_inout->changeMap(p_currentMap_inout) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: changeMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Incrase index tracking the amount of times the maps been changed */
    if (p_currentMap_inout->increaseChangeIndex() !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseChangeIndex returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
}

} /* namespace core */
} /* namespace vs_graphs */
