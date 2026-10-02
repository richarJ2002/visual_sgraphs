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
 * @file            postLoad.cc
 *
 * @brief           Implements KeyFrame::postLoad(), declared in KeyFrame.h.
 */

#include "KeyFrame.h"

#include "ImuTypes.h"
#include "SerializationUtils.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

KeyFrameStatus KeyFrame::postLoad(
    std::map<long unsigned int, KeyFrame *> &keyFrameId_in,
    std::map<long unsigned int, MapPoint *> &mapPointId_in,
    std::map<unsigned int, camera_models::geometriccamera::GeometricCamera *>
        &cameraId_in)
{
    // Rebuild the empty variables

    // Pose
    if (setPose(poseTcw) != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setPose returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    poseTrl = poseTlr.inverse();

    // Reference reconstruction
    // Each MapPoint sight from this KeyFrame
    mapPoints.clear();
    mapPoints.resize(keyPointCount);
    for (int keyPointIndex = 0; keyPointIndex < keyPointCount; ++keyPointIndex)
    {
        if (backupMapPointsId[keyPointIndex] != -1)
            mapPoints[keyPointIndex] =
                mapPointId_in[backupMapPointsId[keyPointIndex]];
        else
            mapPoints[keyPointIndex] = static_cast<MapPoint *>(nullptr);
    }

    // Conected KeyFrames with him weight
    connectedKeyFrameWeights.clear();
    for (std::map<long unsigned int, int>::const_iterator
             mergeEdgeIdIt = backupConnectedKeyFrameIdWeights.begin(),
             end           = backupConnectedKeyFrameIdWeights.end();
         mergeEdgeIdIt != end;
         ++mergeEdgeIdIt)
    {
        KeyFrame *p_keyFrame = keyFrameId_in[mergeEdgeIdIt->first];
        connectedKeyFrameWeights[p_keyFrame] = mergeEdgeIdIt->second;
    }

    // Restore parent KeyFrame
    if (backupParentId >= 0)
        p_parent = keyFrameId_in[backupParentId];

    // KeyFrame childrens
    childrens.clear();
    for (std::vector<long unsigned int>::const_iterator
             mergeEdgeIdIt = backupChildrensId.begin(),
             end           = backupChildrensId.end();
         mergeEdgeIdIt != end;
         ++mergeEdgeIdIt)
    {
        childrens.insert(keyFrameId_in[*mergeEdgeIdIt]);
    }

    // Loop edge KeyFrame
    loopEdges.clear();
    for (std::vector<long unsigned int>::const_iterator
             mergeEdgeIdIt = backupLoopEdgesId.begin(),
             end           = backupLoopEdgesId.end();
         mergeEdgeIdIt != end;
         ++mergeEdgeIdIt)
    {
        loopEdges.insert(keyFrameId_in[*mergeEdgeIdIt]);
    }

    // Merge edge KeyFrame
    mergeEdges.clear();
    for (std::vector<long unsigned int>::const_iterator
             mergeEdgeIdIt = backupMergeEdgesId.begin(),
             end           = backupMergeEdgesId.end();
         mergeEdgeIdIt != end;
         ++mergeEdgeIdIt)
    {
        mergeEdges.insert(keyFrameId_in[*mergeEdgeIdIt]);
    }

    // Camera data
    // An absent camera stays null (the default constructor leaves both
    // pointers unset).
    if (backupCameraId != NO_SAVED_ID<unsigned int>)
    {
        p_camera = cameraId_in[backupCameraId];
    }
    else
    {
        p_camera = nullptr;
        std::cout << "ERROR: There is not a main camera in KF " << id
                  << std::endl;
    }
    if (backupCamera2Id != NO_SAVED_ID<unsigned int>)
    {
        p_camera2 = cameraId_in[backupCamera2Id];
    }
    else
    {
        p_camera2 = nullptr;
    }

    // Inertial data
    if (backupPrevKFId != -1)
    {
        p_prevKF = keyFrameId_in[backupPrevKFId];
    }
    if (backupNextKFId != -1)
    {
        p_nextKF = keyFrameId_in[backupNextKFId];
    }
    p_imuPreintegrated = &backupImuPreintegrated;

    // Remove all backup container
    backupMapPointsId.clear();
    backupConnectedKeyFrameIdWeights.clear();
    backupChildrensId.clear();
    backupLoopEdgesId.clear();

    if (updateBestCovisibles() != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: updateBestCovisibles returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
