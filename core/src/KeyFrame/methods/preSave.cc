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

#include "KeyFrame.h"

#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void KeyFrame::preSave(
    set<KeyFrame *>                                        &keyFrames_in,
    set<MapPoint *>                                        &mapPoints_in,
    set<camera_models::geometriccamera::GeometricCamera *> &cameras_in)
{
    // Save the id of each MapPoint in this KF, there can be null pointer in the
    // vector
    backupMapPointsId.clear();
    backupMapPointsId.reserve(keyPointCount);
    for (int keyPointIndex = 0; keyPointIndex < keyPointCount; ++keyPointIndex)
    {

        if (mapPoints[keyPointIndex] &&
            mapPoints_in.find(mapPoints[keyPointIndex]) !=
                mapPoints_in.end()) // Checks if the element is not null
            backupMapPointsId.push_back(mapPoints[keyPointIndex]->id);
        else // If the element is null his value is -1 because all the id are
             // positives
            backupMapPointsId.push_back(-1);
    }
    // Save the id of each connected KF with it weight
    backupConnectedKeyFrameIdWeights.clear();
    for (std::map<KeyFrame *, int>::const_iterator
             connectionWeightIt = connectedKeyFrameWeights.begin(),
             end                = connectedKeyFrameWeights.end();
         connectionWeightIt != end;
         ++connectionWeightIt)
    {
        if (keyFrames_in.find(connectionWeightIt->first) != keyFrames_in.end())
            backupConnectedKeyFrameIdWeights[connectionWeightIt->first->id] =
                connectionWeightIt->second;
    }

    // Save the parent id
    backupParentId = -1;
    if (p_parent && keyFrames_in.find(p_parent) != keyFrames_in.end())
        backupParentId = p_parent->id;

    // Save the id of the childrens KF
    backupChildrensId.clear();
    backupChildrensId.reserve(childrens.size());
    for (KeyFrame *p_keyFrame : childrens)
    {
        if (keyFrames_in.find(p_keyFrame) != keyFrames_in.end())
            backupChildrensId.push_back(p_keyFrame->id);
    }

    // Save the id of the loop edge KF
    backupLoopEdgesId.clear();
    backupLoopEdgesId.reserve(loopEdges.size());
    for (KeyFrame *p_keyFrame : loopEdges)
    {
        if (keyFrames_in.find(p_keyFrame) != keyFrames_in.end())
            backupLoopEdgesId.push_back(p_keyFrame->id);
    }

    // Save the id of the merge edge KF
    backupMergeEdgesId.clear();
    backupMergeEdgesId.reserve(mergeEdges.size());
    for (KeyFrame *p_keyFrame : mergeEdges)
    {
        if (keyFrames_in.find(p_keyFrame) != keyFrames_in.end())
            backupMergeEdgesId.push_back(p_keyFrame->id);
    }

    // Camera data
    backupCameraId = -1;
    if (p_camera && cameras_in.find(p_camera) != cameras_in.end())
        backupCameraId = p_camera->getId();

    backupCamera2Id = -1;
    if (p_camera2 && cameras_in.find(p_camera2) != cameras_in.end())
        backupCamera2Id = p_camera2->getId();

    // Inertial data
    backupPrevKFId = -1;
    if (p_prevKF && keyFrames_in.find(p_prevKF) != keyFrames_in.end())
        backupPrevKFId = p_prevKF->id;

    backupNextKFId = -1;
    if (p_nextKF && keyFrames_in.find(p_nextKF) != keyFrames_in.end())
        backupNextKFId = p_nextKF->id;

    if (p_imuPreintegrated)
        backupImuPreintegrated.copyFrom(p_imuPreintegrated);
}

} // namespace core
} // namespace vs_graphs
