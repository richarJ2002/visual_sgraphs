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

void KeyFrame::PostLoad(
    map<long unsigned int, KeyFrame *> &mpKFid,
    map<long unsigned int, MapPoint *> &mpMPid,
    map<unsigned int, camera_models::geometriccamera::GeometricCamera *>
        &mpCamId)
{
    // Rebuild the empty variables

    // Pose
    setPose(poseTcw);

    poseTrl = poseTlr.inverse();

    // Reference reconstruction
    // Each MapPoint sight from this KeyFrame
    mapPoints.clear();
    mapPoints.resize(N);
    for (int i = 0; i < N; ++i)
    {
        if (backupMapPointsId[i] != -1)
            mapPoints[i] = mpMPid[backupMapPointsId[i]];
        else
            mapPoints[i] = static_cast<MapPoint *>(nullptr);
    }

    // Conected KeyFrames with him weight
    connectedKeyFrameWeights.clear();
    for (map<long unsigned int, int>::const_iterator
             it  = backupConnectedKeyFrameIdWeights.begin(),
             end = backupConnectedKeyFrameIdWeights.end();
         it != end;
         ++it)
    {
        KeyFrame *pKFi                 = mpKFid[it->first];
        connectedKeyFrameWeights[pKFi] = it->second;
    }

    // Restore parent KeyFrame
    if (backupParentId >= 0)
        p_parent = mpKFid[backupParentId];

    // KeyFrame childrens
    childrens.clear();
    for (vector<long unsigned int>::const_iterator
             it  = backupChildrensId.begin(),
             end = backupChildrensId.end();
         it != end;
         ++it)
    {
        childrens.insert(mpKFid[*it]);
    }

    // Loop edge KeyFrame
    loopEdges.clear();
    for (vector<long unsigned int>::const_iterator
             it  = backupLoopEdgesId.begin(),
             end = backupLoopEdgesId.end();
         it != end;
         ++it)
    {
        loopEdges.insert(mpKFid[*it]);
    }

    // Merge edge KeyFrame
    mergeEdges.clear();
    for (vector<long unsigned int>::const_iterator
             it  = backupMergeEdgesId.begin(),
             end = backupMergeEdgesId.end();
         it != end;
         ++it)
    {
        mergeEdges.insert(mpKFid[*it]);
    }

    // Camera data
    if (backupCameraId >= 0)
    {
        p_camera = mpCamId[backupCameraId];
    }
    else
    {
        cout << "ERROR: There is not a main camera in KF " << mnId << endl;
    }
    if (backupCamera2Id >= 0)
    {
        p_camera2 = mpCamId[backupCamera2Id];
    }

    // Inertial data
    if (backupPrevKFId != -1)
    {
        p_prevKF = mpKFid[backupPrevKFId];
    }
    if (backupNextKFId != -1)
    {
        p_nextKF = mpKFid[backupNextKFId];
    }
    p_imuPreintegrated = &backupImuPreintegrated;

    // Remove all backup container
    backupMapPointsId.clear();
    backupConnectedKeyFrameIdWeights.clear();
    backupChildrensId.clear();
    backupLoopEdgesId.clear();

    updateBestCovisibles();
}

} // namespace core
} // namespace vs_graphs
