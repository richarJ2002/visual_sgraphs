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

void KeyFrame::PreSave(
    set<KeyFrame *>                                        &spKF,
    set<MapPoint *>                                        &spMP,
    set<camera_models::geometriccamera::GeometricCamera *> &spCam)
{
    // Save the id of each MapPoint in this KF, there can be null pointer in the
    // vector
    backupMapPointsId.clear();
    backupMapPointsId.reserve(N);
    for (int i = 0; i < N; ++i)
    {

        if (mapPoints[i] && spMP.find(mapPoints[i]) !=
                                spMP.end()) // Checks if the element is not null
            backupMapPointsId.push_back(mapPoints[i]->mnId);
        else // If the element is null his value is -1 because all the id are
             // positives
            backupMapPointsId.push_back(-1);
    }
    // Save the id of each connected KF with it weight
    backupConnectedKeyFrameIdWeights.clear();
    for (std::map<KeyFrame *, int>::const_iterator
             it  = connectedKeyFrameWeights.begin(),
             end = connectedKeyFrameWeights.end();
         it != end;
         ++it)
    {
        if (spKF.find(it->first) != spKF.end())
            backupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
    }

    // Save the parent id
    backupParentId = -1;
    if (p_parent && spKF.find(p_parent) != spKF.end())
        backupParentId = p_parent->mnId;

    // Save the id of the childrens KF
    backupChildrensId.clear();
    backupChildrensId.reserve(childrens.size());
    for (KeyFrame *pKFi : childrens)
    {
        if (spKF.find(pKFi) != spKF.end())
            backupChildrensId.push_back(pKFi->mnId);
    }

    // Save the id of the loop edge KF
    backupLoopEdgesId.clear();
    backupLoopEdgesId.reserve(loopEdges.size());
    for (KeyFrame *pKFi : loopEdges)
    {
        if (spKF.find(pKFi) != spKF.end())
            backupLoopEdgesId.push_back(pKFi->mnId);
    }

    // Save the id of the merge edge KF
    backupMergeEdgesId.clear();
    backupMergeEdgesId.reserve(mergeEdges.size());
    for (KeyFrame *pKFi : mergeEdges)
    {
        if (spKF.find(pKFi) != spKF.end())
            backupMergeEdgesId.push_back(pKFi->mnId);
    }

    // Camera data
    backupCameraId = -1;
    if (p_camera && spCam.find(p_camera) != spCam.end())
        backupCameraId = p_camera->getId();

    backupCamera2Id = -1;
    if (p_camera2 && spCam.find(p_camera2) != spCam.end())
        backupCamera2Id = p_camera2->getId();

    // Inertial data
    backupPrevKFId = -1;
    if (p_prevKF && spKF.find(p_prevKF) != spKF.end())
        backupPrevKFId = p_prevKF->mnId;

    backupNextKFId = -1;
    if (p_nextKF && spKF.find(p_nextKF) != spKF.end())
        backupNextKFId = p_nextKF->mnId;

    if (p_imuPreintegrated)
        backupImuPreintegrated.copyFrom(p_imuPreintegrated);
}

} // namespace core
} // namespace vs_graphs
