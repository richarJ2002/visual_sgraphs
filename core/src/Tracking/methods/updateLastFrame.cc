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

#include "Tracking.h"

namespace vs_graphs
{
namespace core
{

void Tracking::updateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame    *pRef = lastFrame.p_referenceKeyFrame;
    Sophus::SE3f Tlr =
        relativeFramePoses.empty() ? Sophus::SE3f() : relativeFramePoses.back();
    lastFrame.setPose(Tlr * pRef->getPose());

    if (lastKeyFrameId == lastFrame.mnId || sensor == System::MONOCULAR ||
        sensor == System::IMU_MONOCULAR || !onlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D
    // sensor
    vector<pair<float, int>> vDepthIdx;
    const int Nfeat = lastFrame.Nleft == -1 ? lastFrame.N : lastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for (int i = 0; i < Nfeat; i++)
    {
        float z = lastFrame.depths[i];
        if (z > 0)
        {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = lastFrame.mapPoints[i];

        if (!pMP)
            bCreateNew = true;
        else if (pMP->getObservationCount() < 1)
            bCreateNew = true;

        if (bCreateNew)
        {
            Eigen::Vector3f x3D;

            if (lastFrame.Nleft == -1)
            {
                lastFrame.unprojectStereo(i, x3D);
            }
            else
            {
                x3D = lastFrame.unprojectStereoFishEye(i);
            }

            MapPoint *pNewMP =
                new MapPoint(x3D, p_atlas->getCurrentMap(), &lastFrame, i);
            lastFrame.mapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if (vDepthIdx[j].first > depthThreshold && nPoints > 100)
            break;
    }
}

} // namespace core
} // namespace vs_graphs
