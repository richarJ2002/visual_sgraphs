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

#include "LocalMapping.h"

#include "Optimizer.h"
#include "Tracking.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void LocalMapping::scaleRefinement()
{
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a
    // solution. Make the difference between monocular and stereo
    // unique_lock<mutex> lock0(imuInitMutex);
    if (isResetRequested)
        return;

    // Retrieve all keyframes in temporal order
    list<KeyFrame *> temporalKeyFrames;
    KeyFrame        *p_walkKeyFrame = p_currentKeyFrame;
    while (p_walkKeyFrame->p_prevKF)
    {
        temporalKeyFrames.push_front(p_walkKeyFrame);
        p_walkKeyFrame = p_walkKeyFrame->p_prevKF;
    }
    temporalKeyFrames.push_front(p_walkKeyFrame);
    vector<KeyFrame *> orderedKeyFrames(temporalKeyFrames.begin(),
                                        temporalKeyFrames.end());

    while (checkNewKeyFrames())
    {
        processNewKeyFrame();
        orderedKeyFrames.push_back(p_currentKeyFrame);
        temporalKeyFrames.push_back(p_currentKeyFrame);
    }

    mRwg  = Eigen::Matrix3d::Identity();
    scale = 1.0;

    Optimizer::inertialOptimization(p_atlas->getCurrentMap(), mRwg, scale);

    if (scale < 1e-1) // 1e-1
    {
        cout << "scale too small" << endl;
        isInitializationInProgress = false;
        return;
    }

    Sophus::SO3d                 so3wg(mRwg);
    // Before this line we are not changing the map
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();
    Map *p_activeMap = p_atlas->getCurrentMap();

    if (p_activeMap == nullptr)
    {
        isInitializationInProgress = false;
        return;
    }

    unique_lock<mutex> mapUpdateLock(p_activeMap->mapUpdateMutex);
    if ((fabs(scale - 1.f) > 0.002) || !isMonocular)
    {
        Sophus::SE3f Tgw(mRwg.cast<float>().transpose(),
                         Eigen::Vector3f::Zero());
        p_activeMap->applyScaledRotation(Tgw, scale, true);
        p_tracker->updateFrameIMU(scale,
                                  p_currentKeyFrame->getImuBias(),
                                  p_currentKeyFrame);
    }

    for (list<KeyFrame *>::iterator newKeyFrameIt  = newKeyFrames.begin(),
                                    newKeyFrameEnd = newKeyFrames.end();
         newKeyFrameIt != newKeyFrameEnd;
         newKeyFrameIt++)
    {
        (*newKeyFrameIt)->setBadFlag();
        delete *newKeyFrameIt;
    }
    newKeyFrames.clear();

    // To perform pose-inertial opt w.r.t. last keyframe
    p_currentKeyFrame->getMap()->increaseChangeIndex();

    return;
}

} // namespace core
} // namespace vs_graphs
