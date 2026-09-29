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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "System.h"
#include "Tracking.h"

#include <iomanip>

namespace vs_graphs
{
namespace core
{

void System::saveTrajectoryEuRoC(const string &filename_in)
{

    cout << endl << "Saving trajectory to " << filename_in << " ..." << endl;

    vector<Map *> maps                 = p_atlas->getAllMaps();
    std::size_t   maximumKeyFrameCount = 0;
    Map          *p_biggerMap          = nullptr;
    std::cout << "There are " << std::to_string(maps.size())
              << " maps in the atlas" << std::endl;
    for (Map *p_map : maps)
    {
        if (p_map == nullptr)
        {
            continue;
        }

        const std::size_t keyFrameCount = p_map->getAllKeyFrames().size();

        std::cout << "  Map " << std::to_string(p_map->getId()) << " has "
                  << std::to_string(keyFrameCount) << " KFs" << std::endl;
        if (keyFrameCount > maximumKeyFrameCount)
        {
            maximumKeyFrameCount = keyFrameCount;
            p_biggerMap          = p_map;
        }
    }

    if (p_biggerMap == nullptr)
    {
        std::cerr << "Cannot save a trajectory: the Atlas has no keyframes."
                  << std::endl;
        return;
    }

    vector<KeyFrame *> keyFrames = p_biggerMap->getAllKeyFrames();
    sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f
        Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO || sensor == IMU_RGBD)
        Twb = keyFrames[0]->getImuPose();
    else
        Twb = keyFrames[0]->getPoseInverse();

    ofstream f;
    f.open(filename_in.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<vs_graphs::core::KeyFrame *>::iterator rits =
        p_tracker->referenceKeyFrames.begin();
    list<double>::iterator lT  = p_tracker->frameTimes.begin();
    list<bool>::iterator   lbL = p_tracker->lostFlags.begin();

    for (auto lit  = p_tracker->relativeFramePoses.begin(),
              lend = p_tracker->relativeFramePoses.end();
         lit != lend;
         lit++, rits++, lT++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *p_keyFrame = *rits;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
        // get a suitable keyframe.
        if (!p_keyFrame)
            continue;

        while (p_keyFrame->isBad())
        {
            Trw        = Trw * p_keyFrame->tcp;
            p_keyFrame = p_keyFrame->getParent();
        }

        if (!p_keyFrame || p_keyFrame->getMap() != p_biggerMap)
            continue;

        Trw = Trw * p_keyFrame->getPose() *
              Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD)
        {
            Sophus::SE3f Twb =
                (p_keyFrame->imuCalibration.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9)
              << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f       Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    twc = Twc.translation();
            f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9)
              << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }

    f.close();
    cout << endl
         << "End of saving trajectory to " << filename_in << " ..." << endl;
}

} // namespace core
} // namespace vs_graphs
