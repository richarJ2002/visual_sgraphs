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

#include <iomanip>

namespace vs_graphs
{
namespace core
{

void System::saveKeyFrameTrajectoryEuRoC(const string &filename_in)
{
    cout << endl
         << "Saving keyframe trajectory to " << filename_in << " ..." << endl;

    vector<Map *> maps                 = p_atlas->getAllMaps();
    Map          *p_biggerMap          = nullptr;
    std::size_t   maximumKeyFrameCount = 0;
    for (Map *p_map : maps)
    {
        if (p_map && p_map->getAllKeyFrames().size() > maximumKeyFrameCount)
        {
            maximumKeyFrameCount = p_map->getAllKeyFrames().size();
            p_biggerMap          = p_map;
        }
    }

    if (!p_biggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    vector<KeyFrame *> keyFrames = p_biggerMap->getAllKeyFrames();
    sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename_in.c_str());
    f << fixed;

    for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
         keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];

        if (!p_keyFrame || p_keyFrame->isBad())
            continue;
        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD)
        {
            Sophus::SE3f       Twb = p_keyFrame->getImuPose();
            Eigen::Quaternionf q   = Twb.unit_quaternion();
            Eigen::Vector3f    twb = Twb.translation();
            f << setprecision(6) << 1e9 * p_keyFrame->timeStamp << " "
              << setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << endl;
        }
        else
        {
            Sophus::SE3f       Twc = p_keyFrame->getPoseInverse();
            Eigen::Quaternionf q   = Twc.unit_quaternion();
            Eigen::Vector3f    t   = Twc.translation();
            f << setprecision(6) << 1e9 * p_keyFrame->timeStamp << " "
              << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " "
              << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();
}

} // namespace core
} // namespace vs_graphs
