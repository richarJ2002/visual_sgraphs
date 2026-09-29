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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void System::saveKeyFrameTrajectoryTUM(const string &filename_in)
{
    cout << endl
         << "Saving keyframe trajectory to " << filename_in << " ..." << endl;

    vector<KeyFrame *> keyFrames = p_atlas->getAllKeyFrames();
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

        bool keyFrameIsBad{};
        if (p_keyFrame->isBad(keyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (keyFrameIsBad)
            continue;

        Sophus::SE3f Twc{};
        if (p_keyFrame->getPoseInverse(Twc) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f    t = Twc.translation();
        f << setprecision(6) << p_keyFrame->timeStamp << setprecision(7) << " "
          << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
          << " " << q.z() << " " << q.w() << endl;
    }

    f.close();
}

} // namespace core
} // namespace vs_graphs
