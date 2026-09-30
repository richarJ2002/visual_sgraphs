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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::saveTrajectoryKITTI(const std::string &filename_in)
{
    std::cout << std::endl
              << "Saving camera trajectory to " << filename_in << " ..."
              << std::endl;
    if (sensor == MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular."
                  << std::endl;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }

    std::vector<KeyFrame *> keyFrames{};
    if (p_atlas->getAllKeyFrames(keyFrames) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow{};
    if (keyFrames[0]->getPoseInverse(Tow) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPoseInverse returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::ofstream f;
    f.open(filename_in.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is
    // optimized by BA and pose graph). We need to get first the keyframe pose
    // and then concatenate the relative transformation. Frames not localized
    // (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
    // and a flag which is true when tracking failed (lbL).
    list<vs_graphs::core::KeyFrame *>::iterator rits =
        p_tracker->referenceKeyFrames.begin();
    std::list<double>::iterator lT = p_tracker->frameTimes.begin();
    for (std::list<Sophus::SE3f>::iterator
             lit  = p_tracker->relativeFramePoses.begin(),
             lend = p_tracker->relativeFramePoses.end();
         lit != lend;
         lit++, rits++, lT++)
    {
        vs_graphs::core::KeyFrame *p_keyFrame = *rits;

        Sophus::SE3f Trw;

        if (!p_keyFrame)
            continue;

        for (;;)
        {
            bool keyFrameIsBad{};
            if (p_keyFrame->isBad(keyFrameIsBad) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!(keyFrameIsBad))
            {
                break;
            }
            Trw                        = Trw * p_keyFrame->tcp;
            KeyFrame *p_keyFrameParent = nullptr;
            if (p_keyFrame->getParent(p_keyFrameParent) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParent returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            p_keyFrame = p_keyFrameParent;
        }

        Sophus::SE3f keyFramePose{};
        if (p_keyFrame->getPose(keyFramePose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Trw = Trw * keyFramePose * Tow;

        Sophus::SE3f    Tcw = (*lit) * Trw;
        Sophus::SE3f    Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << std::setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " "
          << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1)
          << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " "
          << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << std::endl;
    }
    f.close();

    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
