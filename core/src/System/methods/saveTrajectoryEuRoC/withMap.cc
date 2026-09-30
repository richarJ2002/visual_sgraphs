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

SystemStatus System::saveTrajectoryEuRoC(const std::string &filename_in,
                                         Map               *p_map_in)
{

    unsigned long mapId{};
    if (p_map_in->getId(mapId) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << std::endl
              << "Saving trajectory of map " << mapId << " to " << filename_in
              << " ..." << std::endl;

    std::vector<KeyFrame *> keyFrames{};
    if (p_map_in->getAllKeyFrames(keyFrames) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f poseBodyToWorld; // Can be word to cam0 or world to b
                                  // dependingo on IMU or not.
    if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO || sensor == IMU_RGBD)
    {
        Sophus::SE3f imuPose{};
        if (keyFrames[0]->getImuPose(imuPose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        poseBodyToWorld = imuPose;
    }
    else
    {
        Sophus::SE3f poseInverse{};
        if (keyFrames[0]->getPoseInverse(poseInverse) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPoseInverse returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        poseBodyToWorld = poseInverse;
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
    std::list<double>::iterator lT  = p_tracker->frameTimes.begin();
    std::list<bool>::iterator   lbL = p_tracker->lostFlags.begin();

    for (std::list<Sophus::SE3f>::iterator
             lit  = p_tracker->relativeFramePoses.begin(),
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

        Map *p_keyFrameMap = nullptr;
        if (!(!p_keyFrame) && p_keyFrame->getMap(p_keyFrameMap) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || p_keyFrameMap != p_map_in)
            continue;

        Sophus::SE3f keyFramePose{};
        if (p_keyFrame->getPose(keyFramePose) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Trw =
            Trw * keyFramePose * poseBodyToWorld; // Tcp*Tpw*Twb0=Tcb0 where b0
                                                  // is the new world reference

        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD)
        {
            Sophus::SE3f keyFrameTwb =
                (p_keyFrame->imuCalibration.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = keyFrameTwb.unit_quaternion();
            Eigen::Vector3f translationBodyToWorld = keyFrameTwb.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << translationBodyToWorld(0) << " "
              << translationBodyToWorld(1) << " " << translationBodyToWorld(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;
        }
        else
        {
            Sophus::SE3f       poseCameraToWorld = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q = poseCameraToWorld.unit_quaternion();
            Eigen::Vector3f    translationCameraToWorld =
                poseCameraToWorld.translation();
            f << std::setprecision(6) << 1e9 * (*lT) << " "
              << std::setprecision(9) << translationCameraToWorld(0) << " "
              << translationCameraToWorld(1) << " "
              << translationCameraToWorld(2) << " " << q.x() << " " << q.y()
              << " " << q.z() << " " << q.w() << std::endl;
        }
    }
    f.close();
    std::cout << std::endl
              << "End of saving trajectory to " << filename_in << " ..."
              << std::endl;

    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
