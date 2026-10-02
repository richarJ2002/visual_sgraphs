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

/*!
 * @file            withoutMap.cc
 *
 * @brief           Implements System::saveKeyFrameTrajectoryEuRoC()
 *                  (withoutMap), declared in System.h.
 */

#include "System.h"

#include <iomanip>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::saveKeyFrameTrajectoryEuRoC(const std::string &filename_in)
{
    std::cout << std::endl
              << "Saving keyframe trajectory to " << filename_in << " ..."
              << std::endl;

    std::vector<Map *> maps{};
    if (p_atlas->getAllMaps(maps) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Map        *p_biggerMap          = nullptr;
    std::size_t maximumKeyFrameCount = 0;
    for (Map *p_map : maps)
    {
        std::vector<KeyFrame *> mapAllKeyFrames{};
        if ((p_map) && p_map->getAllKeyFrames(mapAllKeyFrames) !=
                           MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllKeyFrames returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_map && mapAllKeyFrames.size() > maximumKeyFrameCount)
        {
            std::vector<KeyFrame *> mapAllKeyFrames2{};
            if (p_map->getAllKeyFrames(mapAllKeyFrames2) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getAllKeyFrames returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            maximumKeyFrameCount = mapAllKeyFrames2.size();
            p_biggerMap          = p_map;
        }
    }

    if (!p_biggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }

    std::vector<KeyFrame *> keyFrames{};
    if (p_biggerMap->getAllKeyFrames(keyFrames) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::sort(keyFrames.begin(), keyFrames.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename_in.c_str());
    f << std::fixed;

    for (size_t keyFrameIndex = 0; keyFrameIndex < keyFrames.size();
         keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = keyFrames[keyFrameIndex];

        bool keyFrameIsBad{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad)
            continue;
        if (sensor == IMU_MONOCULAR || sensor == IMU_STEREO ||
            sensor == IMU_RGBD)
        {
            Sophus::SE3f pose_bodyToWorld{};
            if (p_keyFrame->getImuPose(pose_bodyToWorld) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getImuPose returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Quaternionf q = pose_bodyToWorld.unit_quaternion();
            Eigen::Vector3f    translation_bodyToWorld =
                pose_bodyToWorld.translation();
            f << std::setprecision(6) << 1e9 * p_keyFrame->timeStamp << " "
              << std::setprecision(9) << translation_bodyToWorld(0) << " "
              << translation_bodyToWorld(1) << " " << translation_bodyToWorld(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;
        }
        else
        {
            Sophus::SE3f pose_cameraToWorld{};
            if (p_keyFrame->getPoseInverse(pose_cameraToWorld) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Quaternionf q = pose_cameraToWorld.unit_quaternion();
            Eigen::Vector3f    t = pose_cameraToWorld.translation();
            f << std::setprecision(6) << 1e9 * p_keyFrame->timeStamp << " "
              << std::setprecision(9) << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
              << std::endl;
        }
    }
    f.close();

    return SystemStatus::SYSTEM_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
