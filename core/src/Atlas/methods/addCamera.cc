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

/*!
 * @file            addCamera.cc
 *
 * @brief           Implements Atlas::addCamera(), declared in Atlas.h.
 */

#include "Atlas.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

AtlasStatus Atlas::addCamera(
    camera_models::geometriccamera::GeometricCamera  *p_camera_in,
    camera_models::geometriccamera::GeometricCamera *&p_camera_out)
{
    /* Only camera methods run under the lock; they take no lock. */
    std::unique_lock<std::mutex> atlasLock(atlasMutex);

    // Check if the camera already exists
    bool isAlreadyInMap     = false;
    int  matchedCameraIndex = -1;
    for (size_t cameraIndex = 0; cameraIndex < cameras.size(); ++cameraIndex)
    {
        camera_models::geometriccamera::GeometricCamera *p_existingCamera =
            cameras[cameraIndex];
        if (!p_camera_in)
            std::cout << "Not pCam" << std::endl;
        if (!p_existingCamera)
            std::cout << "Not pCam_i" << std::endl;
        unsigned int cameraType{};
        if (p_camera_in->getType(cameraType) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getType returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned int existingCameraType{};
        if (p_existingCamera->getType(existingCameraType) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getType returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (cameraType != existingCameraType)
            continue;

        unsigned int cameraType2{};
        if (p_camera_in->getType(cameraType2) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getType returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (cameraType2 ==
            camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE)
        {
            bool isEqual2{};
            // the camera type check above makes this downcast exact
            if (static_cast<camera_models::pinhole::Pinhole *>(p_existingCamera)
                    ->isEqual(p_camera_in, isEqual2) !=
                camera_models::pinhole::PinholeStatus::PINHOLE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isEqual returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (isEqual2)
            {
                isAlreadyInMap     = true;
                matchedCameraIndex = cameraIndex;
            }
        }
        else if (cameraType2 ==
                 camera_models::geometriccamera::GeometricCamera::CAM_FISHEYE)
        {
            bool isEqual3{};
            // the camera type check above makes this downcast exact
            if (static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    p_existingCamera)
                    ->isEqual(p_camera_in, isEqual3) !=
                camera_models::kannalabrandt8::KannalaBrandt8Status::
                    KANNALA_BRANDT8_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isEqual returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (isEqual3)
            {
                isAlreadyInMap     = true;
                matchedCameraIndex = cameraIndex;
            }
        }
    }

    if (isAlreadyInMap)
    {
        p_camera_out = cameras[matchedCameraIndex];
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    else
    {
        cameras.push_back(p_camera_in);
        p_camera_out = p_camera_in;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
