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
 * @file            readCamera1.cc
 *
 * @brief           Implements Settings::readCamera1(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <string>
#include <vector>

#include <opencv2/core/persistence.hpp>

#include "System.h"

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

void Settings::readCamera1(cv::FileStorage &storage_inout)
{
    // Variables
    bool               found;
    std::vector<float> calibrations;

    // Camera model
    std::string cameraModelName =
        readParameter<std::string>(storage_inout, "Camera.type", found);

    if (cameraModelName == "PinHole")
    {
        cameraModel = CameraType::PINHOLE;

        // Intrinsic parameters
        float fx     = readParameter<float>(storage_inout, "Camera1.fx", found);
        float fy     = readParameter<float>(storage_inout, "Camera1.fy", found);
        float cx     = readParameter<float>(storage_inout, "Camera1.cx", found);
        float cy     = readParameter<float>(storage_inout, "Camera1.cy", found);
        calibrations = {fx, fy, cx, cy};

        p_calibration1 = new camera_models::pinhole::Pinhole(calibrations);
        p_originalCalibration1 =
            new camera_models::pinhole::Pinhole(calibrations);

        // Check if the Pinhole is distorted
        readParameter<float>(storage_inout, "Camera1.k1", found, false);
        if (found)
        {
            readParameter<float>(storage_inout, "Camera1.k3", found, false);
            if (found)
            {
                pinholeDistortion1.resize(5);
                pinholeDistortion1[4] =
                    readParameter<float>(storage_inout, "Camera1.k3", found);
            }
            else
                pinholeDistortion1.resize(4);
            pinholeDistortion1[0] =
                readParameter<float>(storage_inout, "Camera1.k1", found);
            pinholeDistortion1[1] =
                readParameter<float>(storage_inout, "Camera1.k2", found);
            pinholeDistortion1[2] =
                readParameter<float>(storage_inout, "Camera1.p1", found);
            pinholeDistortion1[3] =
                readParameter<float>(storage_inout, "Camera1.p2", found);
        }

        // Check if we need to correct distortion from the images
        if ((sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR) &&
            pinholeDistortion1.size() != 0)
            isUndistortionNeeded = true;
    }
    else if (cameraModelName == "Rectified")
    {
        cameraModel = CameraType::RECTIFIED;

        // Intrinsic parameters
        float fx     = readParameter<float>(storage_inout, "Camera1.fx", found);
        float fy     = readParameter<float>(storage_inout, "Camera1.fy", found);
        float cx     = readParameter<float>(storage_inout, "Camera1.cx", found);
        float cy     = readParameter<float>(storage_inout, "Camera1.cy", found);
        calibrations = {fx, fy, cx, cy};

        p_calibration1 = new camera_models::pinhole::Pinhole(calibrations);
        p_originalCalibration1 =
            new camera_models::pinhole::Pinhole(calibrations);
    }
    else if (cameraModelName == "camera_models::KannalaBrandt8")
    {
        cameraModel = CameraType::KANNALA_BRANDT;

        // Read intrinsic parameters
        float fx = readParameter<float>(storage_inout, "Camera1.fx", found);
        float fy = readParameter<float>(storage_inout, "Camera1.fy", found);
        float cx = readParameter<float>(storage_inout, "Camera1.cx", found);
        float cy = readParameter<float>(storage_inout, "Camera1.cy", found);

        float k0 = readParameter<float>(storage_inout, "Camera1.k1", found);
        float k1 = readParameter<float>(storage_inout, "Camera1.k2", found);
        float k2 = readParameter<float>(storage_inout, "Camera1.k3", found);
        float k3 = readParameter<float>(storage_inout, "Camera1.k4", found);

        calibrations = {fx, fy, cx, cy, k0, k1, k2, k3};
        p_calibration1 =
            new camera_models::kannalabrandt8::KannalaBrandt8(calibrations);
        p_originalCalibration1 =
            new camera_models::kannalabrandt8::KannalaBrandt8(calibrations);

        if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        {
            int              colBegin     = readParameter<int>(storage_inout,
                                              "Camera1.overlappingBegin",
                                              found);
            int              colEnd       = readParameter<int>(storage_inout,
                                            "Camera1.overlappingEnd",
                                            found);
            std::vector<int> overlappings = {colBegin, colEnd};
            static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                p_calibration1)
                ->lappingArea = overlappings;
        }
    }
    else
    {
        VSLAM_LOG_ERROR("[Settings] Could not find Camera#1 settings for '%s'! "
                        "Exiting ...\n",
                        cameraModelName.c_str());
        exit(-1);
    }
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
