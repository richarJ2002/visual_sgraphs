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
 * @file            readCamera2.cc
 *
 * @brief           Implements Settings::readCamera2(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/persistence.hpp>

#include "Utils/Converter/objects/Converter.h"

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

using namespace std;

void Settings::readCamera2(cv::FileStorage &storage_inout)
{
    bool          found;
    vector<float> calibrations;
    if (cameraModel == CameraType::PINHOLE)
    {
        isRectificationNeeded = true;

        // Read intrinsic parameters
        float fx = readParameter<float>(storage_inout, "Camera2.fx", found);
        float fy = readParameter<float>(storage_inout, "Camera2.fy", found);
        float cx = readParameter<float>(storage_inout, "Camera2.cx", found);
        float cy = readParameter<float>(storage_inout, "Camera2.cy", found);

        calibrations = {fx, fy, cx, cy};

        p_calibration2 = new camera_models::pinhole::Pinhole(calibrations);
        p_originalCalibration2 =
            new camera_models::pinhole::Pinhole(calibrations);

        // Check if it is a distorted Pinhole
        readParameter<float>(storage_inout, "Camera2.k1", found, false);
        if (found)
        {
            readParameter<float>(storage_inout, "Camera2.k3", found, false);
            if (found)
            {
                pinholeDistortion2.resize(5);
                pinholeDistortion2[4] =
                    readParameter<float>(storage_inout, "Camera2.k3", found);
            }
            else
            {
                pinholeDistortion2.resize(4);
            }
            pinholeDistortion2[0] =
                readParameter<float>(storage_inout, "Camera2.k1", found);
            pinholeDistortion2[1] =
                readParameter<float>(storage_inout, "Camera2.k2", found);
            pinholeDistortion2[2] =
                readParameter<float>(storage_inout, "Camera2.p1", found);
            pinholeDistortion2[3] =
                readParameter<float>(storage_inout, "Camera2.p2", found);
        }
    }
    else if (cameraModel == CameraType::KANNALA_BRANDT)
    {
        // Read intrinsic parameters
        float fx = readParameter<float>(storage_inout, "Camera2.fx", found);
        float fy = readParameter<float>(storage_inout, "Camera2.fy", found);
        float cx = readParameter<float>(storage_inout, "Camera2.cx", found);
        float cy = readParameter<float>(storage_inout, "Camera2.cy", found);

        float k0 = readParameter<float>(storage_inout, "Camera2.k1", found);
        float k1 = readParameter<float>(storage_inout, "Camera2.k2", found);
        float k2 = readParameter<float>(storage_inout, "Camera2.k3", found);
        float k3 = readParameter<float>(storage_inout, "Camera2.k4", found);

        calibrations = {fx, fy, cx, cy, k0, k1, k2, k3};

        p_calibration2 =
            new camera_models::kannalabrandt8::KannalaBrandt8(calibrations);
        p_originalCalibration2 =
            new camera_models::kannalabrandt8::KannalaBrandt8(calibrations);

        int colBegin = readParameter<int>(storage_inout,
                                          "Camera2.overlappingBegin",
                                          found);
        int colEnd =
            readParameter<int>(storage_inout, "Camera2.overlappingEnd", found);
        vector<int> overlappings = {colBegin, colEnd};

        static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
            p_calibration2)
            ->lappingArea = overlappings;
    }

    // Load stereo extrinsic calibration
    if (cameraModel == CameraType::RECTIFIED)
    {
        stereoBaseline = readParameter<float>(storage_inout, "Stereo.b", found);
        baselineFocal  = stereoBaseline * p_calibration1->getParameter(0);
    }
    else
    {
        cv::Mat cvTlr =
            readParameter<cv::Mat>(storage_inout, "Stereo.T_c1_c2", found);
        stereoTransform = converter::Converter::toSophus(cvTlr);

        // TODO: also search for Trl and invert if necessary

        stereoBaseline = stereoTransform.translation().norm();
        baselineFocal  = stereoBaseline * p_calibration1->getParameter(0);
    }

    depthThreshold =
        readParameter<float>(storage_inout, "Stereo.ThDepth", found);
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
