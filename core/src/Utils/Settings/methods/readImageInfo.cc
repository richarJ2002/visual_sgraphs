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
 * @file            readImageInfo.cc
 *
 * @brief           Implements Settings::readImageInfo(), declared in
 *                  Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/persistence.hpp>

#include "System.h"

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

void Settings::readImageInfo(cv::FileStorage &storage_in)
{
    bool found;
    // Read original and desired image dimensions
    int  originalRows = readParameter<int>(storage_in, "Camera.height", found);
    int  originalCols = readParameter<int>(storage_in, "Camera.width", found);
    originalImageSize.width  = originalCols;
    originalImageSize.height = originalRows;

    newImageSize = originalImageSize;
    int newHeigh =
        readParameter<int>(storage_in, "Camera.newHeight", found, false);
    if (found)
    {
        resize1Needed       = true;
        newImageSize.height = newHeigh;

        if (!rectifyNeeded)
        {
            // Update calibration
            float scaleRowFactor =
                (float)newImageSize.height / (float)originalImageSize.height;
            calibration1->setParameter(calibration1->getParameter(1) *
                                           scaleRowFactor,
                                       1);
            calibration1->setParameter(calibration1->getParameter(3) *
                                           scaleRowFactor,
                                       3);

            if ((sensor == System::STEREO || sensor == System::IMU_STEREO) &&
                cameraModel != CameraType::RECTIFIED)
            {
                calibration2->setParameter(calibration2->getParameter(1) *
                                               scaleRowFactor,
                                           1);
                calibration2->setParameter(calibration2->getParameter(3) *
                                               scaleRowFactor,
                                           3);
            }
        }
    }

    int newWidth =
        readParameter<int>(storage_in, "Camera.newWidth", found, false);
    if (found)
    {
        resize1Needed      = true;
        newImageSize.width = newWidth;

        if (!rectifyNeeded)
        {
            // Update calibration
            float scaleColFactor =
                (float)newImageSize.width / (float)originalImageSize.width;
            calibration1->setParameter(calibration1->getParameter(0) *
                                           scaleColFactor,
                                       0);
            calibration1->setParameter(calibration1->getParameter(2) *
                                           scaleColFactor,
                                       2);

            if ((sensor == System::STEREO || sensor == System::IMU_STEREO) &&
                cameraModel != CameraType::RECTIFIED)
            {
                calibration2->setParameter(calibration2->getParameter(0) *
                                               scaleColFactor,
                                           0);
                calibration2->setParameter(calibration2->getParameter(2) *
                                               scaleColFactor,
                                           2);

                if (cameraModel == CameraType::KANNALA_BRANDT)
                {
                    static_cast<
                        camera_models::kannalabrandt8::KannalaBrandt8 *>(
                        calibration1)
                        ->lappingArea[0] *= scaleColFactor;
                    static_cast<
                        camera_models::kannalabrandt8::KannalaBrandt8 *>(
                        calibration1)
                        ->lappingArea[1] *= scaleColFactor;

                    static_cast<
                        camera_models::kannalabrandt8::KannalaBrandt8 *>(
                        calibration2)
                        ->lappingArea[0] *= scaleColFactor;
                    static_cast<
                        camera_models::kannalabrandt8::KannalaBrandt8 *>(
                        calibration2)
                        ->lappingArea[1] *= scaleColFactor;
                }
            }
        }
    }

    framesPerSecond = readParameter<int>(storage_in, "Camera.fps", found);
    rgbEnabled      = (bool)readParameter<int>(storage_in, "Camera.RGB", found);
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
