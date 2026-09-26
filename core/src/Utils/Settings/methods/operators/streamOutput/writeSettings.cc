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
 * @file            writeSettings.cc
 *
 * @brief           Implements the Settings stream-output operator,
 *                  declared in Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <iostream>

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

using namespace std;

std::ostream &operator<<(std::ostream &output, const Settings &settings)
{
    // Camera#1
    output << "\t- Camera#1 parameters (";
    if (settings.cameraModel == Settings::CameraType::PINHOLE ||
        settings.cameraModel == Settings::CameraType::RECTIFIED)
        output << "camera_models::Pinhole";
    else
        output << "Kannala-Brandt";
    output << "): [";
    for (size_t i = 0; i < settings.originalCalibration1->size(); i++)
        output << " " << settings.originalCalibration1->getParameter(i);
    output << " ]" << endl;

    if (!settings.pinholeDistortion1.empty())
    {
        output << "\t- Camera#1 distortion parameters: [ ";
        for (float d : settings.pinholeDistortion1)
            output << " " << d;
        output << " ]" << endl;
    }

    if ((settings.sensor == System::STEREO ||
         settings.sensor == System::IMU_STEREO) &&
        (settings.cameraModel != Settings::CameraType::RECTIFIED))
    {
        output << "\t- Camera#2 parameters (";
        if (settings.cameraModel == Settings::CameraType::PINHOLE)
            output << "camera_models::Pinhole";
        else
            output << "Kannala-Brandt";
        output << "): [";
        for (size_t i = 0; i < settings.originalCalibration2->size(); i++)
            output << " " << settings.originalCalibration2->getParameter(i);
        output << " ]" << endl;

        if (!settings.pinholeDistortion2.empty())
        {
            output << "\t- Camera#2 distortion parameters: [ ";
            for (float d : settings.pinholeDistortion2)
                output << " " << d;
            output << " ]" << endl;
        }
    }

    output << "\t- Original frame size: [ " << settings.originalImageSize.width
           << "," << settings.originalImageSize.height << " ]" << endl;
    output << "\t- Current frame size: [ " << settings.newImageSize.width << ","
           << settings.newImageSize.height << " ]" << endl;

    if (settings.rectifyNeeded)
    {
        output << "\t- Camera#1 parameters after rectification: [";
        for (size_t i = 0; i < settings.calibration1->size(); i++)
            output << " " << settings.calibration1->getParameter(i);
        output << " ]" << endl;

        if (settings.sensor == System::STEREO ||
            settings.sensor == System::IMU_STEREO)
        {
            output << "\t- Camera#2 parameters after rectification: [";
            for (size_t i = 0; i < settings.calibration2->size(); i++)
                output << " " << settings.calibration2->getParameter(i);
            output << " ]" << endl;
        }
    }
    else if (settings.resize1Needed)
    {
        output << "\t- Camera#1 parameters after resize: [";
        for (size_t i = 0; i < settings.calibration1->size(); i++)
            output << " " << settings.calibration1->getParameter(i);
        output << " ]" << endl;

        if ((settings.sensor == System::STEREO ||
             settings.sensor == System::IMU_STEREO) &&
            settings.cameraModel == Settings::CameraType::KANNALA_BRANDT)
        {
            output << "\t- Camera#2 parameters after resize: [";
            for (size_t i = 0; i < settings.calibration2->size(); i++)
                output << " " << settings.calibration2->getParameter(i);
            output << " ]" << endl;
        }
    }

    // Frame rate
    output << "\t- Sequence FPS: " << settings.framesPerSecond << endl;

    // Stereo stuff
    if (settings.sensor == System::STEREO ||
        settings.sensor == System::IMU_STEREO)
    {
        output << "\t- Stereo baseline: " << settings.stereoBaseline << endl;
        output << "\t- Stereo depth threshold : " << settings.depthThreshold
               << endl;

        if (settings.cameraModel == Settings::CameraType::KANNALA_BRANDT)
        {
            auto vOverlapping1 =
                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    settings.calibration1)
                    ->lappingArea;
            auto vOverlapping2 =
                static_cast<camera_models::kannalabrandt8::KannalaBrandt8 *>(
                    settings.calibration2)
                    ->lappingArea;
            output << "\t- Camera 1 overlapping area: [ " << vOverlapping1[0]
                   << " , " << vOverlapping1[1] << " ]" << endl;
            output << "\t- Camera 2 overlapping area: [ " << vOverlapping2[0]
                   << " , " << vOverlapping2[1] << " ]" << endl;
        }
    }

    // IMU parameters
    if (settings.sensor == System::IMU_MONOCULAR ||
        settings.sensor == System::IMU_STEREO ||
        settings.sensor == System::IMU_RGBD)
    {
        output << "\t- Gyro noise: " << settings.gyroNoise << endl;
        output << "\t- Accelerometer noise: " << settings.accelNoise << endl;
        output << "\t- Gyro walk: " << settings.gyroWalkNoise << endl;
        output << "\t- Accelerometer walk: " << settings.accelWalkNoise << endl;
        output << "\t- IMU frequency: " << settings.imuSampleRate << endl;
        output << "\t- IMU threshold: " << settings.imuErrorThreshold << endl;
    }

    // RGB-D parameters
    if (settings.sensor == System::RGBD || settings.sensor == System::IMU_RGBD)
    {
        output << "\t- RGB-D depth map factor: " << settings.depthMapScale
               << endl;
        output << "\t- Stereo depth threshold: " << settings.depthThreshold
               << endl;
        output << "\t- Metric close depth: "
               << settings.stereoBaseline * settings.depthThreshold << endl;
    }

    // ORB parameters
    output << "\t- Features per image: " << settings.featureCount << endl;
    output << "\t- ORB scale factor: " << settings.orbScaleFactor << endl;
    output << "\t- ORB number of scales: " << settings.pyramidLevels << endl;
    output << "\t- Initial FAST threshold: " << settings.initialFastThreshold
           << endl;
    output << "\t- Min FAST threshold: " << settings.minimumFastThreshold
           << endl;

    return output;
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
