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

#include "Tracking.h"
#include "System.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

Tracking::Tracking(System                    *p_sys_in,
                   ORBVocabulary             *p_vocabulary_in,
                   FrameDrawer               *p_frameDrawer_in,
                   MapDrawer                 *p_mapDrawer_in,
                   Atlas                     *p_atlas_in,
                   KeyFrameDatabase          *p_keyFrameDatabase_in,
                   const string              &settingPath_in,
                   const int                  sensorType_in,
                   utils::settings::Settings *p_settings_in,
                   const string              &nameSeq_in) :
    state(NO_IMAGES_YET),
    sensor(sensorType_in),
    trackedFr(0),
    isStepRequested(false),
    isTrackingOnlyMode(false),
    isMapUpdated(false),
    isVisualOdometry(false),
    p_orbVocabulary(p_vocabulary_in),
    p_keyFrameDatabase(p_keyFrameDatabase_in),
    isReadyToInitialize(false),
    p_system(p_sys_in),
    p_viewer(nullptr),
    p_frameDrawer(p_frameDrawer_in),
    p_mapDrawer(p_mapDrawer_in),
    isStepByStepMode(false),
    p_atlas(p_atlas_in),
    p_lastKeyFrame(static_cast<KeyFrame *>(nullptr)),
    lastRelocFrameId(0),
    time_recently_lost(10.0),
    firstFrameId(0),
    initialFrameId(0),
    hasCreatedMap(false),
    p_camera2(nullptr)
{
    (void)nameSeq_in;
    // Load camera parameters from settings file
    if (p_settings_in)
    {
        std::cout << "[Tracking] New parameters from the config file!"
                  << std::endl;
        if (newParameterLoader(p_settings_in) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: newParameterLoader returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    else
    {
        cv::FileStorage settingsFile(settingPath_in, cv::FileStorage::READ);

        // Load camera parameters
        std::cout
            << "[Tracking] Loading camera parameters from the config file!"
            << std::endl;
        bool boolParseCameraParams{};
        if (parseCamParamFile(settingsFile, boolParseCameraParams) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: parseCamParamFile returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!boolParseCameraParams)
            std::cerr << "[Tracking] Error with the camera parameters in the "
                         "config file!"
                      << std::endl;

        // Load ORB parameters
        std::cout << "[Tracking] Loading ORB parameters from the config file!"
                  << std::endl;
        bool boolParseOrbFeats{};
        if (parseORBParamFile(settingsFile, boolParseOrbFeats) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: parseORBParamFile returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!boolParseOrbFeats)
            std::cerr << "[Tracking] Error with the ORB parameters in the "
                         "config file!"
                      << std::endl;

        // Load IMU parameters if needed
        bool boolParseImu = true;
        std::cout << "[Tracking] Loading IMU parameters from the config file!"
                  << std::endl;
        if (sensorType_in == System::IMU_MONOCULAR ||
            sensorType_in == System::IMU_STEREO ||
            sensorType_in == System::IMU_RGBD)
        {
            bool isParsed{};
            if (parseIMUParamFile(settingsFile, isParsed) !=
                TrackingStatus::TRACKING_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: parseIMUParamFile returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            boolParseImu = isParsed;
            if (!boolParseImu)
                std::cerr << "[Tracking] Error with the IMU parameters in the "
                             "config file!"
                          << std::endl;
            framesToResetIMU = maxFrames;
        }

        // Check if everything is correctly loaded
        if (!boolParseCameraParams || !boolParseOrbFeats || !boolParseImu)
        {
            std::cerr << "[Tracking] Error found in the config file! The "
                         "format seems to be incorrect!"
                      << std::endl;
            try
            {
                throw -1;
            }
            catch (exception &e)
            {}
        }
    }

    if (loadTrackingParameters(settingPath_in) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: loadTrackingParameters returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (sensorType_in == System::IMU_MONOCULAR ||
        sensorType_in == System::IMU_STEREO ||
        sensorType_in == System::IMU_RGBD)
    {
        framesToResetIMU = maxFrames;
    }

    // Obtain the angles which will be used to rotate the world frame
    poseTc0w = Sophus::SE3f();
    if (sensorType_in == System::MONOCULAR)
    {
        float worldRollPitchYaw[3] = {};

        string angleNames[3] = {"roll", "pitch", "yaw"};

        cv::FileStorage settingsFile(settingPath_in, cv::FileStorage::READ);

        std::cout << "Rotate world frame by (rad): ";
        for (int axisIndex = 0; axisIndex < 3; axisIndex++)
        {
            cv::FileNode node =
                settingsFile["WorldRPY." + angleNames[axisIndex]];
            if (!node.empty() && node.isReal())
            {
                worldRollPitchYaw[axisIndex] = node.real();
            }
            else
            {
                worldRollPitchYaw[axisIndex] = 0;
            }
            std::cout << angleNames[axisIndex] << " "
                      << worldRollPitchYaw[axisIndex] << " ";
        }
        std::cout << endl;

        Eigen::AngleAxisf  angleR(worldRollPitchYaw[0],
                                 Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf  angleP(worldRollPitchYaw[1],
                                 Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf  angleY(worldRollPitchYaw[2],
                                 Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf rollPitchYawQuaternion = angleR * angleP * angleY;
        Eigen::Matrix3f    rotRpy = rollPitchYawQuaternion.matrix();
        poseTc0w = Sophus::SE3f(rotRpy, Eigen::Vector3f::Zero());
    }

    initId                             = 0;
    lastId                             = 0;
    shouldInitializeWithThreeKeyFrames = false;
    numDataset                         = 0;

    std::vector<camera_models::geometriccamera::GeometricCamera *> cams{};
    if (p_atlas->getAllCameras(cams) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllCameras returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "\n[Tracking] Found " << cams.size() << " camera(s) in Atlas!"
              << std::endl;
    for (camera_models::geometriccamera::GeometricCamera *p_camera : cams)
    {
        unsigned int cameraId{};
        if (p_camera->getId(cameraId) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "- Camera " << cameraId;
        unsigned int cameraType{};
        if (p_camera->getType(cameraType) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getType returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (cameraType ==
            camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE)
            std::cout << " is a pinhole!" << std::endl;
        else if (cameraType ==
                 camera_models::geometriccamera::GeometricCamera::CAM_FISHEYE)
            std::cout << " is a fisheye!" << std::endl;
        else
            std::cout << " is unknown!" << std::endl;
    }

#ifdef REGISTER_TIMES
    stereoRectificationTimes_ms.clear();
    imageResizeTimes_ms.clear();
    orbExtractionTimes_ms.clear();
    stereoMatchTimes_ms.clear();
    imuIntegrationTimes_ms.clear();
    posePredictionTimes_ms.clear();
    localMapTrackTimes_ms.clear();
    newKeyFrameTimes_ms.clear();
    trackTotalTimes_ms.clear();
#endif
}

} // namespace core
} // namespace vs_graphs
