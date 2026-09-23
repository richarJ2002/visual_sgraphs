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

#include <iostream>

namespace vs_graphs
{
namespace core
{

Tracking::Tracking(System                    *pSys,
                   ORBVocabulary             *pVoc,
                   FrameDrawer               *pFrameDrawer,
                   MapDrawer                 *pMapDrawer,
                   Atlas                     *pAtlas,
                   KeyFrameDatabase          *pKFDB,
                   const string              &strSettingPath,
                   const int                  sensorType,
                   utils::settings::Settings *settings,
                   const string              &_nameSeq) :
    state(NO_IMAGES_YET),
    sensor(sensorType),
    trackedFr(0),
    step(false),
    onlyTracking(false),
    mapUpdated(false),
    visualOdometry(false),
    p_orbVocabulary(pVoc),
    p_keyFrameDatabase(pKFDB),
    readyToInitialize(false),
    p_system(pSys),
    p_viewer(nullptr),
    p_frameDrawer(pFrameDrawer),
    p_mapDrawer(pMapDrawer),
    stepByStep(false),
    p_atlas(pAtlas),
    p_lastKeyFrame(static_cast<KeyFrame *>(nullptr)),
    lastRelocFrameId(0),
    time_recently_lost(10.0),
    firstFrameId(0),
    initialFrameId(0),
    createdMap(false),
    p_camera2(nullptr)
{
    (void)_nameSeq;
    // Load camera parameters from settings file
    if (settings)
    {
        std::cout << "[Tracking] New parameters from the config file!"
                  << std::endl;
        newParameterLoader(settings);
    }
    else
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        // Load camera parameters
        std::cout
            << "[Tracking] Loading camera parameters from the config file!"
            << std::endl;
        bool boolParseCamParams = parseCamParamFile(fSettings);
        if (!boolParseCamParams)
            std::cerr << "[Tracking] Error with the camera parameters in the "
                         "config file!"
                      << std::endl;

        // Load ORB parameters
        std::cout << "[Tracking] Loading ORB parameters from the config file!"
                  << std::endl;
        bool boolParseORBFeats = parseORBParamFile(fSettings);
        if (!boolParseORBFeats)
            std::cerr << "[Tracking] Error with the ORB parameters in the "
                         "config file!"
                      << std::endl;

        // Load IMU parameters if needed
        bool boolParseIMU = true;
        std::cout << "[Tracking] Loading IMU parameters from the config file!"
                  << std::endl;
        if (sensorType == System::IMU_MONOCULAR ||
            sensorType == System::IMU_STEREO || sensorType == System::IMU_RGBD)
        {
            boolParseIMU = parseIMUParamFile(fSettings);
            if (!boolParseIMU)
                std::cerr << "[Tracking] Error with the IMU parameters in the "
                             "config file!"
                          << std::endl;
            framesToResetIMU = maxFrames;
        }

        // Check if everything is correctly loaded
        if (!boolParseCamParams || !boolParseORBFeats || !boolParseIMU)
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

    loadTrackingParameters(strSettingPath);
    if (sensorType == System::IMU_MONOCULAR ||
        sensorType == System::IMU_STEREO || sensorType == System::IMU_RGBD)
    {
        framesToResetIMU = maxFrames;
    }

    // Obtain the angles which will be used to rotate the world frame
    poseTc0w = Sophus::SE3f();
    if (sensorType == System::MONOCULAR)
    {
        float dWorldRPY[3] = {};

        string strAngleNames[3] = {"roll", "pitch", "yaw"};

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        std::cout << "Rotate world frame by (rad): ";
        for (int i = 0; i < 3; i++)
        {
            cv::FileNode node = fSettings["WorldRPY." + strAngleNames[i]];
            if (!node.empty() && node.isReal())
            {
                dWorldRPY[i] = node.real();
            }
            else
            {
                dWorldRPY[i] = 0;
            }
            std::cout << strAngleNames[i] << " " << dWorldRPY[i] << " ";
        }
        std::cout << endl;

        Eigen::AngleAxisf  AngleR(dWorldRPY[0], Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf  AngleP(dWorldRPY[1], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf  AngleY(dWorldRPY[2], Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf qRPY   = AngleR * AngleP * AngleY;
        Eigen::Matrix3f    RotRPY = qRPY.matrix();
        poseTc0w = Sophus::SE3f(RotRPY, Eigen::Vector3f::Zero());
    }

    initId       = 0;
    lastId       = 0;
    initWith3KFs = false;
    numDataset   = 0;

    vector<camera_models::geometriccamera::GeometricCamera *> vpCams =
        p_atlas->getAllCameras();
    std::cout << "\n[Tracking] Found " << vpCams.size()
              << " camera(s) in Atlas!" << std::endl;
    for (camera_models::geometriccamera::GeometricCamera *pCam : vpCams)
    {
        std::cout << "- Camera " << pCam->getId();
        if (pCam->getType() ==
            camera_models::geometriccamera::GeometricCamera::CAM_PINHOLE)
            std::cout << " is a pinhole!" << std::endl;
        else if (pCam->getType() ==
                 camera_models::geometriccamera::GeometricCamera::CAM_FISHEYE)
            std::cout << " is a fisheye!" << std::endl;
        else
            std::cout << " is unknown!" << std::endl;
    }

#ifdef REGISTER_TIMES
    vdRectStereo_ms.clear();
    vdResizeImage_ms.clear();
    vdORBExtract_ms.clear();
    vdStereoMatch_ms.clear();
    vdIMUInteg_ms.clear();
    vdPosePred_ms.clear();
    vdLMTrack_ms.clear();
    vdNewKF_ms.clear();
    vdTrackTotal_ms.clear();
#endif
}

} // namespace core
} // namespace vs_graphs
