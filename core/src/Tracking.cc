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
 * @file         Tracking.cc
 *
 * @brief        Implements Tracking declared in Tracking.h.
 */

#include "Tracking.h"

#include "Converter.h"
#include "FrameDrawer.h"
#include "G2oTypes.h"
#include "GeometricTools.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "ResetCause.h"

#include <cmath>
#include <iostream>

#include <chrono>
#include <mutex>

using namespace std;

namespace vs_graphs
{
namespace core
{
Tracking::Tracking(System           *pSys,
                   ORBVocabulary    *pVoc,
                   FrameDrawer      *pFrameDrawer,
                   MapDrawer        *pMapDrawer,
                   Atlas            *pAtlas,
                   KeyFrameDatabase *pKFDB,
                   const string     &strSettingPath,
                   const int         sensorType,
                   Settings         *settings,
                   const string     &_nameSeq) :
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

    vector<camera_models::GeometricCamera *> vpCams = p_atlas->getAllCameras();
    std::cout << "\n[Tracking] Found " << vpCams.size()
              << " camera(s) in Atlas!" << std::endl;
    for (camera_models::GeometricCamera *pCam : vpCams)
    {
        std::cout << "- Camera " << pCam->getId();
        if (pCam->getType() == camera_models::GeometricCamera::CAM_PINHOLE)
            std::cout << " is a pinhole!" << std::endl;
        else if (pCam->getType() == camera_models::GeometricCamera::CAM_FISHEYE)
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

#ifdef REGISTER_TIMES
double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for (double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for (double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_values)
{
    double accum = 0;
    int    total = 0;
    for (double value : v_values)
    {
        if (value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int    total = 0;
    for (double value : v_values)
    {
        if (value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

void Tracking::localMapStats2File()
{
    ofstream f;
    f.open("LocalMapTimeStats.txt");
    f << fixed << setprecision(6);
    f << "#Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF "
         "culling[ms], Total[ms]"
      << endl;
    for (int i = 0; i < p_localMapper->vdLMTotal_ms.size(); ++i)
    {
        f << p_localMapper->vdKFInsert_ms[i] << ","
          << p_localMapper->vdMPCulling_ms[i] << ","
          << p_localMapper->vdMPCreation_ms[i] << ","
          << p_localMapper->vdLBASync_ms[i] << ","
          << p_localMapper->vdKFCullingSync_ms[i] << ","
          << p_localMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    f.open("LBA_Stats.txt");
    f << fixed << setprecision(6);
    f << "#LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for (int i = 0; i < p_localMapper->vdLBASync_ms.size(); ++i)
    {
        f << p_localMapper->vdLBASync_ms[i] << ","
          << p_localMapper->vnLBA_KFopt[i] << ","
          << p_localMapper->vnLBA_KFfixed[i] << ","
          << p_localMapper->vnLBA_MPs[i] << "," << p_localMapper->vnLBA_edges[i]
          << endl;
    }

    f.close();
}

void Tracking::trackStats2File()
{
    ofstream f;
    f.open("SessionInfo.txt");
    f << fixed;
    f << "Number of KFs: " << p_atlas->getAllKeyFrames().size() << endl;
    f << "Number of MPs: " << p_atlas->getAllMapPoints().size() << endl;

    f << "OpenCV version: " << CV_VERSION << endl;

    f.close();

    f.open("TrackingTimeStats.txt");
    f << fixed << setprecision(6);

    f << "#Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], "
         "IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]"
      << endl;

    for (int i = 0; i < vdTrackTotal_ms.size(); ++i)
    {
        double stereo_rect = 0.0;
        if (!vdRectStereo_ms.empty())
        {
            stereo_rect = vdRectStereo_ms[i];
        }

        double resize_image = 0.0;
        if (!vdResizeImage_ms.empty())
        {
            resize_image = vdResizeImage_ms[i];
        }

        double stereo_match = 0.0;
        if (!vdStereoMatch_ms.empty())
        {
            stereo_match = vdStereoMatch_ms[i];
        }

        double imu_preint = 0.0;
        if (!vdIMUInteg_ms.empty())
        {
            imu_preint = vdIMUInteg_ms[i];
        }

        f << stereo_rect << "," << resize_image << "," << vdORBExtract_ms[i]
          << "," << stereo_match << "," << imu_preint << "," << vdPosePred_ms[i]
          << "," << vdLMTrack_ms[i] << "," << vdNewKF_ms[i] << ","
          << vdTrackTotal_ms[i] << endl;
    }

    f.close();
}

void Tracking::printTimeStats()
{
    // Save data in files
    trackStats2File();
    localMapStats2File();

    ofstream f;
    f.open("ExecMean.txt");
    f << fixed;
    // Report the mean and std of each one
    std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    cout << "OpenCV version: " << CV_VERSION << endl;
    f << "OpenCV version: " << CV_VERSION << endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if (!vdRectStereo_ms.empty())
    {
        average   = calcAverage(vdRectStereo_ms);
        deviation = calcDeviation(vdRectStereo_ms, average);
        std::cout << "Stereo Rectification: " << average << "$\\pm$"
                  << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    if (!vdResizeImage_ms.empty())
    {
        average   = calcAverage(vdResizeImage_ms);
        deviation = calcDeviation(vdResizeImage_ms, average);
        std::cout << "Image Resize: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
    }

    average   = calcAverage(vdORBExtract_ms);
    deviation = calcDeviation(vdORBExtract_ms, average);
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if (!vdStereoMatch_ms.empty())
    {
        average   = calcAverage(vdStereoMatch_ms);
        deviation = calcDeviation(vdStereoMatch_ms, average);
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    if (!vdIMUInteg_ms.empty())
    {
        average   = calcAverage(vdIMUInteg_ms);
        deviation = calcDeviation(vdIMUInteg_ms, average);
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    average   = calcAverage(vdPosePred_ms);
    deviation = calcDeviation(vdPosePred_ms, average);
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(vdLMTrack_ms);
    deviation = calcDeviation(vdLMTrack_ms, average);
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(vdNewKF_ms);
    deviation = calcDeviation(vdNewKF_ms, average);
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(vdTrackTotal_ms);
    deviation = calcDeviation(vdTrackTotal_ms, average);
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    average   = calcAverage(p_localMapper->vdKFInsert_ms);
    deviation = calcDeviation(p_localMapper->vdKFInsert_ms, average);
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->vdMPCulling_ms);
    deviation = calcDeviation(p_localMapper->vdMPCulling_ms, average);
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->vdMPCreation_ms);
    deviation = calcDeviation(p_localMapper->vdMPCreation_ms, average);
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->vdLBA_ms);
    deviation = calcDeviation(p_localMapper->vdLBA_ms, average);
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->vdKFCulling_ms);
    deviation = calcDeviation(p_localMapper->vdKFCulling_ms, average);
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->vdLMTotal_ms);
    deviation = calcDeviation(p_localMapper->vdLMTotal_ms, average);
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation
      << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average   = calcAverage(p_localMapper->vnLBA_edges);
    deviation = calcDeviation(p_localMapper->vnLBA_edges, average);
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->vnLBA_KFopt);
    deviation = calcDeviation(p_localMapper->vnLBA_KFopt, average);
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->vnLBA_KFfixed);
    deviation = calcDeviation(p_localMapper->vnLBA_KFfixed, average);
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->vnLBA_MPs);
    deviation = calcDeviation(p_localMapper->vnLBA_MPs, average);
    std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
              << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;

    std::cout << "LBA executions: " << p_localMapper->nLBA_exec << std::endl;
    std::cout << "LBA aborts: " << p_localMapper->nLBA_abort << std::endl;
    f << "LBA executions: " << p_localMapper->nLBA_exec << std::endl;
    f << "LBA aborts: " << p_localMapper->nLBA_abort << std::endl;

    // Map complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::cout << "KFs in map: " << p_atlas->getAllKeyFrames().size()
              << std::endl;
    std::cout << "MPs in map: " << p_atlas->getAllMapPoints().size()
              << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    vector<Map *> vpMaps   = p_atlas->getAllMaps();
    Map          *pBestMap = vpMaps[0];
    for (int i = 1; i < vpMaps.size(); ++i)
    {
        if (pBestMap->getAllKeyFrames().size() <
            vpMaps[i]->getAllKeyFrames().size())
        {
            pBestMap = vpMaps[i];
        }
    }

    f << "KFs in map: " << pBestMap->getAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << pBestMap->getAllMapPoints().size() << std::endl;

    f << "---------------------------" << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    average   = calcAverage(p_loopClosing->vdDataQuery_ms);
    deviation = calcDeviation(p_loopClosing->vdDataQuery_ms, average);
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Database Query: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vdEstSim3_ms);
    deviation = calcDeviation(p_loopClosing->vdEstSim3_ms, average);
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vdPRTotal_ms);
    deviation = calcDeviation(p_loopClosing->vdPRTotal_ms, average);
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation
      << std::endl
      << std::endl;
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    average   = calcAverage(p_loopClosing->vdLoopFusion_ms);
    deviation = calcDeviation(p_loopClosing->vdLoopFusion_ms, average);
    f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vdLoopOptEss_ms);
    deviation = calcDeviation(p_loopClosing->vdLoopOptEss_ms, average);
    f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Essential Graph: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vdLoopTotal_ms);
    deviation = calcDeviation(p_loopClosing->vdLoopTotal_ms, average);
    f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << "Numb exec: " << p_loopClosing->nLoop << std::endl;
    std::cout << "Num exec: " << p_loopClosing->nLoop << std::endl;
    average   = calcAverage(p_loopClosing->vnLoopKFs);
    deviation = calcDeviation(p_loopClosing->vnLoopKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;

    f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    average   = calcAverage(p_loopClosing->vdMergeMaps_ms);
    deviation = calcDeviation(p_loopClosing->vdMergeMaps_ms, average);
    f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Merge Maps: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vdWeldingBA_ms);
    deviation = calcDeviation(p_loopClosing->vdWeldingBA_ms, average);
    f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Welding BA: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vdMergeOptEss_ms);
    deviation = calcDeviation(p_loopClosing->vdMergeOptEss_ms, average);
    f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vdMergeTotal_ms);
    deviation = calcDeviation(p_loopClosing->vdMergeTotal_ms, average);
    f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << "Numb exec: " << p_loopClosing->nMerges << std::endl;
    std::cout << "Num exec: " << p_loopClosing->nMerges << std::endl;
    average   = calcAverage(p_loopClosing->vnMergeKFs);
    deviation = calcDeviation(p_loopClosing->vnMergeKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vnMergeMPs);
    deviation = calcDeviation(p_loopClosing->vnMergeMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
              << std::endl;

    f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    average   = calcAverage(p_loopClosing->vdGBA_ms);
    deviation = calcDeviation(p_loopClosing->vdGBA_ms, average);
    f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    average   = calcAverage(p_loopClosing->vdUpdateMap_ms);
    deviation = calcDeviation(p_loopClosing->vdUpdateMap_ms, average);
    f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Map Update: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vdFGBATotal_ms);
    deviation = calcDeviation(p_loopClosing->vdFGBATotal_ms, average);
    f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << "Numb exec: " << p_loopClosing->nFGBA_exec << std::endl;
    std::cout << "Num exec: " << p_loopClosing->nFGBA_exec << std::endl;
    f << "Numb abort: " << p_loopClosing->nFGBA_abort << std::endl;
    std::cout << "Num abort: " << p_loopClosing->nFGBA_abort << std::endl;
    average   = calcAverage(p_loopClosing->vnGBAKFs);
    deviation = calcDeviation(p_loopClosing->vnGBAKFs, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->vnGBAMPs);
    deviation = calcDeviation(p_loopClosing->vnGBAMPs, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
              << std::endl;

    f.close();
}

#endif

Tracking::~Tracking() {}

void Tracking::newParameterLoader(Settings *settings)
{
    p_camera = settings->camera1();
    p_camera = p_atlas->addCamera(p_camera);

    if (settings->needToUndistort())
    {
        distortionCoefficients = settings->camera1DistortionCoef();
    }
    else
    {
        distortionCoefficients = cv::Mat::zeros(4, 1, CV_32F);
    }

    // TODO: missing image scaling and rectification
    imageScale = 1.0f;

    calibrationMatrix                 = cv::Mat::eye(3, 3, CV_32F);
    calibrationMatrix.at<float>(0, 0) = p_camera->getParameter(0);
    calibrationMatrix.at<float>(1, 1) = p_camera->getParameter(1);
    calibrationMatrix.at<float>(0, 2) = p_camera->getParameter(2);
    calibrationMatrix.at<float>(1, 2) = p_camera->getParameter(3);

    calibrationMatrixEigen.setIdentity();
    calibrationMatrixEigen(0, 0) = p_camera->getParameter(0);
    calibrationMatrixEigen(1, 1) = p_camera->getParameter(1);
    calibrationMatrixEigen(0, 2) = p_camera->getParameter(2);
    calibrationMatrixEigen(1, 2) = p_camera->getParameter(3);

    if ((sensor == System::STEREO || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        settings->cameraType() == Settings::CameraType::KANNALA_BRANDT)
    {
        p_camera2 = settings->camera2();
        p_camera2 = p_atlas->addCamera(p_camera2);

        poseTlr = settings->getLeftToRightTransform();

        p_frameDrawer->both = true;
    }

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        mbf            = settings->getBaselineFocal();
        depthThreshold = settings->b() * settings->thDepth();
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD)
    {
        depthMapFactor = settings->depthMapFactor();
        if (fabs(depthMapFactor) < 1e-5)
            depthMapFactor = 1;
        else
            depthMapFactor = 1.0f / depthMapFactor;
    }

    minFrames  = 0;
    maxFrames  = settings->getFramesPerSecond();
    rgbEnabled = settings->isRgbEnabled();

    // ORB parameters
    int   nFeatures    = settings->nFeatures();
    int   nLevels      = settings->nLevels();
    int   fIniThFAST   = settings->initThFAST();
    int   fMinThFAST   = settings->getMinimumFastThreshold();
    float fScaleFactor = settings->scaleFactor();

    p_orbExtractorLeft = new ORBextractor(nFeatures,
                                          fScaleFactor,
                                          nLevels,
                                          fIniThFAST,
                                          fMinThFAST);

    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        p_orbExtractorRight = new ORBextractor(nFeatures,
                                               fScaleFactor,
                                               nLevels,
                                               fIniThFAST,
                                               fMinThFAST);

    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
        p_iniOrbExtractor = new ORBextractor(5 * nFeatures,
                                             fScaleFactor,
                                             nLevels,
                                             fIniThFAST,
                                             fMinThFAST);

    // Adaptive FAST threshold initialization
    lastFrameFeatures        = 0;
    consecutiveLowFeatures   = 0;
    baseInitialFastThreshold = fIniThFAST;
    baseMinimumFastThreshold = fMinThFAST;

    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
    {
        Sophus::SE3f Tbc = settings->Tbc();
        imuFrequency     = settings->imuFrequency();
        imuThresh        = settings->imuThreshold();
        insertKFsLost    = settings->insertKFsWhenLost();
        fastInit         = settings->fastInit();
        imuPeriod        = 1.0 / static_cast<double>(imuFrequency);
        float Ng         = settings->noiseGyro();
        float Na         = settings->noiseAcc();
        float Ngw        = settings->gyroWalk();
        float Naw        = settings->accWalk();

        const float sf = sqrt(imuFrequency);
        p_imuCalibration =
            new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
    }
}

void Tracking::loadTrackingParameters(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    auto readInt = [&fSettings](const char *name,
                                int         minimum,
                                int         maximum,
                                int         defaultValue)
    {
        const cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            return defaultValue;
        }

        if (!node.isInt())
        {
            std::cerr << "[Tracking] Ignoring '" << name
                      << "': expected an integer in [" << minimum << ", "
                      << maximum << "]. Using " << defaultValue << "."
                      << std::endl;
            return defaultValue;
        }

        const int value = node.operator int();
        if (value < minimum || value > maximum)
        {
            std::cerr << "[Tracking] Ignoring '" << name << "': " << value
                      << " is outside [" << minimum << ", " << maximum
                      << "]. Using " << defaultValue << "." << std::endl;
            return defaultValue;
        }
        return value;
    };

    auto readReal = [&fSettings](const char *name,
                                 double      minimum,
                                 double      maximum,
                                 double      defaultValue)
    {
        const cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            return defaultValue;
        }

        if (!node.isReal())
        {
            std::cerr << "[Tracking] Ignoring '" << name
                      << "': expected a real number in [" << minimum << ", "
                      << maximum << "]. Using " << defaultValue << "."
                      << std::endl;
            return defaultValue;
        }

        const double value = node.real();
        if (!std::isfinite(value) || value < minimum || value > maximum)
        {
            std::cerr << "[Tracking] Ignoring '" << name << "': " << value
                      << " is outside [" << minimum << ", " << maximum
                      << "]. Using " << defaultValue << "." << std::endl;
            return defaultValue;
        }
        return value;
    };

    minInliersForKF =
        readInt("Tracking.MinInliersForKF", 1, 1000, minInliersForKF);
    minCloseInliersForKF =
        readInt("Tracking.MinCloseInliersForKF", 0, 1000, minCloseInliersForKF);
    if (minCloseInliersForKF > minInliersForKF)
    {
        std::cerr << "[Tracking] Tracking.MinCloseInliersForKF exceeds "
                     "Tracking.MinInliersForKF. Using "
                  << minInliersForKF << "." << std::endl;
        minCloseInliersForKF = minInliersForKF;
    }

    mdMinTemporalSpacingKF = readReal("Tracking.MinTemporalSpacingKF",
                                      0.0,
                                      60.0,
                                      mdMinTemporalSpacingKF);
    maxKFsInLocalMap =
        readInt("Tracking.MaxKFsInLocalMap", 1, 10000, maxKFsInLocalMap);
    mfMotionModelSearchRadiusMultiplier = static_cast<float>(
        readReal("Tracking.MotionModelSearchRadiusMultiplier",
                 1.0,
                 4.0,
                 mfMotionModelSearchRadiusMultiplier));
    motionModelMaxSearchRadius = readInt("Tracking.MotionModelMaxSearchRadius",
                                         15,
                                         200,
                                         motionModelMaxSearchRadius);
    initializationMinPoints    = readInt("Tracking.InitializationMinPoints",
                                      1,
                                      10000,
                                      initializationMinPoints);
    relocalizationMinInliers   = readInt("Tracking.RelocalizationMinInliers",
                                       6,
                                       1000,
                                       relocalizationMinInliers);

    cout << endl << "Effective Tracking Parameters:" << endl;
    cout << "- Min Inliers for KF: " << minInliersForKF << endl;
    cout << "- Min Close Inliers for KF: " << minCloseInliersForKF << endl;
    cout << "- Min Temporal Spacing KF: " << mdMinTemporalSpacingKF << " s"
         << endl;
    cout << "- Max KFs in Local Map: " << maxKFsInLocalMap << endl;
    cout << "- Motion Model Search Radius Multiplier: "
         << mfMotionModelSearchRadiusMultiplier << endl;
    cout << "- Motion Model Max Search Radius: " << motionModelMaxSearchRadius
         << endl;
    cout << "- Initialization Min Points: " << initializationMinPoints << endl;
    cout << "- Relocalization Min Inliers: " << relocalizationMinInliers
         << endl;
}

// Adaptive FAST threshold: lower thresholds when tracking degrades
// In low-texture corridors, fewer features are extracted, so we lower the
// threshold
void Tracking::adjustFASTThreshold()
{
    // Count features in current frame
    int nCurrentFeatures = currentFrame.N;

    // If this is the first frame after initialization, just record
    if (lastFrameFeatures == 0)
    {
        lastFrameFeatures = nCurrentFeatures;
        return;
    }

    // Check if feature count dropped significantly
    float featureRatio =
        (float)nCurrentFeatures / (float)std::max(1, lastFrameFeatures);

    // If features dropped below 50% of previous (more sensitive), or absolute
    // count is very low
    bool lowFeatures = (featureRatio < 0.5f) || (nCurrentFeatures < 400);

    if (lowFeatures)
    {
        consecutiveLowFeatures++;
    }
    else
    {
        consecutiveLowFeatures = 0;
    }

    // Adjust thresholds based on consecutive low-feature frames
    // Lower thresholds to extract more features in textureless areas
    int newIniThFAST = baseInitialFastThreshold;
    int newMinThFAST = baseMinimumFastThreshold;

    if (consecutiveLowFeatures >= 1) // React faster - after just 1 frame
    {
        // Progressively lower thresholds (but not below minimum)
        // Each step reduces by 3, minimum of 1 for both (more aggressive)
        int reduction = std::min(consecutiveLowFeatures, 6) * 3;
        newIniThFAST  = std::max(baseInitialFastThreshold - reduction, 1);
        newMinThFAST  = std::max(baseMinimumFastThreshold - reduction, 1);
    }
    else if (consecutiveLowFeatures == 0 && nCurrentFeatures > 2500)
    {
        // Plenty of features - can restore base thresholds
        newIniThFAST = baseInitialFastThreshold;
        newMinThFAST = baseMinimumFastThreshold;
    }

    // Apply new thresholds if changed
    if (newIniThFAST != p_orbExtractorLeft->getInitialFastThreshold() ||
        newMinThFAST != p_orbExtractorLeft->getMinimumFastThreshold())
    {
        p_orbExtractorLeft->setInitialFastThreshold(newIniThFAST);
        p_orbExtractorLeft->setMinimumFastThreshold(newMinThFAST);
        if (p_orbExtractorRight)
        {
            p_orbExtractorRight->setInitialFastThreshold(newIniThFAST);
            p_orbExtractorRight->setMinimumFastThreshold(newMinThFAST);
        }
        if (p_iniOrbExtractor)
        {
            p_iniOrbExtractor->setInitialFastThreshold(newIniThFAST);
            p_iniOrbExtractor->setMinimumFastThreshold(newMinThFAST);
        }
        Verbose::printMess(
            "[Tracking] Adaptive FAST: iniTh=" + std::to_string(newIniThFAST) +
                " minTh=" + std::to_string(newMinThFAST) +
                " (features=" + std::to_string(nCurrentFeatures) +
                " consecutive_low=" + std::to_string(consecutiveLowFeatures) +
                ")",
            Verbose::VERBOSITY_NORMAL);
    }

    lastFrameFeatures = nCurrentFeatures;
}

bool Tracking::parseCamParamFile(cv::FileStorage &fSettings)
{
    distortionCoefficients = cv::Mat::zeros(4, 1, CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];
    if (sCameraName == "PinHole")
    {
        float fx   = 0.0F;
        float fy   = 0.0F;
        float cx   = 0.0F;
        float cy   = 0.0F;
        imageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if (!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if (!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if (!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if (!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.at<float>(0) = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.at<float>(1) = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.at<float>(2) = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.p1 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.at<float>(3) = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.p2 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if (!node.empty() && node.isReal())
        {
            distortionCoefficients.resize(5);
            distortionCoefficients.at<float>(4) = node.real();
        }

        node = fSettings["Camera.imageScale"];
        if (!node.empty() && node.isReal())
        {
            imageScale = node.real();
        }

        if (b_miss_params)
        {
            return false;
        }

        if (imageScale != 1.f)
        {
            // K matrix parameters must be scaled.
            fx = fx * imageScale;
            fy = fy * imageScale;
            cx = cx * imageScale;
            cy = cy * imageScale;
        }

        vector<float> vCamCalib{fx, fy, cx, cy};

        p_camera = new camera_models::Pinhole(vCamCalib);

        p_camera = p_atlas->addCamera(p_camera);

        std::cout << "- Camera: camera_models::Pinhole" << std::endl;
        std::cout << "- Image scale: " << imageScale << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << distortionCoefficients.at<float>(0)
                  << std::endl;
        std::cout << "- k2: " << distortionCoefficients.at<float>(1)
                  << std::endl;

        std::cout << "- p1: " << distortionCoefficients.at<float>(2)
                  << std::endl;
        std::cout << "- p2: " << distortionCoefficients.at<float>(3)
                  << std::endl;

        if (distortionCoefficients.rows == 5)
            std::cout << "- k3: " << distortionCoefficients.at<float>(4)
                      << std::endl;

        calibrationMatrix                 = cv::Mat::eye(3, 3, CV_32F);
        calibrationMatrix.at<float>(0, 0) = fx;
        calibrationMatrix.at<float>(1, 1) = fy;
        calibrationMatrix.at<float>(0, 2) = cx;
        calibrationMatrix.at<float>(1, 2) = cy;

        calibrationMatrixEigen.setIdentity();
        calibrationMatrixEigen(0, 0) = fx;
        calibrationMatrixEigen(1, 1) = fy;
        calibrationMatrixEigen(0, 2) = cx;
        calibrationMatrixEigen(1, 2) = cy;
    }
    else if (sCameraName == "camera_models::KannalaBrandt8")
    {
        float fx   = 0.0F;
        float fy   = 0.0F;
        float cx   = 0.0F;
        float cy   = 0.0F;
        float k1   = 0.0F;
        float k2   = 0.0F;
        float k3   = 0.0F;
        float k4   = 0.0F;
        imageScale = 1.f;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if (!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.fx parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if (!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.fy parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if (!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.cx parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if (!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.cy parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if (!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k1 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if (!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k2 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if (!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k3 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if (!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr
                << "*Camera.k4 parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.imageScale"];
        if (!node.empty() && node.isReal())
        {
            imageScale = node.real();
        }

        if (!b_miss_params)
        {
            if (imageScale != 1.f)
            {
                // K matrix parameters must be scaled.
                fx = fx * imageScale;
                fy = fy * imageScale;
                cx = cx * imageScale;
                cy = cy * imageScale;
            }

            vector<float> vCamCalib{fx, fy, cx, cy, k1, k2, k3, k4};
            p_camera = new camera_models::KannalaBrandt8(vCamCalib);
            p_camera = p_atlas->addCamera(p_camera);
            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- Image scale: " << imageScale << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            calibrationMatrix                 = cv::Mat::eye(3, 3, CV_32F);
            calibrationMatrix.at<float>(0, 0) = fx;
            calibrationMatrix.at<float>(1, 1) = fy;
            calibrationMatrix.at<float>(0, 2) = cx;
            calibrationMatrix.at<float>(1, 2) = cy;

            calibrationMatrixEigen.setIdentity();
            calibrationMatrixEigen(0, 0) = fx;
            calibrationMatrixEigen(1, 1) = fy;
            calibrationMatrixEigen(0, 2) = cx;
            calibrationMatrixEigen(1, 2) = cy;
        }

        if (sensor == System::STEREO || sensor == System::IMU_STEREO ||
            sensor == System::IMU_RGBD)
        {
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if (!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if (!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if (!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if (!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if (!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if (!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if (!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if (!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a "
                             "real number*"
                          << std::endl;
                b_miss_params = true;
            }

            int leftLappingBegin = -1;
            int leftLappingEnd   = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd   = -1;

            node = fSettings["Camera.lappingBegin"];
            if (!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout
                    << "WARNING: Camera.lappingBegin not correctly defined"
                    << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if (!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined"
                          << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if (!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout
                    << "WARNING: Camera2.lappingBegin not correctly defined"
                    << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if (!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined"
                          << std::endl;
            }

            node = fSettings["Tlr"];
            cv::Mat cvTlr;
            if (!node.empty())
            {
                cvTlr = node.mat();
                if (cvTlr.rows != 3 || cvTlr.cols != 4)
                {
                    std::cerr
                        << "*Tlr matrix have to be a 3x4 transformation matrix*"
                        << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if (!b_miss_params)
            {
                if (imageScale != 1.f)
                {
                    // K matrix parameters must be scaled.
                    fx = fx * imageScale;
                    fy = fy * imageScale;
                    cx = cx * imageScale;
                    cy = cy * imageScale;

                    leftLappingBegin  = leftLappingBegin * imageScale;
                    leftLappingEnd    = leftLappingEnd * imageScale;
                    rightLappingBegin = rightLappingBegin * imageScale;
                    rightLappingEnd   = rightLappingEnd * imageScale;
                }

                static_cast<camera_models::KannalaBrandt8 *>(p_camera)
                    ->lappingArea[0] = leftLappingBegin;
                static_cast<camera_models::KannalaBrandt8 *>(p_camera)
                    ->lappingArea[1] = leftLappingEnd;

                p_frameDrawer->both = true;

                vector<float> vCamCalib2{fx, fy, cx, cy, k1, k2, k3, k4};
                p_camera2 = new camera_models::KannalaBrandt8(vCamCalib2);
                p_camera2 = p_atlas->addCamera(p_camera2);

                poseTlr = Converter::toSophus(cvTlr);

                static_cast<camera_models::KannalaBrandt8 *>(p_camera2)
                    ->lappingArea[0] = rightLappingBegin;
                static_cast<camera_models::KannalaBrandt8 *>(p_camera2)
                    ->lappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", "
                          << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- Image scale: " << imageScale << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << cvTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", "
                          << rightLappingEnd << std::endl;
            }
        }

        if (b_miss_params)
        {
            return false;
        }
    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr
            << "Check an example configuration file with the desired sensor"
            << std::endl;
    }

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if (!node.empty() && node.isReal())
        {
            mbf = node.real();
            if (imageScale != 1.f)
            {
                mbf *= imageScale;
            }
        }
        else
        {
            std::cerr
                << "*Camera.bf parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }
    }

    float fps = fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    minFrames = 0;
    maxFrames = fps;

    cout << "- fps: " << fps << endl;

    int nRGB   = fSettings["Camera.RGB"];
    rgbEnabled = nRGB;

    if (rgbEnabled)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if (sensor == System::STEREO || sensor == System::RGBD ||
        sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        float        fx   = p_camera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if (!node.empty() && node.isReal())
        {
            depthThreshold = node.real();
            depthThreshold = mbf * depthThreshold / fx;
            cout << endl
                 << "Depth Threshold (Close/Far Points): " << depthThreshold
                 << endl;
        }
        else
        {
            std::cerr
                << "*ThDepth parameter doesn't exist or is not a real number*"
                << std::endl;
            b_miss_params = true;
        }
    }

    if (sensor == System::RGBD || sensor == System::IMU_RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if (!node.empty() && node.isReal())
        {
            depthMapFactor = node.real();
            if (fabs(depthMapFactor) < 1e-5)
                depthMapFactor = 1;
            else
                depthMapFactor = 1.0f / depthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a "
                         "real number*"
                      << std::endl;
            b_miss_params = true;
        }
    }

    if (b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::parseORBParamFile(cv::FileStorage &fSettings)
{
    bool  b_miss_params = false;
    int   nFeatures     = 0;
    int   nLevels       = 0;
    int   fIniThFAST    = 0;
    int   fMinThFAST    = 0;
    float fScaleFactor  = 0.0F;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if (!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if (!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is "
                     "not a real number*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if (!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not "
                     "an integer*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if (!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if (!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is "
                     "not an integer*"
                  << std::endl;
        b_miss_params = true;
    }

    if (b_miss_params)
    {
        return false;
    }

    p_orbExtractorLeft = new ORBextractor(nFeatures,
                                          fScaleFactor,
                                          nLevels,
                                          fIniThFAST,
                                          fMinThFAST);

    if (sensor == System::STEREO || sensor == System::IMU_STEREO)
        p_orbExtractorRight = new ORBextractor(nFeatures,
                                               fScaleFactor,
                                               nLevels,
                                               fIniThFAST,
                                               fMinThFAST);

    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
        p_iniOrbExtractor = new ORBextractor(5 * nFeatures,
                                             fScaleFactor,
                                             nLevels,
                                             fIniThFAST,
                                             fMinThFAST);

    // Adaptive FAST threshold initialization
    lastFrameFeatures        = 0;
    consecutiveLowFeatures   = 0;
    baseInitialFastThreshold = fIniThFAST;
    baseMinimumFastThreshold = fMinThFAST;

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

bool Tracking::parseIMUParamFile(cv::FileStorage &fSettings)
{
    bool  boolMissingParam = false;
    float Ng               = 0.0F;
    float Na               = 0.0F;
    float Ngw              = 0.0F;
    float Naw              = 0.0F;

    cv::Mat      cvTbc;
    cv::FileNode node = fSettings["Tbc"];
    if (!node.empty())
    {
        cvTbc = node.mat();
        if (cvTbc.rows != 4 || cvTbc.cols != 4)
        {
            std::cerr
                << "\t- Tbc matrix needs to be a 4x4 transformation matrix!"
                << std::endl;
            return false;
        }
    }
    else
    {
        std::cerr << "\t- Tbc matrix does not exist!" << std::endl;
        return false;
    }
    cout << "\t- Left camera to Imu Transform (Tbc): " << endl << cvTbc << endl;
    Eigen::Matrix<float, 4, 4, Eigen::RowMajor> eigTbc(cvTbc.ptr<float>(0));
    Sophus::SE3f                                Tbc(eigTbc);

    node          = fSettings["InsertKFsWhenLost"];
    insertKFsLost = true;
    if (!node.empty() && node.isInt())
    {
        insertKFsLost = (bool)node.operator int();
    }

    if (!insertKFsLost)
        cout << "Do not insert keyframes when lost visual tracking " << endl;

    node = fSettings["IMU.Frequency"];
    if (!node.empty() && node.isInt())
    {
        imuFrequency = node.operator int();
        imuPeriod    = 1.0 / static_cast<double>(imuFrequency);
    }
    else
    {
        std::cerr
            << "*IMU.Frequency parameter doesn't exist or is not an integer*"
            << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if (!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr
            << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*"
            << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.Threshold"];
    if (!node.empty() && node.isReal())
        imuThresh = node.real();
    else
    {
        std::cerr << "- IMU.Threshold parameter doesn't exist or is not a real "
                     "number!"
                  << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if (!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr
            << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*"
            << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if (!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr
            << "*IMU.GyroWalk parameter doesn't exist or is not a real number*"
            << std::endl;
        boolMissingParam = true;
    }

    node = fSettings["IMU.AccWalk"];
    if (!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr
            << "*IMU.AccWalk parameter doesn't exist or is not a real number*"
            << std::endl;
        boolMissingParam = true;
    }

    node     = fSettings["IMU.FastInit"];
    fastInit = false;
    if (!node.empty())
        fastInit = static_cast<int>(fSettings["IMU.FastInit"]) != 0;

    if (fastInit)
        std::cout << "\t- Fast IMU initialization triggered! Acceleration is "
                     "not checked!"
                  << std::endl;

    if (boolMissingParam)
        return false;

    const float sf = sqrt(imuFrequency);
    cout << endl;
    cout << "IMU frequency: " << imuFrequency << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    p_imuCalibration =
        new IMU::Calib(Tbc, Ng * sf, Na * sf, Ngw / sf, Naw / sf);

    p_imuPreintegratedFromLastKF =
        new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);

    return true;
}

void Tracking::setLocalMapper(LocalMapping *pLocalMapper)
{
    p_localMapper = pLocalMapper;
}

void Tracking::setLoopClosing(LoopClosing *pLoopClosing)
{
    p_loopClosing = pLoopClosing;
}

void Tracking::setViewer(Viewer *pViewer)
{
    p_viewer = pViewer;
}

void Tracking::setStepByStep(bool bSet)
{
    stepByStep = bSet;
}

bool Tracking::getStepByStep()
{
    return stepByStep;
}

Sophus::SE3f
    Tracking::grabImageStereo(const cv::Mat                        &imRectLeft,
                              const cv::Mat                        &imRectRight,
                              const double                         &timestamp,
                              string                                filename,
                              const std::vector<semantic::Marker *> markers,
                              const std::vector<semantic::Room *>   rooms)
{
    // Set arguments to local variables
    env_rooms = rooms;

    // Adaptive FAST threshold: adjust before feature extraction
    adjustFASTThreshold();

    imageGray           = imRectLeft;
    cv::Mat imGrayRight = imRectRight;
    imageRight          = imRectRight;

    if (imageGray.channels() == 3)
    {
        if (rgbEnabled)
        {
            cvtColor(imageGray, imageGray, cv::COLOR_RGB2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
        }
        else
        {
            cvtColor(imageGray, imageGray, cv::COLOR_BGR2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
        }
    }
    else if (imageGray.channels() == 4)
    {
        if (rgbEnabled)
        {
            cvtColor(imageGray, imageGray, cv::COLOR_RGBA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
        }
        else
        {
            cvtColor(imageGray, imageGray, cv::COLOR_BGRA2GRAY);
            cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
        }
    }

    if (sensor == System::STEREO && !p_camera2)
        currentFrame = Frame(imRectLeft,
                             imageGray,
                             imGrayRight,
                             timestamp,
                             p_orbExtractorLeft,
                             p_orbExtractorRight,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             nullptr,
                             IMU::Calib(),
                             markers);
    else if (sensor == System::STEREO && p_camera2)
        currentFrame = Frame(imRectLeft,
                             imageGray,
                             imGrayRight,
                             timestamp,
                             p_orbExtractorLeft,
                             p_orbExtractorRight,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             p_camera2,
                             poseTlr,
                             nullptr,
                             IMU::Calib(),
                             markers);
    else if (sensor == System::IMU_STEREO && !p_camera2)
        currentFrame = Frame(imRectLeft,
                             imageGray,
                             imGrayRight,
                             timestamp,
                             p_orbExtractorLeft,
                             p_orbExtractorRight,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             &lastFrame,
                             *p_imuCalibration,
                             markers);
    else if (sensor == System::IMU_STEREO && p_camera2)
        currentFrame = Frame(imRectLeft,
                             imageGray,
                             imGrayRight,
                             timestamp,
                             p_orbExtractorLeft,
                             p_orbExtractorRight,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             p_camera2,
                             poseTlr,
                             &lastFrame,
                             *p_imuCalibration,
                             markers);

    currentFrame.fileName  = filename;
    currentFrame.datasetId = numDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(currentFrame.orbExtractionTime);
    vdStereoMatch_ms.push_back(currentFrame.stereoMatchTime);
#endif

    track();

    return currentFrame.getPose();
}

Sophus::SE3f Tracking::grabImageRGBD(
    const cv::Mat                                &imRGB,
    const cv::Mat                                &imD,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pointcloud,
    const double                                 &timestamp,
    string                                        filename,
    const std::vector<semantic::Marker *>         markers,
    const std::vector<semantic::Room *>           rooms)
{
    // Set arguments to local variables
    env_rooms = rooms;

    // Adaptive FAST threshold: adjust before feature extraction
    adjustFASTThreshold();

    imageGray       = imRGB;
    cv::Mat imDepth = imD;

    if (imageGray.channels() == 3)
    {
        if (rgbEnabled)
            cvtColor(imageGray, imageGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(imageGray, imageGray, cv::COLOR_BGR2GRAY);
    }
    else if (imageGray.channels() == 4)
    {
        if (rgbEnabled)
            cvtColor(imageGray, imageGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(imageGray, imageGray, cv::COLOR_BGRA2GRAY);
    }

    if ((fabs(depthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
        imDepth.convertTo(imDepth, CV_32F, depthMapFactor);

    // RGB-D
    if (sensor == System::RGBD)
        currentFrame = Frame(imRGB,
                             imageGray,
                             imDepth,
                             pointcloud,
                             timestamp,
                             p_orbExtractorLeft,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             nullptr,
                             IMU::Calib(),
                             markers);
    // RGB-D Intertial
    else if (sensor == System::IMU_RGBD)
        currentFrame = Frame(imRGB,
                             imageGray,
                             imDepth,
                             pointcloud,
                             timestamp,
                             p_orbExtractorLeft,
                             p_orbVocabulary,
                             calibrationMatrix,
                             distortionCoefficients,
                             mbf,
                             depthThreshold,
                             p_camera,
                             &lastFrame,
                             *p_imuCalibration,
                             markers);

    currentFrame.fileName  = filename;
    currentFrame.datasetId = numDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(currentFrame.orbExtractionTime);
#endif

    track();

    return currentFrame.getPose();
}

Sophus::SE3f
    Tracking::grabImageMonocular(const cv::Mat &im,
                                 const double  &timestamp,
                                 string         filename,
                                 const std::vector<semantic::Marker *> markers,
                                 const std::vector<semantic::Room *>   rooms)
{
    // Set arguments to local variables
    env_rooms = rooms;

    // Adaptive FAST threshold: adjust before feature extraction
    adjustFASTThreshold();

    imageGray = im;
    if (imageGray.channels() == 3)
    {
        if (rgbEnabled)
            cvtColor(imageGray, imageGray, cv::COLOR_RGB2GRAY);
        else
            cvtColor(imageGray, imageGray, cv::COLOR_BGR2GRAY);
    }
    else if (imageGray.channels() == 4)
    {
        if (rgbEnabled)
            cvtColor(imageGray, imageGray, cv::COLOR_RGBA2GRAY);
        else
            cvtColor(imageGray, imageGray, cv::COLOR_BGRA2GRAY);
    }

    if (sensor == System::MONOCULAR)
    {
        if (state == NOT_INITIALIZED || state == NO_IMAGES_YET ||
            (lastId - initId) < maxFrames)
            currentFrame = Frame(im,
                                 imageGray,
                                 timestamp,
                                 p_iniOrbExtractor,
                                 p_orbVocabulary,
                                 p_camera,
                                 distortionCoefficients,
                                 mbf,
                                 depthThreshold,
                                 nullptr,
                                 IMU::Calib(),
                                 markers);
        else
            currentFrame = Frame(im,
                                 imageGray,
                                 timestamp,
                                 p_orbExtractorLeft,
                                 p_orbVocabulary,
                                 p_camera,
                                 distortionCoefficients,
                                 mbf,
                                 depthThreshold,
                                 nullptr,
                                 IMU::Calib(),
                                 markers);
    }
    else if (sensor == System::IMU_MONOCULAR)
    {
        if (state == NOT_INITIALIZED || state == NO_IMAGES_YET)
        {
            currentFrame = Frame(im,
                                 imageGray,
                                 timestamp,
                                 p_iniOrbExtractor,
                                 p_orbVocabulary,
                                 p_camera,
                                 distortionCoefficients,
                                 mbf,
                                 depthThreshold,
                                 &lastFrame,
                                 *p_imuCalibration,
                                 markers);
        }
        else
            currentFrame = Frame(im,
                                 imageGray,
                                 timestamp,
                                 p_orbExtractorLeft,
                                 p_orbVocabulary,
                                 p_camera,
                                 distortionCoefficients,
                                 mbf,
                                 depthThreshold,
                                 &lastFrame,
                                 *p_imuCalibration,
                                 markers);
    }

    if (state == NO_IMAGES_YET)
        t0 = timestamp;

    currentFrame.fileName  = filename;
    currentFrame.datasetId = numDataset;

#ifdef REGISTER_TIMES
    vdORBExtract_ms.push_back(currentFrame.orbExtractionTime);
#endif

    lastId = currentFrame.mnId;
    track();

    return currentFrame.getPose();
}

void Tracking::grabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    queueImuData.push_back(imuMeasurement);
}

void Tracking::preintegrateIMU()
{
    if (!currentFrame.p_previousFrame)
    {
        Verbose::printMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        currentFrame.setIntegrated();
        return;
    }

    imuFromLastFrame.clear();
    imuFromLastFrame.reserve(queueImuData.size());
    if (queueImuData.size() == 0)
    {
        Verbose::printMess("Not IMU data in mlQueueImuData!!",
                           Verbose::VERBOSITY_NORMAL);
        currentFrame.setIntegrated();
        return;
    }

    while (true)
    {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if (!queueImuData.empty())
            {
                IMU::Point *m = &queueImuData.front();
                cout.precision(17);
                if (m->t < currentFrame.p_previousFrame->timeStamp - imuPeriod)
                    queueImuData.pop_front();
                else if (m->t < currentFrame.timeStamp - imuPeriod)
                {
                    imuFromLastFrame.push_back(*m);
                    queueImuData.pop_front();
                }
                else
                {
                    imuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if (bSleep)
            usleep(500);
    }

    const int n = imuFromLastFrame.size() - 1;
    if (n == 0)
    {
        cout << "Empty IMU measurements vector!!!\n";
        return;
    }

    std::shared_ptr<IMU::Preintegrated> pImuPreintegratedFromLastFrame =
        std::make_shared<IMU::Preintegrated>(lastFrame.imuBias,
                                             currentFrame.imuCalibration);

    for (int i = 0; i < n; i++)
    {
        float           tstep;
        Eigen::Vector3f acc, angVel;
        if ((i == 0) && (i < (n - 1)))
        {
            float tab = imuFromLastFrame[i + 1].t - imuFromLastFrame[i].t;
            float tini =
                imuFromLastFrame[i].t - currentFrame.p_previousFrame->timeStamp;
            acc = (imuFromLastFrame[i].a + imuFromLastFrame[i + 1].a -
                   (imuFromLastFrame[i + 1].a - imuFromLastFrame[i].a) *
                       (tini / tab)) *
                  0.5f;
            angVel = (imuFromLastFrame[i].w + imuFromLastFrame[i + 1].w -
                      (imuFromLastFrame[i + 1].w - imuFromLastFrame[i].w) *
                          (tini / tab)) *
                     0.5f;
            tstep = imuFromLastFrame[i + 1].t -
                    currentFrame.p_previousFrame->timeStamp;
        }
        else if (i < (n - 1))
        {
            acc    = (imuFromLastFrame[i].a + imuFromLastFrame[i + 1].a) * 0.5f;
            angVel = (imuFromLastFrame[i].w + imuFromLastFrame[i + 1].w) * 0.5f;
            tstep  = imuFromLastFrame[i + 1].t - imuFromLastFrame[i].t;
        }
        else if ((i > 0) && (i == (n - 1)))
        {
            float tab  = imuFromLastFrame[i + 1].t - imuFromLastFrame[i].t;
            float tend = imuFromLastFrame[i + 1].t - currentFrame.timeStamp;
            acc        = (imuFromLastFrame[i].a + imuFromLastFrame[i + 1].a -
                   (imuFromLastFrame[i + 1].a - imuFromLastFrame[i].a) *
                       (tend / tab)) *
                  0.5f;
            angVel = (imuFromLastFrame[i].w + imuFromLastFrame[i + 1].w -
                      (imuFromLastFrame[i + 1].w - imuFromLastFrame[i].w) *
                          (tend / tab)) *
                     0.5f;
            tstep = currentFrame.timeStamp - imuFromLastFrame[i].t;
        }
        else if ((i == 0) && (i == (n - 1)))
        {
            acc    = imuFromLastFrame[i].a;
            angVel = imuFromLastFrame[i].w;
            tstep  = currentFrame.timeStamp -
                    currentFrame.p_previousFrame->timeStamp;
        }

        if (!p_imuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        p_imuPreintegratedFromLastKF->integrateNewMeasurement(acc,
                                                              angVel,
                                                              tstep);
        pImuPreintegratedFromLastFrame->integrateNewMeasurement(acc,
                                                                angVel,
                                                                tstep);
    }

    currentFrame.p_imuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    currentFrame.p_imuPreintegrated      = p_imuPreintegratedFromLastKF;
    currentFrame.p_lastKeyFrame          = p_lastKeyFrame;

    currentFrame.setIntegrated();

    // Verbose::PrintMess("Preintegration is finished!! ",
    // Verbose::VERBOSITY_DEBUG);
}

bool Tracking::predictStateIMU()
{
    if (!currentFrame.p_previousFrame)
    {
        Verbose::printMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if (mapUpdated && p_lastKeyFrame)
    {
        const Eigen::Vector3f twb1 = p_lastKeyFrame->getImuPosition();
        const Eigen::Matrix3f Rwb1 = p_lastKeyFrame->getImuRotation();
        const Eigen::Vector3f Vwb1 = p_lastKeyFrame->getVelocity();

        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float           t12 = p_imuPreintegratedFromLastKF->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaRotation(
                       p_lastKeyFrame->getImuBias()));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaPosition(
                       p_lastKeyFrame->getImuBias());
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * p_imuPreintegratedFromLastKF->getDeltaVelocity(
                       p_lastKeyFrame->getImuBias());
        currentFrame.setImuPoseVelocity(Rwb2, twb2, Vwb2);

        currentFrame.imuBias       = p_lastKeyFrame->getImuBias();
        currentFrame.predictedBias = currentFrame.imuBias;
        return true;
    }
    else if (!mapUpdated)
    {
        const Eigen::Vector3f twb1 = lastFrame.getImuPosition();
        const Eigen::Matrix3f Rwb1 = lastFrame.getImuRotation();
        const Eigen::Vector3f Vwb1 = lastFrame.getVelocity();
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const float           t12 = currentFrame.p_imuPreintegratedFrame->dT;

        Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaRotation(
                       lastFrame.imuBias));
        Eigen::Vector3f twb2 =
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaPosition(
                       lastFrame.imuBias);
        Eigen::Vector3f Vwb2 =
            Vwb1 + t12 * Gz +
            Rwb1 * currentFrame.p_imuPreintegratedFrame->getDeltaVelocity(
                       lastFrame.imuBias);

        currentFrame.setImuPoseVelocity(Rwb2, twb2, Vwb2);

        currentFrame.imuBias       = lastFrame.imuBias;
        currentFrame.predictedBias = currentFrame.imuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}

void Tracking::resetFrameIMU()
{
    // TODO To implement...
}

void Tracking::track()
{
    if (stepByStep)
    {
        std::cout << "Waiting for the next step in Tracking ..." << std::endl;
        while (!step && stepByStep)
            usleep(500);
        step = false;
    }

    if (p_localMapper->badImu)
    {
        cout << "[Tracking] Reseting map because the Local Mapper set the 'Bad "
                "IMU' flag ..."
             << endl;
        p_system->requestResetActiveMapWithCause(
            ResetCause::LOCAL_MAPPER_BAD_IMU);
        return;
    }

    Map *pCurrentMap = p_atlas->getCurrentMap();
    if (!pCurrentMap)
    {
        cout << "[ERROR] No active maps found in the ATLAS!" << endl;
        return;
    }

    if (state != NO_IMAGES_YET)
    {
        if (lastFrame.timeStamp > currentFrame.timeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame "
                    "detected!"
                 << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            queueImuData.clear();
            reportResetAttribution(ResetCause::NON_MONOTONIC_SENSOR_TIMESTAMP,
                                   ResetAction::CREATE_MAP_EXECUTION);
            createMapInAtlas();
            return;
        }
        else if (currentFrame.timeStamp > lastFrame.timeStamp + 1.0)
        {
            // cout << mCurrentFrame.timeStamp << ", " << mLastFrame.timeStamp
            // << endl; cout << "id last: " << mLastFrame.mnId << "    id curr:
            // " << mCurrentFrame.mnId << endl;
            if (p_atlas->isInertial())
            {

                if (p_atlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. "
                            "Reseting IMU integration..."
                         << endl;
                    if (!pCurrentMap->getInertialBA2())
                    {
                        p_system->requestResetActiveMapWithCause(
                            ResetCause::TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA);
                    }
                    else
                    {
                        reportResetAttribution(
                            ResetCause::TIMESTAMP_JUMP_AFTER_SECOND_IMU_BA,
                            ResetAction::CREATE_MAP_EXECUTION);
                        createMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU "
                            "initialization. Reseting..."
                         << endl;
                    p_system->requestResetActiveMapWithCause(
                        ResetCause::TIMESTAMP_JUMP_BEFORE_IMU_INITIALIZATION);
                }
                return;
            }
        }
    }

    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        p_lastKeyFrame)
        currentFrame.setNewBias(p_lastKeyFrame->getImuBias());

    if (state == NO_IMAGES_YET)
        state = NOT_INITIALIZED;

    lastProcessedState = state;

    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        !createdMap)
    {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPreIMU =
            std::chrono::steady_clock::now();
#endif
        preintegrateIMU();
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPreIMU =
            std::chrono::steady_clock::now();

        double timePreImu = std::chrono::duration_cast<
                                std::chrono::duration<double, std::milli>>(
                                time_EndPreIMU - time_StartPreIMU)
                                .count();
        vdIMUInteg_ms.push_back(timePreImu);
#endif
    }
    createdMap = false;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->getMapChangeIndex();
    int nMapChangeIndex    = pCurrentMap->getLastMapChange();
    if (nCurMapChangeIndex > nMapChangeIndex)
    {
        pCurrentMap->setLastMapChange(nCurMapChangeIndex);
        mapUpdated = true;
    }

    if (state == NOT_INITIALIZED)
    {
        if (sensor == System::STEREO || sensor == System::RGBD ||
            sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            stereoInitialization();
        else
            monocularInitialization();

        // If initialization succesful, save frame pose
        if (state != OK)
        {
            lastFrame = Frame(currentFrame);
            return;
        }

        if (p_atlas->getAllMaps().size() == 1)
            firstFrameId = currentFrame.mnId;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK = false;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartPosePred =
            std::chrono::steady_clock::now();
#endif

        // Initial camera pose estimation using motion model or relocalization
        // (if tracking is lost)
        if (!onlyTracking)
        {

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            if (state == OK)
            {

                // Local Mapping might have changed some MapPoints tracked in
                // last frame
                checkReplacedInLastFrame();

                if ((!velocityAvailable && !pCurrentMap->isImuInitialized()) ||
                    currentFrame.mnId < lastRelocFrameId + 2)
                {
                    Verbose::printMess(
                        "TRACK: Track with respect to the reference KF ",
                        Verbose::VERBOSITY_DEBUG);
                    bOK = trackReferenceKeyFrame();
                }
                else
                {
                    Verbose::printMess("TRACK: Track with motion model",
                                       Verbose::VERBOSITY_DEBUG);
                    bOK = trackWithMotionModel();
                    if (!bOK)
                        bOK = trackReferenceKeyFrame();
                }

                if (!bOK)
                {
                    if (currentFrame.mnId <=
                            (lastRelocFrameId + framesToResetIMU) &&
                        (sensor == System::IMU_MONOCULAR ||
                         sensor == System::IMU_STEREO ||
                         sensor == System::IMU_RGBD))
                    {
                        state = LOST;
                    }
                    else if (pCurrentMap->getKeyFrameCount() > 10)
                    {
                        // cout << "KF in map: " <<
                        // pCurrentMap->KeyFramesInMap() << endl;
                        state         = RECENTLY_LOST;
                        timeStampLost = currentFrame.timeStamp;
                    }
                    else
                    {
                        state = LOST;
                    }
                }
            }
            else
            {

                if (state == RECENTLY_LOST)
                {
                    Verbose::printMess("Lost for a short time",
                                       Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if ((sensor == System::IMU_MONOCULAR ||
                         sensor == System::IMU_STEREO ||
                         sensor == System::IMU_RGBD))
                    {
                        if (pCurrentMap->isImuInitialized())
                            predictStateIMU();
                        else
                            bOK = false;

                        if (currentFrame.timeStamp - timeStampLost >
                            time_recently_lost)
                        {
                            state = LOST;
                            Verbose::printMess("Track Lost...",
                                               Verbose::VERBOSITY_NORMAL);
                            bOK = false;
                        }
                    }
                    else
                    {
                        // Relocalization
                        bOK = relocalization();
                        // std::cout << "mCurrentFrame.timeStamp:" <<
                        // to_string(mCurrentFrame.timeStamp) << std::endl;
                        // std::cout << "mTimeStampLost:" <<
                        // to_string(mTimeStampLost) << std::endl;
                        if (currentFrame.timeStamp - timeStampLost > 3.0f &&
                            !bOK)
                        {
                            state = LOST;
                            Verbose::printMess("Track Lost...",
                                               Verbose::VERBOSITY_NORMAL);
                            bOK = false;
                        }
                    }
                }
                else if (state == LOST)
                {

                    Verbose::printMess("A new map is started...",
                                       Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->getKeyFrameCount() < 10)
                    {
                        p_system->requestResetActiveMapWithCause(
                            ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP);
                        Verbose::printMess("Reseting current map...",
                                           Verbose::VERBOSITY_NORMAL);
                    }
                    else
                    {
                        reportResetAttribution(
                            ResetCause::VISUAL_TRACKING_LOST_NEW_MAP,
                            ResetAction::CREATE_MAP_EXECUTION);
                        createMapInAtlas();
                    }

                    if (p_lastKeyFrame)
                        p_lastKeyFrame = static_cast<KeyFrame *>(nullptr);

                    Verbose::printMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not
            // available in inertial mode)
            if (state == LOST)
            {
                if (sensor == System::IMU_MONOCULAR ||
                    sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
                    Verbose::printMess("IMU. State LOST",
                                       Verbose::VERBOSITY_NORMAL);
                bOK = relocalization();
            }
            else
            {
                if (!visualOdometry)
                {
                    // In last frame we tracked enough MapPoints in the map
                    if (velocityAvailable)
                    {
                        bOK = trackWithMotionModel();
                    }
                    else
                    {
                        bOK = trackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and
                    // one doing relocalization. If relocalization is sucessfull
                    // we choose that solution, otherwise we retain the "visual
                    // odometry" solution.

                    bool               bOKMM    = false;
                    bool               bOKReloc = false;
                    vector<MapPoint *> vpMPsMM;
                    vector<bool>       vbOutMM;
                    Sophus::SE3f       TcwMM;
                    if (velocityAvailable)
                    {
                        bOKMM   = trackWithMotionModel();
                        vpMPsMM = currentFrame.mapPoints;
                        vbOutMM = currentFrame.outlierFlags;
                        TcwMM   = currentFrame.getPose();
                    }
                    bOKReloc = relocalization();

                    if (bOKMM && !bOKReloc)
                    {
                        currentFrame.setPose(TcwMM);
                        currentFrame.mapPoints    = vpMPsMM;
                        currentFrame.outlierFlags = vbOutMM;

                        if (visualOdometry)
                        {
                            for (int i = 0; i < currentFrame.N; i++)
                            {
                                if (currentFrame.mapPoints[i] &&
                                    !currentFrame.outlierFlags[i])
                                {
                                    currentFrame.mapPoints[i]->increaseFound();
                                }
                            }
                        }
                    }
                    else if (bOKReloc)
                    {
                        visualOdometry = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if (!currentFrame.p_referenceKeyFrame)
            currentFrame.p_referenceKeyFrame = p_referenceKF;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndPosePred =
            std::chrono::steady_clock::now();

        double timePosePred = std::chrono::duration_cast<
                                  std::chrono::duration<double, std::milli>>(
                                  time_EndPosePred - time_StartPosePred)
                                  .count();
        vdPosePred_ms.push_back(timePosePred);
#endif

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartLMTrack =
            std::chrono::steady_clock::now();
#endif
        // If we have an initial estimation of the camera pose and matching.
        // Track the local map.
        if (!onlyTracking)
        {
            if (bOK)
                bOK = trackLocalMap();
            else
                std::cout << "[Tracking] Failed to track the features ..."
                          << std::endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the
            // map. We cannot retrieve a local map and therefore we do not
            // perform TrackLocalMap(). Once the system relocalizes the camera
            // we will use the local map again.
            if (bOK && !visualOdometry)
                bOK = trackLocalMap();
        }

        if (bOK)
            state = OK;
        else if (state == OK)
        {
            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            {
                Verbose::printMess("Visual tracking lost; entering bounded "
                                   "inertial recovery...",
                                   Verbose::VERBOSITY_NORMAL);
                /* Do not destroy a newly initialized inertial map after one
                 * failed visual update. RECENTLY_LOST already propagates the
                 * state with the IMU for a bounded recovery window and the
                 * LOST branch performs the appropriate reset or atlas-map
                 * transition if recovery actually fails. */
                state = RECENTLY_LOST;
            }
            else
                state = RECENTLY_LOST; // visual to lost

            /*if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {*/
            timeStampLost = currentFrame.timeStamp;
            //}
        }

        if (pCurrentMap->isImuInitialized())
        {
            if (bOK)
            {
                if (currentFrame.mnId == (lastRelocFrameId + framesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    resetFrameIMU();
                }
                else if (currentFrame.mnId > (lastRelocFrameId + 30))
                    lastBias = currentFrame.imuBias;
            }
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndLMTrack =
            std::chrono::steady_clock::now();

        double timeLMTrack = std::chrono::duration_cast<
                                 std::chrono::duration<double, std::milli>>(
                                 time_EndLMTrack - time_StartLMTrack)
                                 .count();
        vdLMTrack_ms.push_back(timeLMTrack);
#endif

        // Update drawer
        p_frameDrawer->update(this);
        if (currentFrame.isSet())
            p_mapDrawer->setCurrentCameraPose(currentFrame.getPose());

        if (bOK || state == RECENTLY_LOST)
        {
            // Update motion model
            if (lastFrame.isSet() && currentFrame.isSet())
            {
                Sophus::SE3f LastTwc = lastFrame.getPose().inverse();
                velocity             = currentFrame.getPose() * LastTwc;
                velocityAvailable    = true;
            }
            else
            {
                velocityAvailable = false;
            }

            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
                p_mapDrawer->setCurrentCameraPose(currentFrame.getPose());

            // Clean VO matches
            for (int i = 0; i < currentFrame.N; i++)
            {
                MapPoint *pMP = currentFrame.mapPoints[i];
                if (pMP)
                    if (pMP->getObservationCount() < 1)
                    {
                        currentFrame.outlierFlags[i] = false;
                        currentFrame.mapPoints[i] =
                            static_cast<MapPoint *>(nullptr);
                    }
            }

            // Delete temporal MapPoints
            for (list<MapPoint *>::iterator lit  = mlpTemporalPoints.begin(),
                                            lend = mlpTemporalPoints.end();
                 lit != lend;
                 lit++)
            {
                MapPoint *pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartNewKF =
                std::chrono::steady_clock::now();
#endif
            bool bNeedKF = needNewKeyFrame();

            // Check if we need to insert a new keyframe
            if (bNeedKF && (bOK || (insertKFsLost && state == RECENTLY_LOST &&
                                    (sensor == System::IMU_MONOCULAR ||
                                     sensor == System::IMU_STEREO ||
                                     sensor == System::IMU_RGBD))))
            {
                // Create a new KeyFrame
                createNewKeyFrame();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndNewKF =
                std::chrono::steady_clock::now();

            double timeNewKF = std::chrono::duration_cast<
                                   std::chrono::duration<double, std::milli>>(
                                   time_EndNewKF - time_StartNewKF)
                                   .count();
            vdNewKF_ms.push_back(timeNewKF);
#endif

            // We allow points with high innovation (considererd outliers by the
            // Huber Function) pass to the new keyframe, so that bundle
            // adjustment will finally decide if they are outliers or not. We
            // don't want next frame to estimate its position with those points
            // so we discard them in the frame. Only has effect if lastframe is
            // tracked
            for (int i = 0; i < currentFrame.N; i++)
            {
                if (currentFrame.mapPoints[i] && currentFrame.outlierFlags[i])
                    currentFrame.mapPoints[i] =
                        static_cast<MapPoint *>(nullptr);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (state == LOST)
        {
            if (pCurrentMap->getKeyFrameCount() <= 10)
            {
                p_system->requestResetActiveMapWithCause(
                    ResetCause::VISUAL_TRACKING_LOST_SMALL_MAP);
                return;
            }
            if (sensor == System::IMU_MONOCULAR ||
                sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::printMess(
                        "Track lost before IMU initialisation, reseting...",
                        Verbose::VERBOSITY_QUIET);
                    p_system->requestResetActiveMapWithCause(
                        ResetCause::
                            VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION);
                    return;
                }

            reportResetAttribution(ResetCause::VISUAL_TRACKING_LOST_NEW_MAP,
                                   ResetAction::CREATE_MAP_EXECUTION);
            createMapInAtlas();

            return;
        }

        if (!currentFrame.p_referenceKeyFrame)
            currentFrame.p_referenceKeyFrame = p_referenceKF;

        lastFrame = Frame(currentFrame);
    }

    if (state == OK || state == RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera
        // trajectory afterwards.
        if (currentFrame.isSet())
        {
            Sophus::SE3f Tcr_ =
                currentFrame.getPose() *
                currentFrame.p_referenceKeyFrame->getPoseInverse();
            relativeFramePoses.push_back(Tcr_);
            mlpReferences.push_back(currentFrame.p_referenceKeyFrame);
            frameTimes.push_back(currentFrame.timeStamp);
            mlbLost.push_back(state == LOST);
        }
        else
        {
            // The current frame carries no pose (e.g. tracking was lost):
            // append the last stored entry to keep the trajectory aligned,
            // when one exists.
            if (!relativeFramePoses.empty() && !mlpReferences.empty() &&
                !frameTimes.empty())
            {
                relativeFramePoses.push_back(relativeFramePoses.back());
                mlpReferences.push_back(mlpReferences.back());
                frameTimes.push_back(frameTimes.back());
                mlbLost.push_back(state == LOST);
            }
        }
    }

#ifdef REGISTER_LOOP
    if (stop())
    {

        // Safe area to stop
        while (isStopped())
        {
            usleep(3000);
        }
    }
#endif
}

// Map initialization for Stereo and RGB-D (with/without IMU) setups
void Tracking::stereoInitialization()
{
    // Require more points for robust initialization in corridors
    if (currentFrame.N > initializationMinPoints)
    {
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
        {
            if (!currentFrame.p_imuPreintegrated ||
                !lastFrame.p_imuPreintegrated)
            {
                std::cout << "[Tracking] IMU measurements are not available "
                             "for the current frame!"
                          << std::endl;
                return;
            }

            // Check acceleration difference for fast initialization
            if (!fastInit)
            {
                const double accelDiff =
                    (currentFrame.p_imuPreintegratedFrame->avgA -
                     lastFrame.p_imuPreintegratedFrame->avgA)
                        .norm();

                if (accelDiff < imuThresh)
                {
                    std::cout << "[Tracking] Low IMU acceleration changes: "
                              << std::fixed << std::setprecision(2) << accelDiff
                              << " (threshold: " << imuThresh
                              << ")! Skipping ..." << std::endl;
                    return;
                }
            }

            if (p_imuPreintegratedFromLastKF)
                delete p_imuPreintegratedFromLastKF;

            // Reset IMU preintegration from last keyframe
            p_imuPreintegratedFromLastKF =
                new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
            currentFrame.p_imuPreintegrated = p_imuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
        {
            Eigen::Matrix3f Rwb0 =
                currentFrame.imuCalibration.mTcb.rotationMatrix();
            Eigen::Vector3f twb0 =
                currentFrame.imuCalibration.mTcb.translation();
            Eigen::Vector3f Vwb0;
            Vwb0.setZero();
            currentFrame.setImuPoseVelocity(Rwb0, twb0, Vwb0);
        }
        else
            currentFrame.setPose(Sophus::SE3f());

        // Create KeyFrame
        vs_graphs::core::KeyFrame *pKFini =
            new vs_graphs::core::KeyFrame(currentFrame,
                                          p_atlas->getCurrentMap(),
                                          p_keyFrameDatabase);

        // Insert KeyFrame in the map
        p_atlas->addKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        int nPointsCreated = 0;
        if (!p_camera2)
        {
            for (int i = 0; i < currentFrame.N; i++)
            {
                float z = currentFrame.depths[i];
                if (z > 0)
                {
                    Eigen::Vector3f x3D;
                    currentFrame.unprojectStereo(i, x3D);
                    MapPoint *pNewMP =
                        new MapPoint(x3D, pKFini, p_atlas->getCurrentMap());
                    pNewMP->addObservation(pKFini, i);
                    pKFini->addMapPoint(pNewMP, i);
                    pNewMP->computeDistinctiveDescriptors();
                    pNewMP->updateNormalAndDepth();
                    p_atlas->addMapPoint(pNewMP);

                    currentFrame.mapPoints[i] = pNewMP;
                    nPointsCreated++;
                }
            }
        }
        else
        {
            for (int i = 0; i < currentFrame.Nleft; i++)
            {
                int rightIndex = currentFrame.leftToRightMatches[i];
                if (rightIndex != -1)
                {
                    Eigen::Vector3f x3D = currentFrame.stereoPoints3D[i];

                    MapPoint *pNewMP =
                        new MapPoint(x3D, pKFini, p_atlas->getCurrentMap());

                    pNewMP->addObservation(pKFini, i);
                    pNewMP->addObservation(pKFini,
                                           rightIndex + currentFrame.Nleft);

                    pKFini->addMapPoint(pNewMP, i);
                    pKFini->addMapPoint(pNewMP,
                                        rightIndex + currentFrame.Nleft);

                    pNewMP->computeDistinctiveDescriptors();
                    pNewMP->updateNormalAndDepth();
                    p_atlas->addMapPoint(pNewMP);

                    currentFrame.mapPoints[i] = pNewMP;
                    currentFrame.mapPoints[rightIndex + currentFrame.Nleft] =
                        pNewMP;
                    nPointsCreated++;
                }
            }
        }

        std::cout << "\n[Tracking] New map created with #" +
                         to_string(p_atlas->getMapPointCount()) + " points!"
                  << std::endl;

        // Require minimum points for successful initialization
        if (nPointsCreated < initializationMinPoints)
        {
            std::cout << "[Tracking] Insufficient points for initialization ("
                      << nPointsCreated << " < " << initializationMinPoints
                      << "), resetting..." << std::endl;
            p_system->requestResetActiveMapWithCause(
                ResetCause::INITIALIZATION_INSUFFICIENT_POINTS);
            return;
        }

        p_localMapper->insertKeyFrame(pKFini);

        lastFrame      = Frame(currentFrame);
        lastKeyFrameId = currentFrame.mnId;
        p_lastKeyFrame = pKFini;

        localKeyFrames.push_back(pKFini);
        localMapPoints                   = p_atlas->getAllMapPoints();
        p_referenceKF                    = pKFini;
        currentFrame.p_referenceKeyFrame = pKFini;

        p_atlas->setReferenceMapPoints(localMapPoints);

        p_atlas->getCurrentMap()->keyFrameOrigins.push_back(pKFini);

        p_mapDrawer->setCurrentCameraPose(currentFrame.getPose());

        state = OK;
    }
}

void Tracking::monocularInitialization()
{
    if (!readyToInitialize)
    {
        // Set Reference Frame
        if (currentFrame.keyPoints.size() > 100)
        {
            initialFrame = Frame(currentFrame);
            lastFrame    = Frame(currentFrame);
            mvbPrevMatched.resize(currentFrame.keyPointsUndistorted.size());
            for (size_t i = 0; i < currentFrame.keyPointsUndistorted.size();
                 i++)
                mvbPrevMatched[i] = currentFrame.keyPointsUndistorted[i].pt;

            fill(iniMatches.begin(), iniMatches.end(), -1);

            if (sensor == System::IMU_MONOCULAR)
            {
                if (p_imuPreintegratedFromLastKF)
                {
                    delete p_imuPreintegratedFromLastKF;
                }
                p_imuPreintegratedFromLastKF =
                    new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
                currentFrame.p_imuPreintegrated = p_imuPreintegratedFromLastKF;
            }

            readyToInitialize = true;
            return;
        }
    }
    else
    {
        if (((int)currentFrame.keyPoints.size() <= 100) ||
            ((sensor == System::IMU_MONOCULAR) &&
             (lastFrame.timeStamp - initialFrame.timeStamp > 1.0)))
        {
            readyToInitialize = false;
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        int        nmatches = matcher.searchForInitialization(initialFrame,
                                                       currentFrame,
                                                       mvbPrevMatched,
                                                       iniMatches,
                                                       100);

        // Check if there are enough correspondences
        if (nmatches < 100)
        {
            readyToInitialize = false;
            return;
        }

        Sophus::SE3f Tcw;
        vector<bool>
            vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if (p_camera->reconstructWithTwoViews(initialFrame.keyPointsUndistorted,
                                              currentFrame.keyPointsUndistorted,
                                              iniMatches,
                                              Tcw,
                                              iniP3D,
                                              vbTriangulated))
        {
            for (size_t i = 0, iend = iniMatches.size(); i < iend; i++)
            {
                if (iniMatches[i] >= 0 && !vbTriangulated[i])
                {
                    iniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            // mInitialFrame.setPose(Sophus::SE3f());
            initialFrame.setPose(poseTc0w);
            currentFrame.setPose(Tcw * poseTc0w);

            createInitialMapMonocular();
        }
    }
}

void Tracking::createInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame *pKFini = new KeyFrame(initialFrame,
                                    p_atlas->getCurrentMap(),
                                    p_keyFrameDatabase);
    KeyFrame *pKFcur = new KeyFrame(currentFrame,
                                    p_atlas->getCurrentMap(),
                                    p_keyFrameDatabase);

    if (sensor == System::IMU_MONOCULAR)
        pKFini->p_imuPreintegrated = (IMU::Preintegrated *)(nullptr);

    pKFini->computeBagOfWords();
    pKFcur->computeBagOfWords();

    // Insert KFs in the map
    p_atlas->addKeyFrame(pKFini);
    p_atlas->addKeyFrame(pKFcur);

    for (size_t i = 0; i < iniMatches.size(); i++)
    {
        if (iniMatches[i] < 0)
            continue;

        // Create MapPoint.
        Eigen::Vector3f worldPos;
        worldPos << iniP3D[i].x, iniP3D[i].y, iniP3D[i].z;
        Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), worldPos);
        Sophus::SE3f Twmp = poseTc0w.inverse() * Tc0mp;
        worldPos          = Twmp.translation();
        MapPoint *pMP =
            new MapPoint(worldPos, pKFcur, p_atlas->getCurrentMap());

        pKFini->addMapPoint(pMP, i);
        pKFcur->addMapPoint(pMP, iniMatches[i]);

        pMP->addObservation(pKFini, i);
        pMP->addObservation(pKFcur, iniMatches[i]);

        pMP->computeDistinctiveDescriptors();
        pMP->updateNormalAndDepth();

        // Fill Current Frame structure
        currentFrame.mapPoints[iniMatches[i]]    = pMP;
        currentFrame.outlierFlags[iniMatches[i]] = false;

        // Add to Map
        p_atlas->addMapPoint(pMP);
    }

    // Update Connections
    pKFini->updateConnections();
    pKFcur->updateConnections();

    std::set<MapPoint *> sMPs;
    sMPs = pKFini->getMapPoints();

    // Bundle Adjustment
    std::cout << "\n[Tracking]" << std::endl;
    std::cout << "- New map created with #"
              << to_string(p_atlas->getMapPointCount()) << " points!"
              << std::endl;
    Optimizer::globalBundleAdjustment(
        p_atlas->getCurrentMap(),
        20,
        nullptr,
        0,
        true,
        types::SystemParams::getParams()->markers.impact);

    float medianDepth = pKFini->computeSceneMedianDepth(2);
    float invMedianDepth;
    if (sensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f / medianDepth;
    else
        invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 ||
        pKFcur->getTrackedMapPointCount(1) < 50) // TODO Check, originally 100
                                                 // tracks
    {
        Verbose::printMess("Wrong initialization, reseting...",
                           Verbose::VERBOSITY_QUIET);
        p_system->requestResetActiveMapWithCause(
            ResetCause::INITIALIZATION_INVALID_MONOCULAR_MAP);
        return;
    }

    // Scale initial baseline
    Sophus::SE3f Tc2w = pKFcur->getPose();
    Tc2w.translation() *= invMedianDepth;
    pKFcur->setPose(Tc2w);

    // Scale points
    vector<MapPoint *> vpAllMapPoints = pKFini->getMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
    {
        if (vpAllMapPoints[iMP])
        {
            MapPoint *pMP = vpAllMapPoints[iMP];
            pMP->setWorldPos(pMP->getWorldPos() * invMedianDepth);
            pMP->updateNormalAndDepth();
        }
    }

    if (sensor == System::IMU_MONOCULAR)
    {
        pKFcur->p_prevKF           = pKFini;
        pKFini->p_nextKF           = pKFcur;
        pKFcur->p_imuPreintegrated = p_imuPreintegratedFromLastKF;

        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(pKFcur->p_imuPreintegrated->getUpdatedBias(),
                                   pKFcur->imuCalibration);
    }

    p_localMapper->insertKeyFrame(pKFini);
    p_localMapper->insertKeyFrame(pKFcur);
    p_localMapper->firstTimestamp = pKFcur->timeStamp;

    currentFrame.setPose(pKFcur->getPose());
    lastKeyFrameId = currentFrame.mnId;
    p_lastKeyFrame = pKFcur;
    // mnLastRelocFrameId = mInitialFrame.mnId;

    localKeyFrames.push_back(pKFcur);
    localKeyFrames.push_back(pKFini);
    localMapPoints                   = p_atlas->getAllMapPoints();
    p_referenceKF                    = pKFcur;
    currentFrame.p_referenceKeyFrame = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame *> vKFs = p_atlas->getAllKeyFrames();

    Sophus::SE3f deltaT =
        vKFs.back()->getPose() * vKFs.front()->getPoseInverse();
    velocityAvailable   = false;
    Eigen::Vector3f phi = deltaT.so3().log();

    double aux = (currentFrame.timeStamp - lastFrame.timeStamp) /
                 (currentFrame.timeStamp - initialFrame.timeStamp);
    phi *= aux;

    lastFrame = Frame(currentFrame);

    p_atlas->setReferenceMapPoints(localMapPoints);

    p_mapDrawer->setCurrentCameraPose(pKFcur->getPose());

    p_atlas->getCurrentMap()->keyFrameOrigins.push_back(pKFini);

    state = OK;

    initId = pKFcur->mnId;
}

void Tracking::createMapInAtlas()
{
    lastInitFrameId = currentFrame.mnId;
    p_atlas->createNewMap();
    if (sensor == System::IMU_STEREO || sensor == System::IMU_MONOCULAR ||
        sensor == System::IMU_RGBD)
        p_atlas->setInertialSensor();
    isInitSet = false;

    initialFrameId = currentFrame.mnId + 1;
    state          = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    velocityAvailable = false;
    // mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is
    // the current id, because it is the new starting point for new map
    Verbose::printMess("First frame id in map: " +
                           to_string(lastInitFrameId + 1),
                       Verbose::VERBOSITY_NORMAL);
    visualOdometry = false; // Init value for know if there are enough MapPoints
                            // in the last KF
    if (sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR)
    {
        readyToInitialize = false;
    }

    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        p_imuPreintegratedFromLastKF)
    {
        delete p_imuPreintegratedFromLastKF;
        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(IMU::Bias(), *p_imuCalibration);
    }

    if (p_lastKeyFrame)
        p_lastKeyFrame = static_cast<KeyFrame *>(nullptr);

    if (p_referenceKF)
        p_referenceKF = static_cast<KeyFrame *>(nullptr);

    lastFrame    = Frame();
    currentFrame = Frame();
    iniMatches.clear();

    createdMap = true;
}

void Tracking::checkReplacedInLastFrame()
{
    for (int i = 0; i < lastFrame.N; i++)
    {
        MapPoint *pMP = lastFrame.mapPoints[i];

        if (pMP)
        {
            MapPoint *pRep = pMP->getReplaced();
            if (pRep)
            {
                lastFrame.mapPoints[i] = pRep;
            }
        }
    }
}

bool Tracking::trackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    currentFrame.computeBagOfWords();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher         matcher(0.7, true);
    vector<MapPoint *> vpMapPointMatches;

    int nmatches =
        matcher.searchByBoW(p_referenceKF, currentFrame, vpMapPointMatches);

    if (nmatches < 8)
    {
        std::cout << "[Tracking] Warning: Less than 8 features matched!"
                  << std::endl;
        return false;
    }

    currentFrame.mapPoints = vpMapPointMatches;
    currentFrame.setPose(lastFrame.getPose());

    // mCurrentFrame.printPointDistribution();

    Optimizer::poseOptimization(&currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < currentFrame.N; i++)
    {
        // if(i >= mCurrentFrame.Nleft) break;
        if (currentFrame.mapPoints[i])
        {
            if (currentFrame.outlierFlags[i])
            {
                MapPoint *pMP = currentFrame.mapPoints[i];

                currentFrame.mapPoints[i]    = static_cast<MapPoint *>(nullptr);
                currentFrame.outlierFlags[i] = false;
                if (i < currentFrame.Nleft)
                {
                    pMP->trackInView = false;
                }
                else
                {
                    pMP->trackInViewR = false;
                }
                pMP->trackInView     = false;
                pMP->lastSeenFrameId = currentFrame.mnId;
                nmatches--;
            }
            else if (currentFrame.mapPoints[i]->getObservationCount() > 0)
                nmatchesMap++;
        }
    }

    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap >= 10;
}

void Tracking::updateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame    *pRef = lastFrame.p_referenceKeyFrame;
    Sophus::SE3f Tlr =
        relativeFramePoses.empty() ? Sophus::SE3f() : relativeFramePoses.back();
    lastFrame.setPose(Tlr * pRef->getPose());

    if (lastKeyFrameId == lastFrame.mnId || sensor == System::MONOCULAR ||
        sensor == System::IMU_MONOCULAR || !onlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D
    // sensor
    vector<pair<float, int>> vDepthIdx;
    const int Nfeat = lastFrame.Nleft == -1 ? lastFrame.N : lastFrame.Nleft;
    vDepthIdx.reserve(Nfeat);
    for (int i = 0; i < Nfeat; i++)
    {
        float z = lastFrame.depths[i];
        if (z > 0)
        {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = lastFrame.mapPoints[i];

        if (!pMP)
            bCreateNew = true;
        else if (pMP->getObservationCount() < 1)
            bCreateNew = true;

        if (bCreateNew)
        {
            Eigen::Vector3f x3D;

            if (lastFrame.Nleft == -1)
            {
                lastFrame.unprojectStereo(i, x3D);
            }
            else
            {
                x3D = lastFrame.unprojectStereoFishEye(i);
            }

            MapPoint *pNewMP =
                new MapPoint(x3D, p_atlas->getCurrentMap(), &lastFrame, i);
            lastFrame.mapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if (vDepthIdx[j].first > depthThreshold && nPoints > 100)
            break;
    }
}

bool Tracking::trackWithMotionModel()
{
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    updateLastFrame();

    if (p_atlas->isImuInitialized() &&
        (currentFrame.mnId > lastRelocFrameId + framesToResetIMU))
    {
        // Predict state with IMU if it is initialized and it doesnt need reset
        predictStateIMU();
        return true;
    }
    else
    {
        currentFrame.setPose(velocity * lastFrame.getPose());
    }

    fill(currentFrame.mapPoints.begin(),
         currentFrame.mapPoints.end(),
         static_cast<MapPoint *>(nullptr));

    // Project points seen in previous frame
    int th;

    if (sensor == System::STEREO)
        th = 7;
    else
        th = 15;

    int nmatches = matcher.searchByProjection(
        currentFrame,
        lastFrame,
        th,
        sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR);

    // If few matches, use progressively wider window searches.
    int searchStep   = 1;
    int searchRadius = th;
    while (nmatches < 20 && searchRadius < motionModelMaxSearchRadius)
    {
        int expandedTh = static_cast<int>(std::ceil(
            th * (1.0F +
                  searchStep * (mfMotionModelSearchRadiusMultiplier - 1.0F))));
        expandedTh     = std::min(expandedTh, motionModelMaxSearchRadius);
        if (expandedTh <= searchRadius)
        {
            break;
        }

        Verbose::printMess("Not enough matches, wider window search (radius " +
                               std::to_string(expandedTh) + ")!!",
                           Verbose::VERBOSITY_NORMAL);
        fill(currentFrame.mapPoints.begin(),
             currentFrame.mapPoints.end(),
             static_cast<MapPoint *>(nullptr));

        nmatches = matcher.searchByProjection(
            currentFrame,
            lastFrame,
            expandedTh,
            sensor == System::MONOCULAR || sensor == System::IMU_MONOCULAR);
        Verbose::printMess("Matches with wider search: " + to_string(nmatches),
                           Verbose::VERBOSITY_NORMAL);
        searchRadius = expandedTh;
        searchStep++;
    }

    if (nmatches < 20)
    {
        Verbose::printMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
            sensor == System::IMU_RGBD)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    Optimizer::poseOptimization(&currentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < currentFrame.N; i++)
    {
        if (currentFrame.mapPoints[i])
        {
            if (currentFrame.outlierFlags[i])
            {
                MapPoint *pMP = currentFrame.mapPoints[i];

                currentFrame.mapPoints[i]    = static_cast<MapPoint *>(nullptr);
                currentFrame.outlierFlags[i] = false;
                if (i < currentFrame.Nleft)
                {
                    pMP->trackInView = false;
                }
                else
                {
                    pMP->trackInViewR = false;
                }
                pMP->lastSeenFrameId = currentFrame.mnId;
                nmatches--;
            }
            else if (currentFrame.mapPoints[i]->getObservationCount() > 0)
                nmatchesMap++;
        }
    }

    if (onlyTracking)
    {
        visualOdometry = nmatchesMap < 10;
        return nmatches > 20;
    }

    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
        return true;
    else
        return nmatchesMap >= 10;
}

bool Tracking::trackLocalMap()
{

    // We have an estimation of the camera pose and some map points tracked in
    // the frame. We retrieve the local map and try to find matches to points in
    // the local map.
    trackedFr++;

    updateLocalMap();
    searchLocalPoints();

    // TOO check outliers before PO
    int aux1 = 0, aux2 = 0;
    for (int i = 0; i < currentFrame.N; i++)
        if (currentFrame.mapPoints[i])
        {
            aux1++;
            if (currentFrame.outlierFlags[i])
                aux2++;
        }

    if (!p_atlas->isImuInitialized())
        Optimizer::poseOptimization(&currentFrame);
    else
    {
        if (currentFrame.mnId <= lastRelocFrameId + framesToResetIMU)
        {
            Verbose::printMess("TLM: PoseOptimization ",
                               Verbose::VERBOSITY_DEBUG);
            Optimizer::poseOptimization(&currentFrame);
        }
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if (!mapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::printMess("TLM: PoseInertialOptimizationLastFrame ",
                                   Verbose::VERBOSITY_DEBUG);
                Optimizer::poseInertialOptimizationLastFrame(
                    &currentFrame); // ,
                                    // !mpLastKeyFrame->getMap()->getInertialBA1());
            }
            else
            {
                Verbose::printMess("TLM: PoseInertialOptimizationLastKeyFrame ",
                                   Verbose::VERBOSITY_DEBUG);
                Optimizer::poseInertialOptimizationLastKeyFrame(
                    &currentFrame); // ,
                                    // !mpLastKeyFrame->getMap()->getInertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for (int i = 0; i < currentFrame.N; i++)
        if (currentFrame.mapPoints[i])
        {
            aux1++;
            if (currentFrame.outlierFlags[i])
                aux2++;
        }

    matchesInliers = 0;

    // Update MapPoints Statistics
    int nCloseInliers = 0;
    int nFarInliers   = 0;
    for (int i = 0; i < currentFrame.N; i++)
    {
        if (currentFrame.mapPoints[i])
        {
            if (!currentFrame.outlierFlags[i])
            {
                currentFrame.mapPoints[i]->increaseFound();
                if (!onlyTracking)
                {
                    if (currentFrame.mapPoints[i]->getObservationCount() > 0)
                        matchesInliers++;
                }
                else
                    matchesInliers++;

                // Track close vs far inliers for adaptive acceptance
                if ((sensor == System::RGBD || sensor == System::IMU_RGBD ||
                     sensor == System::STEREO ||
                     sensor == System::IMU_STEREO) &&
                    i < (int)currentFrame.depths.size() &&
                    currentFrame.depths[i] > 0)
                {
                    if (currentFrame.depths[i] < depthThreshold)
                        nCloseInliers++;
                    else
                        nFarInliers++;
                }
            }
            else if (sensor == System::STEREO)
                currentFrame.mapPoints[i] = static_cast<MapPoint *>(nullptr);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    p_localMapper->matchesInliers = matchesInliers;
    if (currentFrame.mnId < lastRelocFrameId + maxFrames && matchesInliers < 25)
        return false;

    if ((matchesInliers > 10) && (state == RECENTLY_LOST))
        return true;

    // AGGRESSIVE TRACKING ACCEPTANCE for corridors: Require only 5 close
    // inliers In featureless corridors, close points (walls/floor) are more
    // reliable than far points
    if (sensor == System::IMU_MONOCULAR)
    {
        // For IMU monocular, rely on IMU + minimum visual inliers - very
        // permissive LOWERED: 8->5 with IMU, 25->15 without IMU
        if ((matchesInliers < 5 && p_atlas->isImuInitialized()) ||
            (matchesInliers < 15 && !p_atlas->isImuInitialized()))
        {
            return false;
        }
        else
            return true;
    }
    else if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
    {
        // For IMU stereo/RGBD: require only 5 close inliers (AGGRESSIVE)
        // In corridors, close points (walls) provide strong geometric
        // constraints
        if (nCloseInliers >= 5 && matchesInliers >= 5)
            return true;
        else if (matchesInliers >= 10) // fallback with more total inliers
            return true;
        else
            return false;
    }
    else if (sensor == System::RGBD || sensor == System::STEREO)
    {
        // For visual-only stereo/RGBD: require only 5 close inliers
        // (AGGRESSIVE) Close points are more reliable in corridors (wall/floor
        // planes)
        if (nCloseInliers >= 5)
            return true;
        else if (nCloseInliers >= 3 && matchesInliers >= 10)
            return true;
        else if (matchesInliers >= 15)
            return true;
        else
            return false;
    }
    else
    {
        // Monocular: lowered threshold
        if (matchesInliers < 10)
            return false;
        else
            return true;
    }
}

bool Tracking::needNewKeyFrame()
{
    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        !p_atlas->getCurrentMap()->isImuInitialized())
    {
        if (sensor == System::IMU_MONOCULAR &&
            (currentFrame.timeStamp - p_lastKeyFrame->timeStamp) >= 0.25)
            return true;
        else if ((sensor == System::IMU_STEREO || sensor == System::IMU_RGBD) &&
                 (currentFrame.timeStamp - p_lastKeyFrame->timeStamp) >= 0.25)
            return true;
        else
            return false;
    }

    if (onlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (p_localMapper->isStopped() || p_localMapper->stopRequested())
    {
        /*if(mSensor == System::MONOCULAR)
        {
            std::cout << "NeedNewKeyFrame: localmap stopped" << std::endl;
        }*/
        return false;
    }

    const int nKFs = p_atlas->getKeyFrameCount();

    // Do not insert keyframes if not enough frames have passed from last
    // relocalisation
    if (currentFrame.mnId < lastRelocFrameId + maxFrames && nKFs > maxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = p_referenceKF->getTrackedMapPointCount(nMinObs);

    // Check how many "close" points are being tracked and how many could be
    // potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose    = 0;

    if (sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR)
    {
        int N =
            (currentFrame.Nleft == -1) ? currentFrame.N : currentFrame.Nleft;
        for (int i = 0; i < N; i++)
        {
            if (currentFrame.depths[i] > 0 &&
                currentFrame.depths[i] < depthThreshold)
            {
                if (currentFrame.mapPoints[i] && !currentFrame.outlierFlags[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // AGGRESSIVE CORRIDOR TRACKING: Stricter KF insertion criteria
    // Require minimum 30 total inliers, 15 close inliers, 1.0s temporal spacing
    const bool bEnoughTotalInliers = (matchesInliers >= minInliersForKF);
    const bool bEnoughCloseInliers = (nTrackedClose >= minCloseInliersForKF);
    const bool bEnoughTimeSinceLastKF =
        p_lastKeyFrame && (currentFrame.timeStamp - p_lastKeyFrame->timeStamp >=
                           mdMinTemporalSpacingKF);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    if (sensor == System::MONOCULAR)
        thRefRatio = 0.9f;

    if (p_camera2)
        thRefRatio = 0.75f;

    if (sensor == System::IMU_MONOCULAR)
    {
        if (matchesInliers > 350)
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = p_localMapper->isAcceptingKeyFrames();

    // Condition 1a: More than "MaxFrames" have passed from last keyframe
    // insertion
    const bool c1a = currentFrame.mnId >= lastKeyFrameId + maxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b =
        ((currentFrame.mnId >= lastKeyFrameId + minFrames) &&
         bLocalMappingIdle && p_localMapper->keyframesInQueue() < 5);
    // Condition 1c: tracking is weak
    const bool c1c =
        sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR &&
        sensor != System::IMU_STEREO && sensor != System::IMU_RGBD &&
        (matchesInliers < nRefMatches * 0.25 || bNeedToInsertClose) &&
        matchesInliers > 20;
    // Condition 2: Few tracked points compared to reference keyframe.
    const bool c2 =
        (((matchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose)) &&
         matchesInliers > minInliersForKF);

    // AGGRESSIVE: Additional corridor-specific conditions
    // Condition 3: Temporal spacing (1.0s minimum)
    bool c3 = false;
    if (p_lastKeyFrame)
    {
        if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
             sensor == System::IMU_RGBD) &&
            (currentFrame.timeStamp - p_lastKeyFrame->timeStamp) >=
                mdMinTemporalSpacingKF)
        {
            c3 = true;
        }
    }

    // Condition 4: Enough inliers but temporal spacing met
    bool c4 = false;
    if (bEnoughTotalInliers && bEnoughCloseInliers && bEnoughTimeSinceLastKF)
    {
        c4 = true;
    }
    // Also insert if tracking is weak (RECENTLY_LOST) and we have minimum
    // inliers
    else if (state == RECENTLY_LOST && matchesInliers > minInliersForKF)
    {
        c4 = true;
    }

    if (((c1a || c1b || c1c) && c2) || c3 || c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle || p_localMapper->isInitializing())
        {
            return true;
        }
        else
        {
            p_localMapper->interruptBA();
            if (sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR)
            {
                if (p_localMapper->keyframesInQueue() < 8)
                    return true;
                else
                    return false;
            }
            else
            {
                return false;
            }
        }
    }
    else
        return false;
}

void Tracking::createNewKeyFrame()
{
    if (p_localMapper->isInitializing() && !p_atlas->isImuInitialized())
        return;

    if (!p_localMapper->setNotStop(true))
        return;

    KeyFrame *pKF = new KeyFrame(currentFrame,
                                 p_atlas->getCurrentMap(),
                                 p_keyFrameDatabase);

    if (p_atlas->isImuInitialized()) //  || mpLocalMapper->IsInitializing())
        pKF->isImu = true;

    pKF->setNewBias(currentFrame.imuBias);
    p_referenceKF                    = pKF;
    currentFrame.p_referenceKeyFrame = pKF;

    if (p_lastKeyFrame)
    {
        pKF->p_prevKF            = p_lastKeyFrame;
        p_lastKeyFrame->p_nextKF = pKF;
    }
    else
        Verbose::printMess("No last KF in KF creation!!",
                           Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
        sensor == System::IMU_RGBD)
    {
        p_imuPreintegratedFromLastKF =
            new IMU::Preintegrated(pKF->getImuBias(), pKF->imuCalibration);
    }

    if (sensor != System::MONOCULAR && sensor != System::IMU_MONOCULAR)
    {
        currentFrame.updatePoseMatrices();
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        // Both sensor branches intentionally use the same cap of 100.
        int maxPoint = 100;
        if (sensor == System::IMU_STEREO || sensor == System::IMU_RGBD)
            maxPoint = 100;

        vector<pair<float, int>> vDepthIdx;
        int                      N =
            (currentFrame.Nleft != -1) ? currentFrame.Nleft : currentFrame.N;
        vDepthIdx.reserve(currentFrame.N);
        for (int i = 0; i < N; i++)
        {
            float z = currentFrame.depths[i];
            if (z > 0)
                vDepthIdx.push_back(make_pair(z, i));
        }

        if (!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++)
            {
                bool bCreateNew = false;
                int  i          = vDepthIdx[j].second;

                MapPoint *pMP = currentFrame.mapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->getObservationCount() < 1)
                {
                    bCreateNew = true;
                    currentFrame.mapPoints[i] =
                        static_cast<MapPoint *>(nullptr);
                }

                if (bCreateNew)
                {
                    Eigen::Vector3f x3D;

                    if (currentFrame.Nleft == -1)
                        currentFrame.unprojectStereo(i, x3D);
                    else
                        x3D = currentFrame.unprojectStereoFishEye(i);

                    MapPoint *pNewMP =
                        new MapPoint(x3D, pKF, p_atlas->getCurrentMap());
                    pNewMP->addObservation(pKF, i);

                    // Check if it is a stereo observation in order to not
                    // duplicate mappoints
                    if (currentFrame.Nleft != -1 &&
                        currentFrame.leftToRightMatches[i] >= 0)
                    {
                        currentFrame
                            .mapPoints[currentFrame.Nleft +
                                       currentFrame.leftToRightMatches[i]] =
                            pNewMP;
                        pNewMP->addObservation(
                            pKF,
                            currentFrame.Nleft +
                                currentFrame.leftToRightMatches[i]);
                        pKF->addMapPoint(
                            pNewMP,
                            currentFrame.Nleft +
                                currentFrame.leftToRightMatches[i]);
                    }

                    pKF->addMapPoint(pNewMP, i);
                    pNewMP->computeDistinctiveDescriptors();
                    pNewMP->updateNormalAndDepth();
                    p_atlas->addMapPoint(pNewMP);

                    currentFrame.mapPoints[i] = pNewMP;
                    nPoints++;
                }
                else
                    nPoints++;

                if (vDepthIdx[j].first > depthThreshold && nPoints > maxPoint)
                {
                    break;
                }
            }
        }
    }

    // Check if the marker ids fromt he current frame exist in all the previous
    // keyframes first get the mapped marker from the keyframes
    for (const auto currentMapMarker : p_atlas->getAllMarkers())
    {
        // Check if the marker is already in the Global map
        for (auto currentFrameMaker : currentFrame.mapMarkers)
            if (currentFrameMaker->getId() == currentMapMarker->getId())
                currentFrameMaker->setMarkerInGMap(true);
    }

    p_localMapper->insertKeyFrame(pKF);

    p_localMapper->setNotStop(false);

    lastKeyFrameId = currentFrame.mnId;
    p_lastKeyFrame = pKF;
}

void Tracking::searchLocalPoints()
{
    // Do not search map points already matched
    for (vector<MapPoint *>::iterator vit  = currentFrame.mapPoints.begin(),
                                      vend = currentFrame.mapPoints.end();
         vit != vend;
         vit++)
    {
        MapPoint *pMP = *vit;
        if (pMP)
        {
            if (pMP->isBad())
            {
                *vit = static_cast<MapPoint *>(nullptr);
            }
            else
            {
                pMP->increaseVisible();
                pMP->lastSeenFrameId = currentFrame.mnId;
                pMP->trackInView     = false;
                pMP->trackInViewR    = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit  = localMapPoints.begin(),
                                      vend = localMapPoints.end();
         vit != vend;
         vit++)
    {
        MapPoint *pMP = *vit;

        if (pMP->lastSeenFrameId == currentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (currentFrame.isInFrustum(pMP, 0.5))
        {
            pMP->increaseVisible();
            nToMatch++;
        }
        if (pMP->trackInView)
        {
            currentFrame.projectedPoints[pMP->mnId] =
                cv::Point2f(pMP->trackProjX, pMP->trackProjY);
        }
    }

    if (nToMatch > 0)
    {
        ORBmatcher matcher(0.8);
        int        th = 1;
        if (sensor == System::RGBD || sensor == System::IMU_RGBD)
            th = 3;
        if (p_atlas->isImuInitialized())
        {
            if (p_atlas->getCurrentMap()->getInertialBA2())
                th = 2;
            else
                th = 6;
        }
        else if (!p_atlas->isImuInitialized() &&
                 (sensor == System::IMU_MONOCULAR ||
                  sensor == System::IMU_STEREO || sensor == System::IMU_RGBD))
        {
            th = 10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        if (currentFrame.mnId < lastRelocFrameId + 2)
            th = 5;

        if (state == LOST ||
            state == RECENTLY_LOST) // Lost for less than 1 second
            th = 15;                // 15

        // AGGRESSIVE: Even wider search during degraded tracking in corridors
        // If we have very few inliers, expand search radius significantly
        if (matchesInliers < 30 && matchesInliers > 0)
        {
            th = std::min(th * 3, motionModelMaxSearchRadius);
            Verbose::printMess(
                "[Tracking] Expanded search radius to " + std::to_string(th) +
                    " (inliers: " + std::to_string(matchesInliers) + ")",
                Verbose::VERBOSITY_NORMAL);
        }

        // DEPTH-AIDED TRACKING: For RGB-D, use depth to guide matching window
        // In low-texture corridors, constrain search using known depth
        if ((sensor == System::RGBD || sensor == System::IMU_RGBD) &&
            currentFrame.depths.size() > 0)
        {
            // Depth-guided search: reduce search radius for points with
            // reliable depth This helps in repetitive corridors where visual
            // appearance is ambiguous
            matcher.searchByProjectionWithDepth(
                currentFrame,
                localMapPoints,
                th,
                p_localMapper->farPoints,
                p_localMapper->farPointsThreshold,
                depthThreshold);
        }
        else
        {
            matcher.searchByProjection(currentFrame,
                                       localMapPoints,
                                       th,
                                       p_localMapper->farPoints,
                                       p_localMapper->farPointsThreshold);
        }
    }
}

void Tracking::updateLocalMap()
{
    // This is for visualization
    p_atlas->setReferenceMapPoints(localMapPoints);

    // Update
    updateLocalKeyFrames();
    updateLocalPoints();
}

void Tracking::updateLocalPoints()
{
    localMapPoints.clear();

    int count_pts = 0;

    for (vector<KeyFrame *>::const_reverse_iterator
             itKF    = localKeyFrames.rbegin(),
             itEndKF = localKeyFrames.rend();
         itKF != itEndKF;
         ++itKF)
    {
        KeyFrame                *pKF   = *itKF;
        const vector<MapPoint *> vpMPs = pKF->getMapPointMatches();

        for (vector<MapPoint *>::const_iterator itMP    = vpMPs.begin(),
                                                itEndMP = vpMPs.end();
             itMP != itEndMP;
             itMP++)
        {

            MapPoint *pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->trackReferenceFrameId == currentFrame.mnId)
                continue;
            if (!pMP->isBad())
            {
                count_pts++;
                localMapPoints.push_back(pMP);
                pMP->trackReferenceFrameId = currentFrame.mnId;
            }
        }
    }
}

void Tracking::updateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame *, int> keyframeCounter;
    if (!p_atlas->isImuInitialized() ||
        (currentFrame.mnId < lastRelocFrameId + 2))
    {
        for (int i = 0; i < currentFrame.N; i++)
        {
            MapPoint *pMP = currentFrame.mapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    const map<KeyFrame *, tuple<int, int>> observations =
                        pMP->getObservations();
                    for (map<KeyFrame *, tuple<int, int>>::const_iterator
                             it    = observations.begin(),
                             itend = observations.end();
                         it != itend;
                         it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    currentFrame.mapPoints[i] = nullptr;
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < lastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if (lastFrame.mapPoints[i])
            {
                MapPoint *pMP = lastFrame.mapPoints[i];
                if (!pMP)
                    continue;
                if (!pMP->isBad())
                {
                    const map<KeyFrame *, tuple<int, int>> observations =
                        pMP->getObservations();
                    for (map<KeyFrame *, tuple<int, int>>::const_iterator
                             it    = observations.begin(),
                             itend = observations.end();
                         it != itend;
                         it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    lastFrame.mapPoints[i] = nullptr;
                }
            }
        }
    }

    int       max    = 0;
    KeyFrame *pKFmax = static_cast<KeyFrame *>(nullptr);

    localKeyFrames.clear();
    localKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map.
    // Also check which keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator it    = keyframeCounter.begin(),
                                              itEnd = keyframeCounter.end();
         it != itEnd;
         it++)
    {
        KeyFrame *pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max)
        {
            max    = it->second;
            pKFmax = pKF;
        }

        localKeyFrames.push_back(pKF);
        pKF->trackReferenceFrameId = currentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to
    // already-included keyframes
    for (vector<KeyFrame *>::const_iterator itKF    = localKeyFrames.begin(),
                                            itEndKF = localKeyFrames.end();
         itKF != itEndKF;
         itKF++)
    {
        // Limit the number of keyframes - use configurable max (200 for
        // corridors)
        if (localKeyFrames.size() > static_cast<size_t>(maxKFsInLocalMap))
        {
            break;
        }

        KeyFrame *pKF = *itKF;

        const vector<KeyFrame *> vNeighs =
            pKF->getBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame *>::const_iterator itNeighKF    = vNeighs.begin(),
                                                itEndNeighKF = vNeighs.end();
             itNeighKF != itEndNeighKF;
             itNeighKF++)
        {
            KeyFrame *pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad())
            {
                if (pNeighKF->trackReferenceFrameId != currentFrame.mnId)
                {
                    localKeyFrames.push_back(pNeighKF);
                    pNeighKF->trackReferenceFrameId = currentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame *> spChilds = pKF->getChilds();
        for (set<KeyFrame *>::const_iterator sit  = spChilds.begin(),
                                             send = spChilds.end();
             sit != send;
             sit++)
        {
            KeyFrame *pChildKF = *sit;
            if (!pChildKF->isBad())
            {
                if (pChildKF->trackReferenceFrameId != currentFrame.mnId)
                {
                    localKeyFrames.push_back(pChildKF);
                    pChildKF->trackReferenceFrameId = currentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame *pParent = pKF->getParent();
        if (pParent)
        {
            if (pParent->trackReferenceFrameId != currentFrame.mnId)
            {
                localKeyFrames.push_back(pParent);
                pParent->trackReferenceFrameId = currentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if ((sensor == System::IMU_MONOCULAR || sensor == System::IMU_STEREO ||
         sensor == System::IMU_RGBD) &&
        localKeyFrames.size() < 80)
    {
        KeyFrame *tempKeyFrame = currentFrame.p_lastKeyFrame;

        const int Nd = 20;
        for (int i = 0; i < Nd; i++)
        {
            if (!tempKeyFrame)
                break;
            if (tempKeyFrame->trackReferenceFrameId != currentFrame.mnId)
            {
                localKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->trackReferenceFrameId = currentFrame.mnId;
                tempKeyFrame                        = tempKeyFrame->p_prevKF;
            }
        }
    }

    if (pKFmax)
    {
        p_referenceKF                    = pKFmax;
        currentFrame.p_referenceKeyFrame = p_referenceKF;
    }
}

bool Tracking::relocalization()
{
    Verbose::printMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    currentFrame.computeBagOfWords();

    // STRUCTURAL PRIORS: Use room centroids from S-Graph to guide
    // relocalization In office corridors, room/passage markers provide strong
    // topological priors
    vector<Eigen::Vector3f>  roomCentroids;
    vector<semantic::Room *> currentRooms;
    Map                     *pCurrentMap = p_atlas->getCurrentMap();
    if (pCurrentMap)
    {
        const auto &rooms = pCurrentMap->getAllDetectedMapRooms();
        for (semantic::Room *pRoom : rooms)
        {
            if (!pRoom->isBad() && pRoom->getBoundaryStatus() ==
                                       semantic::Room::BoundaryStatus::COMPLETE)
            {
                Eigen::Vector3d centroid_d = pRoom->getCentroid();
                roomCentroids.push_back(Eigen::Vector3f(centroid_d.x(),
                                                        centroid_d.y(),
                                                        centroid_d.z()));
                currentRooms.push_back(pRoom);
            }
        }
    }

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for
    // relocalisation
    vector<KeyFrame *> vpCandidateKFs =
        p_keyFrameDatabase->detectRelocalizationCandidates(
            &currentFrame,
            p_atlas->getCurrentMap());

    if (vpCandidateKFs.empty())
    {
        Verbose::printMess("There are not candidates",
                           Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<MLPnPsolver *> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *>> vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++)
    {
        KeyFrame *pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches =
                matcher.searchByBoW(pKF, currentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                MLPnPsolver *pSolver =
                    new MLPnPsolver(currentFrame, vvpMapPointMatches[i]);
                pSolver->setRansacParameters(
                    0.99,
                    10,
                    300,
                    6,
                    0.5,
                    5.991); // This solver needs at least 6 points
                vpMLPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // STRUCTURAL PRIOR: Re-rank candidates by proximity to room centroids
    // This helps in repetitive corridors where visual appearance is similar
    if (!roomCentroids.empty() && !vpCandidateKFs.empty())
    {
        // Get current frame's estimated position from IMU prediction or motion
        // model
        Eigen::Vector3f currentPos =
            currentFrame.getPose().translation().head<3>();

        // Score candidates by: visual matches + proximity to known room
        // centroids
        vector<float> candidateScores(nKFs, 0.0f);
        for (int i = 0; i < nKFs; i++)
        {
            if (vbDiscarded[i])
                continue;

            KeyFrame       *pKF   = vpCandidateKFs[i];
            Eigen::Vector3f kfPos = pKF->getPose().translation().head<3>();

            // Visual match score (normalized)
            int nmatches       = vvpMapPointMatches[i].size();
            candidateScores[i] = nmatches * 1.0f;

            // Structural prior: proximity to room centroids
            for (size_t r = 0; r < roomCentroids.size(); r++)
            {
                float dist = (kfPos - roomCentroids[r]).norm();
                // Boost score if KF is near a known room centroid (within 3m)
                if (dist < 3.0f)
                    candidateScores[i] +=
                        (3.0f - dist) * 2.0f; // Max boost of 6
            }
        }

        // Re-sort candidates by combined score (highest first)
        vector<int> sortedIndices(nKFs);
        for (int i = 0; i < nKFs; i++)
            sortedIndices[i] = i;
        std::sort(sortedIndices.begin(),
                  sortedIndices.end(),
                  [&](int a, int b)
                  { return candidateScores[a] > candidateScores[b]; });

        // Reorder vectors for processing
        vector<KeyFrame *>         reorderedKFs       = vpCandidateKFs;
        vector<vector<MapPoint *>> reorderedMatches   = vvpMapPointMatches;
        vector<MLPnPsolver *>      reorderedSolvers   = vpMLPnPsolvers;
        vector<bool>               reorderedDiscarded = vbDiscarded;

        for (int i = 0; i < nKFs; i++)
        {
            vpCandidateKFs[i]     = reorderedKFs[sortedIndices[i]];
            vvpMapPointMatches[i] = reorderedMatches[sortedIndices[i]];
            vpMLPnPsolvers[i]     = reorderedSolvers[sortedIndices[i]];
            vbDiscarded[i]        = reorderedDiscarded[sortedIndices[i]];
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool       bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch)
    {
        for (int i = 0; i < nKFs; i++)
        {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int          nInliers;
            bool         bNoMore;

            MLPnPsolver    *pSolver = vpMLPnPsolvers[i];
            Eigen::Matrix4f eigTcw;
            bool            bTcw =
                pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore)
            {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (bTcw)
            {
                Sophus::SE3f Tcw(eigTcw);
                currentFrame.setPose(Tcw);
                // Tcw.copyTo(mCurrentFrame.poseTcw);

                set<MapPoint *> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++)
                {
                    if (vbInliers[j])
                    {
                        currentFrame.mapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        currentFrame.mapPoints[j] = nullptr;
                }

                int nGood = Optimizer::poseOptimization(&currentFrame);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < currentFrame.N; io++)
                    if (currentFrame.outlierFlags[io])
                        currentFrame.mapPoints[io] =
                            static_cast<MapPoint *>(nullptr);

                // If few inliers, search by projection in a coarse window and
                // optimize again
                if (nGood < 50)
                {
                    int nadditional =
                        matcher2.searchByProjection(currentFrame,
                                                    vpCandidateKFs[i],
                                                    sFound,
                                                    10,
                                                    100);

                    if (nadditional + nGood >= 50)
                    {
                        nGood = Optimizer::poseOptimization(&currentFrame);

                        // If many inliers but still not enough, search by
                        // projection again in a narrower window the camera has
                        // been already optimized with many points
                        if (nGood > 30 && nGood < 50)
                        {
                            sFound.clear();
                            for (int ip = 0; ip < currentFrame.N; ip++)
                                if (currentFrame.mapPoints[ip])
                                    sFound.insert(currentFrame.mapPoints[ip]);
                            nadditional =
                                matcher2.searchByProjection(currentFrame,
                                                            vpCandidateKFs[i],
                                                            sFound,
                                                            3,
                                                            64);

                            // Final optimization
                            if (nGood + nadditional >= 50)
                            {
                                nGood =
                                    Optimizer::poseOptimization(&currentFrame);

                                for (int io = 0; io < currentFrame.N; io++)
                                    if (currentFrame.outlierFlags[io])
                                        currentFrame.mapPoints[io] = nullptr;
                            }
                        }
                    }
                }

                // If the pose is supported by enough inliers stop ransacs and
                // continue
                // LOWERED: 25 -> 10 inliers for relocalization in textureless
                // corridors Use configurable threshold
                if (nGood >= relocalizationMinInliers)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch)
    {
        return false;
    }
    else
    {
        lastRelocFrameId = currentFrame.mnId;
        std::cout << "[Tracking] Relocalized!" << std::endl;
        return true;
    }
}

void Tracking::reset(bool bLocMap)
{
    Verbose::printMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    if (p_viewer)
    {
        p_viewer->requestStop();
        while (!p_viewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::printMess("Reseting Local Mapper...",
                           Verbose::VERBOSITY_NORMAL);
        p_localMapper->requestReset();
        Verbose::printMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::printMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    p_loopClosing->requestReset();
    Verbose::printMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::printMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    p_keyFrameDatabase->clear();
    Verbose::printMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    p_atlas->clearAtlas();
    p_atlas->createNewMap();
    if (sensor == System::IMU_STEREO || sensor == System::IMU_MONOCULAR ||
        sensor == System::IMU_RGBD)
        p_atlas->setInertialSensor();
    initialFrameId = 0;

    KeyFrame::nNextId = 0;
    Frame::nNextId    = 0;
    state             = NO_IMAGES_YET;

    readyToInitialize = false;
    isInitSet         = false;

    relativeFramePoses.clear();
    mlpReferences.clear();
    frameTimes.clear();
    mlbLost.clear();
    currentFrame     = Frame();
    lastRelocFrameId = 0;
    lastFrame        = Frame();
    p_referenceKF    = static_cast<KeyFrame *>(nullptr);
    p_lastKeyFrame   = static_cast<KeyFrame *>(nullptr);
    iniMatches.clear();

    if (p_viewer)
        p_viewer->release();

    Verbose::printMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::resetActiveMap(bool bLocMap)
{
    if (p_loopClosing)
    {
        while (p_loopClosing->isMergeInProgress())
        {
            usleep(1000);
        }
    }

    Verbose::printMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if (p_viewer)
    {
        p_viewer->requestStop();
        while (!p_viewer->isStopped())
        {
            usleep(3000);
        }
    }

    Map *pMap = p_atlas->getCurrentMap();

    if (!bLocMap)
    {
        Verbose::printMess("[Tracking] Reseting 'LocalMapping' ...",
                           Verbose::VERBOSITY_VERY_VERBOSE);
        p_localMapper->requestResetActiveMap(pMap);
        Verbose::printMess("[Tracking] Finished resetting 'LocalMapping'!",
                           Verbose::VERBOSITY_VERY_VERBOSE);
    }

    // Reset Loop Closing
    Verbose::printMess("[Tracking] Reseting 'LoopClosing' ...",
                       Verbose::VERBOSITY_NORMAL);
    p_loopClosing->requestResetActiveMap(pMap);
    Verbose::printMess("[Tracking] Finished resetting 'LocalMapping'!",
                       Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::printMess("[Tracking] Reseting 'Database' ...",
                       Verbose::VERBOSITY_NORMAL);
    p_keyFrameDatabase->clearMap(pMap);
    Verbose::printMess("[Tracking] Finished resetting 'Database'!",
                       Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    p_atlas->clearMap();

    lastInitFrameId = Frame::nNextId;
    state           = NO_IMAGES_YET;

    readyToInitialize = false;

    unsigned int index = firstFrameId;
    for (Map *pMap : p_atlas->getAllMaps())
        if (pMap->getAllKeyFrames().size() > 0)
            if (index > pMap->getLowerKeyFrameId())
                index = pMap->getLowerKeyFrameId();

    // Count lost frames
    std::list<bool> lbLost;
    int             lostFrameCount = 0;
    for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end();
         ilbL++)
    {
        if (index < initialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            lostFrameCount += 1;
        }
        index++;
    }
    std::cout << "[Tracking] " << lostFrameCount << " frames were set to lost!"
              << endl;

    mlbLost = lbLost;

    initialFrameId   = currentFrame.mnId;
    lastRelocFrameId = currentFrame.mnId;

    currentFrame   = Frame();
    lastFrame      = Frame();
    p_referenceKF  = static_cast<KeyFrame *>(nullptr);
    p_lastKeyFrame = static_cast<KeyFrame *>(nullptr);
    iniMatches.clear();

    velocityAvailable = false;

    if (p_viewer)
        p_viewer->release();
}

vector<MapPoint *> Tracking::getLocalMapPoints()
{
    return localMapPoints;
}

void Tracking::changeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float           fx = fSettings["Camera.fx"];
    float           fy = fSettings["Camera.fy"];
    float           cx = fSettings["Camera.cx"];
    float           cy = fSettings["Camera.cy"];

    calibrationMatrixEigen.setIdentity();
    calibrationMatrixEigen(0, 0) = fx;
    calibrationMatrixEigen(1, 1) = fy;
    calibrationMatrixEigen(0, 2) = cx;
    calibrationMatrixEigen(1, 2) = cy;

    cv::Mat K         = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(calibrationMatrix);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3        = fSettings["Camera.k3"];
    if (k3 != 0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(distortionCoefficients);

    mbf = fSettings["Camera.bf"];

    Frame::initialComputationsDone = true;
}

void Tracking::informOnlyTracking(const bool &flag)
{
    onlyTracking = flag;
}

void Tracking::updateFrameIMU(const float      s,
                              const IMU::Bias &b,
                              KeyFrame        *pCurrentKeyFrame)
{
    Map *pMap = pCurrentKeyFrame->getMap();
    list<vs_graphs::core::KeyFrame *>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator                        lbL  = mlbLost.begin();
    for (auto lit = relativeFramePoses.begin(), lend = relativeFramePoses.end();
         lit != lend;
         lit++, lRit++, lbL++)
    {
        if (*lbL)
            continue;

        KeyFrame *pKF = *lRit;

        while (pKF->isBad() && pKF->getParent())
        {
            pKF = pKF->getParent();
        }

        if (pKF->getMap() == pMap)
        {
            (*lit).translation() *= s;
        }
    }

    lastBias = b;

    p_lastKeyFrame = pCurrentKeyFrame;

    lastFrame.setNewBias(lastBias);
    currentFrame.setNewBias(lastBias);

    while (!currentFrame.isImuPreintegrated())
    {
        usleep(500);
    }

    if (lastFrame.mnId == lastFrame.p_lastKeyFrame->frameId)
    {
        lastFrame.setImuPoseVelocity(lastFrame.p_lastKeyFrame->getImuRotation(),
                                     lastFrame.p_lastKeyFrame->getImuPosition(),
                                     lastFrame.p_lastKeyFrame->getVelocity());
    }
    else
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
        const Eigen::Vector3f twb1 = lastFrame.p_lastKeyFrame->getImuPosition();
        const Eigen::Matrix3f Rwb1 = lastFrame.p_lastKeyFrame->getImuRotation();
        const Eigen::Vector3f Vwb1 = lastFrame.p_lastKeyFrame->getVelocity();
        float                 t12  = lastFrame.p_imuPreintegrated->dT;

        lastFrame.setImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 * lastFrame.p_imuPreintegrated->getUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 * lastFrame.p_imuPreintegrated->getUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 +
                Rwb1 * lastFrame.p_imuPreintegrated->getUpdatedDeltaVelocity());
    }

    if (currentFrame.p_imuPreintegrated)
    {
        const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

        const Eigen::Vector3f twb1 =
            currentFrame.p_lastKeyFrame->getImuPosition();
        const Eigen::Matrix3f Rwb1 =
            currentFrame.p_lastKeyFrame->getImuRotation();
        const Eigen::Vector3f Vwb1 = currentFrame.p_lastKeyFrame->getVelocity();
        float                 t12  = currentFrame.p_imuPreintegrated->dT;

        currentFrame.setImuPoseVelocity(
            IMU::NormalizeRotation(
                Rwb1 *
                currentFrame.p_imuPreintegrated->getUpdatedDeltaRotation()),
            twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                Rwb1 *
                    currentFrame.p_imuPreintegrated->getUpdatedDeltaPosition(),
            Vwb1 + Gz * t12 +
                Rwb1 *
                    currentFrame.p_imuPreintegrated->getUpdatedDeltaVelocity());
    }

    firstImuFrameId = currentFrame.mnId;
}

void Tracking::newDataset()
{
    numDataset++;
}

int Tracking::getNumberDataset()
{
    return numDataset;
}

int Tracking::getMatchesInliers()
{
    return matchesInliers;
}

void Tracking::saveSubTrajectory(string strNameFile_frames,
                                 string strNameFile_kf,
                                 string strFolder)
{
    (void)strNameFile_kf;
    p_system->saveTrajectoryEuRoC(strFolder + strNameFile_frames);
    // mpSystem->SaveKeyFrameTrajectoryEuRoC(strFolder + strNameFile_kf);
}

void Tracking::saveSubTrajectory(string strNameFile_frames,
                                 string strNameFile_kf,
                                 Map   *pMap)
{
    p_system->saveTrajectoryEuRoC(strNameFile_frames, pMap);
    if (!strNameFile_kf.empty())
        p_system->saveKeyFrameTrajectoryEuRoC(strNameFile_kf, pMap);
}

float Tracking::getImageScale()
{
    return imageScale;
}

Sophus::SE3f Tracking::getCamTwc()
{
    return (currentFrame.getPose()).inverse();
}

Sophus::SE3f Tracking::getImuTwb()
{
    return currentFrame.getImuPose();
}

Eigen::Vector3f Tracking::getImuVwb()
{
    return currentFrame.getVelocity();
}

bool Tracking::isImuPreintegrated()
{
    return currentFrame.p_imuPreintegrated;
}

// Semantic Entities
std::vector<MapPoint *>
    Tracking::findPointsCloseToMarker(const semantic::Marker *currentMarker)
{
    // Get all map points
    std::vector<MapPoint *> allmapPoints = p_atlas->getAllMapPoints();
    // Get all map points close to the marker
    std::vector<MapPoint *> closePoints =
        findPointsCloseToLocation(allmapPoints,
                                  currentMarker->getGlobalPose().translation(),
                                  0.1);
    // Return the close points
    return closePoints;
}

std::vector<MapPoint *>
    Tracking::findPointsCloseToLocation(const std::vector<MapPoint *> &points,
                                        const Eigen::Vector3f         &location,
                                        double distanceThreshold)
{
    std::vector<MapPoint *> closePoints;
    for (MapPoint *point : points)
    {
        double distance =
            Utils::calculateEuclideanDistance(point->getWorldPos(), location);
        if (distance <= distanceThreshold)
        {
            closePoints.push_back(point);
        }
    }

    return closePoints;
}

double Tracking::getMarkerImpact() const
{
    return markerImpact;
};

void Tracking::setMarkerImpact(const double newValue)
{
    markerImpact = newValue;
};

#ifdef REGISTER_LOOP
void Tracking::requestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    stopRequestedFlag = true;
}

bool Tracking::stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if (stopRequestedFlag && !notStop)
    {
        stopped = true;
        cout << "Tracking STOP" << endl;
        return true;
    }

    return false;
}

bool Tracking::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return stopRequestedFlag;
}

bool Tracking::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return stopped;
}

void Tracking::release()
{
    unique_lock<mutex> lock(mMutexStop);
    stopped           = false;
    stopRequestedFlag = false;
}
#endif

} // namespace core
} // namespace vs_graphs
