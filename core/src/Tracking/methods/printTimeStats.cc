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

#include "LocalMapping.h"
#include "LoopClosing.h"

#include "../private_functions.h"

#include <iomanip>
#include <iostream>
#include <rclcpp/logging.hpp>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{

#ifdef REGISTER_TIMES
TrackingStatus Tracking::printTimeStats()
{
    // Save data in files
    if (trackStats2File() != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: trackStats2File returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (localMapStats2File() != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: localMapStats2File returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::ofstream f;
    f.open("ExecMean.txt");
    f << std::fixed;
    // Report the mean and std of each one
    std::cout << std::endl << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    f << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    f << "---------------------------" << std::endl;
    f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    if (!stereoRectificationTimes_ms.empty())
    {
        double average2{};
        if (calcAverage(stereoRectificationTimes_ms, average2) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calcAverage returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        average = average2;
        double deviation2{};
        if (calcDeviation(stereoRectificationTimes_ms, average, deviation2) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calcDeviation returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        deviation = deviation2;
        std::cout << "Stereo Rectification: " << average << "$\\pm$"
                  << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    if (!imageResizeTimes_ms.empty())
    {
        double average3{};
        if (calcAverage(imageResizeTimes_ms, average3) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calcAverage returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        average = average3;
        double deviation3{};
        if (calcDeviation(imageResizeTimes_ms, average, deviation3) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calcDeviation returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        deviation = deviation3;
        std::cout << "Image Resize: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
    }

    double average4{};
    if (calcAverage(orbExtractionTimes_ms, average4) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average4;
    double deviation4{};
    if (calcDeviation(orbExtractionTimes_ms, average, deviation4) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation4;
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if (!stereoMatchTimes_ms.empty())
    {
        double average5{};
        if (calcAverage(stereoMatchTimes_ms, average5) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calcAverage returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        average = average5;
        double deviation5{};
        if (calcDeviation(stereoMatchTimes_ms, average, deviation5) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calcDeviation returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        deviation = deviation5;
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    if (!imuIntegrationTimes_ms.empty())
    {
        double average6{};
        if (calcAverage(imuIntegrationTimes_ms, average6) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calcAverage returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        average = average6;
        double deviation6{};
        if (calcDeviation(imuIntegrationTimes_ms, average, deviation6) !=
            TrackingStatus::TRACKING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: calcDeviation returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        deviation = deviation6;
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    double average7{};
    if (calcAverage(posePredictionTimes_ms, average7) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average7;
    double deviation7{};
    if (calcDeviation(posePredictionTimes_ms, average, deviation7) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation7;
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    double average8{};
    if (calcAverage(localMapTrackTimes_ms, average8) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average8;
    double deviation8{};
    if (calcDeviation(localMapTrackTimes_ms, average, deviation8) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation8;
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    double average9{};
    if (calcAverage(newKeyFrameTimes_ms, average9) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average9;
    double deviation9{};
    if (calcDeviation(newKeyFrameTimes_ms, average, deviation9) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation9;
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    double average10{};
    if (calcAverage(trackTotalTimes_ms, average10) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average10;
    double deviation10{};
    if (calcDeviation(trackTotalTimes_ms, average, deviation10) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation10;
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    double average11{};
    if (calcAverage(p_localMapper->keyFrameInsertTimes_ms, average11) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average11;
    double deviation11{};
    if (calcDeviation(p_localMapper->keyFrameInsertTimes_ms,
                      average,
                      deviation11) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation11;
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    double average12{};
    if (calcAverage(p_localMapper->mapPointCullingTimes_ms, average12) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average12;
    double deviation12{};
    if (calcDeviation(p_localMapper->mapPointCullingTimes_ms,
                      average,
                      deviation12) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation12;
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    double average13{};
    if (calcAverage(p_localMapper->mapPointCreationTimes_ms, average13) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average13;
    double deviation13{};
    if (calcDeviation(p_localMapper->mapPointCreationTimes_ms,
                      average,
                      deviation13) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation13;
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    double average14{};
    if (calcAverage(p_localMapper->localBaTimes_ms, average14) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average14;
    double deviation14{};
    if (calcDeviation(p_localMapper->localBaTimes_ms, average, deviation14) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation14;
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    double average15{};
    if (calcAverage(p_localMapper->keyFrameCullingTimes_ms, average15) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average15;
    double deviation15{};
    if (calcDeviation(p_localMapper->keyFrameCullingTimes_ms,
                      average,
                      deviation15) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation15;
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    double average16{};
    if (calcAverage(p_localMapper->localMappingTotalTimes_ms, average16) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average16;
    double deviation16{};
    if (calcDeviation(p_localMapper->localMappingTotalTimes_ms,
                      average,
                      deviation16) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation16;
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation
      << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    double average17{};
    if (calcAverage(p_localMapper->localBaEdgeCounts, average17) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average17;
    double deviation17{};
    if (calcDeviation(p_localMapper->localBaEdgeCounts, average, deviation17) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation17;
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    double average18{};
    if (calcAverage(p_localMapper->localBaOptimizedKeyFrameCounts, average18) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average18;
    double deviation18{};
    if (calcDeviation(p_localMapper->localBaOptimizedKeyFrameCounts,
                      average,
                      deviation18) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation18;
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    double average19{};
    if (calcAverage(p_localMapper->localBaFixedKeyFrameCounts, average19) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average19;
    double deviation19{};
    if (calcDeviation(p_localMapper->localBaFixedKeyFrameCounts,
                      average,
                      deviation19) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation19;
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    double average20{};
    if (calcAverage(p_localMapper->localBaMapPointCounts, average20) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average20;
    double deviation20{};
    if (calcDeviation(p_localMapper->localBaMapPointCounts,
                      average,
                      deviation20) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation20;
    std::cout << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
              << std::endl;
    f << "LBA MP: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;

    std::cout << "LBA executions: " << p_localMapper->localBaExecutionCount
              << std::endl;
    std::cout << "LBA aborts: " << p_localMapper->localBaAbortCount
              << std::endl;
    f << "LBA executions: " << p_localMapper->localBaExecutionCount
      << std::endl;
    f << "LBA aborts: " << p_localMapper->localBaAbortCount << std::endl;

    // Map complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Map complexity" << std::endl;
    std::vector<KeyFrame *> atlasAllKeyFrames{};
    if (p_atlas->getAllKeyFrames(atlasAllKeyFrames) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "KFs in map: " << atlasAllKeyFrames.size() << std::endl;
    std::vector<MapPoint *> atlasAllMapPoints{};
    if (p_atlas->getAllMapPoints(atlasAllMapPoints) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::cout << "MPs in map: " << atlasAllMapPoints.size() << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    std::vector<Map *> maps{};
    if (p_atlas->getAllMaps(maps) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMaps returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Map *p_bestMap = maps[0];
    for (std::size_t mapIndex = 1; mapIndex < maps.size(); ++mapIndex)
    {
        std::vector<KeyFrame *> bestMapKeyFrames;
        if (p_bestMap->getAllKeyFrames(bestMapKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getAllKeyFrames returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        std::vector<KeyFrame *> mapKeyFrames;
        if (maps[mapIndex]->getAllKeyFrames(mapKeyFrames) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getAllKeyFrames returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
        if (bestMapKeyFrames.size() < mapKeyFrames.size())
        {
            p_bestMap = maps[mapIndex];
        }
    }

    std::vector<KeyFrame *> bestMapKeyFrames;
    if (p_bestMap->getAllKeyFrames(bestMapKeyFrames) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getAllKeyFrames returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
    std::vector<MapPoint *> bestMapMapPoints;
    if (p_bestMap->getAllMapPoints(bestMapMapPoints) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: getAllMapPoints returned a failure status although it "
            "cannot fail; continuing as before.",
            __func__);
    }
    f << "KFs in map: " << bestMapKeyFrames.size() << std::endl;
    f << "MPs in map: " << bestMapMapPoints.size() << std::endl;

    f << "---------------------------" << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    double average21{};
    if (calcAverage(p_loopClosing->dataQueryTimes_ms, average21) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average21;
    double deviation21{};
    if (calcDeviation(p_loopClosing->dataQueryTimes_ms, average, deviation21) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation21;
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Database Query: " << average << "$\\pm$" << deviation
              << std::endl;
    double average22{};
    if (calcAverage(p_loopClosing->sim3EstimationTimes_ms, average22) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average22;
    double deviation22{};
    if (calcDeviation(p_loopClosing->sim3EstimationTimes_ms,
                      average,
                      deviation22) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation22;
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation
              << std::endl;
    double average23{};
    if (calcAverage(p_loopClosing->placeRecognitionTotalTimes_ms, average23) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average23;
    double deviation23{};
    if (calcDeviation(p_loopClosing->placeRecognitionTotalTimes_ms,
                      average,
                      deviation23) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation23;
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation
      << std::endl
      << std::endl;
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    double average24{};
    if (calcAverage(p_loopClosing->loopFusionTimes_ms, average24) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average24;
    double deviation24{};
    if (calcDeviation(p_loopClosing->loopFusionTimes_ms,
                      average,
                      deviation24) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation24;
    f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation
              << std::endl;
    double average25{};
    if (calcAverage(p_loopClosing->loopEssentialGraphTimes_ms, average25) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average25;
    double deviation25{};
    if (calcDeviation(p_loopClosing->loopEssentialGraphTimes_ms,
                      average,
                      deviation25) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation25;
    f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Essential Graph: " << average << "$\\pm$" << deviation
              << std::endl;
    double average26{};
    if (calcAverage(p_loopClosing->loopTotalTimes_ms, average26) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average26;
    double deviation26{};
    if (calcDeviation(p_loopClosing->loopTotalTimes_ms, average, deviation26) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation26;
    f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << "Numb exec: " << p_loopClosing->loopCount << std::endl;
    std::cout << "Num exec: " << p_loopClosing->loopCount << std::endl;
    double average27{};
    if (calcAverage(p_loopClosing->loopKeyFrameCounts, average27) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average27;
    double deviation27{};
    if (calcDeviation(p_loopClosing->loopKeyFrameCounts,
                      average,
                      deviation27) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation27;
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;

    f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    double average28{};
    if (calcAverage(p_loopClosing->mergeMapsTimes_ms, average28) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average28;
    double deviation28{};
    if (calcDeviation(p_loopClosing->mergeMapsTimes_ms, average, deviation28) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation28;
    f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Merge Maps: " << average << "$\\pm$" << deviation
              << std::endl;
    double average29{};
    if (calcAverage(p_loopClosing->weldingBaTimes_ms, average29) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average29;
    double deviation29{};
    if (calcDeviation(p_loopClosing->weldingBaTimes_ms, average, deviation29) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation29;
    f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Welding BA: " << average << "$\\pm$" << deviation
              << std::endl;
    double average30{};
    if (calcAverage(p_loopClosing->mergeEssentialGraphTimes_ms, average30) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average30;
    double deviation30{};
    if (calcDeviation(p_loopClosing->mergeEssentialGraphTimes_ms,
                      average,
                      deviation30) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation30;
    f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation
              << std::endl;
    double average31{};
    if (calcAverage(p_loopClosing->mergeTotalTimes_ms, average31) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average31;
    double deviation31{};
    if (calcDeviation(p_loopClosing->mergeTotalTimes_ms,
                      average,
                      deviation31) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation31;
    f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << "Numb exec: " << p_loopClosing->mergeCount << std::endl;
    std::cout << "Num exec: " << p_loopClosing->mergeCount << std::endl;
    double average32{};
    if (calcAverage(p_loopClosing->mergeKeyFrameCounts, average32) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average32;
    double deviation32{};
    if (calcDeviation(p_loopClosing->mergeKeyFrameCounts,
                      average,
                      deviation32) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation32;
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;
    double average33{};
    if (calcAverage(p_loopClosing->mergeMapPointCounts, average33) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average33;
    double deviation33{};
    if (calcDeviation(p_loopClosing->mergeMapPointCounts,
                      average,
                      deviation33) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation33;
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
              << std::endl;

    f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    double average34{};
    if (calcAverage(p_loopClosing->gbaTimes_ms, average34) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average34;
    double deviation34{};
    if (calcDeviation(p_loopClosing->gbaTimes_ms, average, deviation34) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation34;
    f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    double average35{};
    if (calcAverage(p_loopClosing->updateMapTimes_ms, average35) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average35;
    double deviation35{};
    if (calcDeviation(p_loopClosing->updateMapTimes_ms, average, deviation35) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation35;
    f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Map Update: " << average << "$\\pm$" << deviation
              << std::endl;
    double average36{};
    if (calcAverage(p_loopClosing->fullGbaTotalTimes_ms, average36) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average36;
    double deviation36{};
    if (calcDeviation(p_loopClosing->fullGbaTotalTimes_ms,
                      average,
                      deviation36) != TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation36;
    f << "Total Full GBA: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    std::cout << "Total Full GBA: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << "Numb exec: " << p_loopClosing->fullGbaExecutionCount << std::endl;
    std::cout << "Num exec: " << p_loopClosing->fullGbaExecutionCount
              << std::endl;
    f << "Numb abort: " << p_loopClosing->fullGbaAbortCount << std::endl;
    std::cout << "Num abort: " << p_loopClosing->fullGbaAbortCount << std::endl;
    double average37{};
    if (calcAverage(p_loopClosing->gbaKeyFrameCounts, average37) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average37;
    double deviation37{};
    if (calcDeviation(p_loopClosing->gbaKeyFrameCounts, average, deviation37) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation37;
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;
    double average38{};
    if (calcAverage(p_loopClosing->gbaMapPointCounts, average38) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcAverage returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    average = average38;
    double deviation38{};
    if (calcDeviation(p_loopClosing->gbaMapPointCounts, average, deviation38) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: calcDeviation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    deviation = deviation38;
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
              << std::endl;

    f.close();

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}
#endif

} // namespace core
} // namespace vs_graphs
