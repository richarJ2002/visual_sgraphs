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

#include "../private_functions.h"

#include <iostream>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{

#ifdef REGISTER_TIMES
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
    if (!stereoRectificationTimes_ms.empty())
    {
        average   = calcAverage(stereoRectificationTimes_ms);
        deviation = calcDeviation(stereoRectificationTimes_ms, average);
        std::cout << "Stereo Rectification: " << average << "$\\pm$"
                  << deviation << std::endl;
        f << "Stereo Rectification: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    if (!imageResizeTimes_ms.empty())
    {
        average   = calcAverage(imageResizeTimes_ms);
        deviation = calcDeviation(imageResizeTimes_ms, average);
        std::cout << "Image Resize: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "Image Resize: " << average << "$\\pm$" << deviation << std::endl;
    }

    average   = calcAverage(orbExtractionTimes_ms);
    deviation = calcDeviation(orbExtractionTimes_ms, average);
    std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;

    if (!stereoMatchTimes_ms.empty())
    {
        average   = calcAverage(stereoMatchTimes_ms);
        deviation = calcDeviation(stereoMatchTimes_ms, average);
        std::cout << "Stereo Matching: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "Stereo Matching: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    if (!imuIntegrationTimes_ms.empty())
    {
        average   = calcAverage(imuIntegrationTimes_ms);
        deviation = calcDeviation(imuIntegrationTimes_ms, average);
        std::cout << "IMU Preintegration: " << average << "$\\pm$" << deviation
                  << std::endl;
        f << "IMU Preintegration: " << average << "$\\pm$" << deviation
          << std::endl;
    }

    average   = calcAverage(posePredictionTimes_ms);
    deviation = calcDeviation(posePredictionTimes_ms, average);
    std::cout << "Pose Prediction: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Pose Prediction: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(localMapTrackTimes_ms);
    deviation = calcDeviation(localMapTrackTimes_ms, average);
    std::cout << "LM Track: " << average << "$\\pm$" << deviation << std::endl;
    f << "LM Track: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(newKeyFrameTimes_ms);
    deviation = calcDeviation(newKeyFrameTimes_ms, average);
    std::cout << "New KF decision: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "New KF decision: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(trackTotalTimes_ms);
    deviation = calcDeviation(trackTotalTimes_ms, average);
    std::cout << "Total Tracking: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Total Tracking: " << average << "$\\pm$" << deviation << std::endl;

    // Local Mapping time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "Local Mapping" << std::endl << std::endl;
    f << std::endl << "Local Mapping" << std::endl << std::endl;

    average   = calcAverage(p_localMapper->keyFrameInsertTimes_ms);
    deviation = calcDeviation(p_localMapper->keyFrameInsertTimes_ms, average);
    std::cout << "KF Insertion: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "KF Insertion: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->mapPointCullingTimes_ms);
    deviation = calcDeviation(p_localMapper->mapPointCullingTimes_ms, average);
    std::cout << "MP Culling: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "MP Culling: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->mapPointCreationTimes_ms);
    deviation = calcDeviation(p_localMapper->mapPointCreationTimes_ms, average);
    std::cout << "MP Creation: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "MP Creation: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->localBaTimes_ms);
    deviation = calcDeviation(p_localMapper->localBaTimes_ms, average);
    std::cout << "LBA: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->keyFrameCullingTimes_ms);
    deviation = calcDeviation(p_localMapper->keyFrameCullingTimes_ms, average);
    std::cout << "KF Culling: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "KF Culling: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(p_localMapper->localMappingTotalTimes_ms);
    deviation =
        calcDeviation(p_localMapper->localMappingTotalTimes_ms, average);
    std::cout << "Total Local Mapping: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "Total Local Mapping: " << average << "$\\pm$" << deviation
      << std::endl;

    // Local Mapping LBA complexity
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "LBA complexity (mean$\\pm$std)" << std::endl;

    average   = calcAverage(p_localMapper->localBaEdgeCounts);
    deviation = calcDeviation(p_localMapper->localBaEdgeCounts, average);
    std::cout << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;
    f << "LBA Edges: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(p_localMapper->localBaOptimizedKeyFrameCounts);
    deviation =
        calcDeviation(p_localMapper->localBaOptimizedKeyFrameCounts, average);
    std::cout << "LBA KF optimized: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "LBA KF optimized: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(p_localMapper->localBaFixedKeyFrameCounts);
    deviation =
        calcDeviation(p_localMapper->localBaFixedKeyFrameCounts, average);
    std::cout << "LBA KF fixed: " << average << "$\\pm$" << deviation
              << std::endl;
    f << "LBA KF fixed: " << average << "$\\pm$" << deviation << std::endl;

    average   = calcAverage(p_localMapper->localBaMapPointCounts);
    deviation = calcDeviation(p_localMapper->localBaMapPointCounts, average);
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
    std::cout << "KFs in map: " << p_atlas->getAllKeyFrames().size()
              << std::endl;
    std::cout << "MPs in map: " << p_atlas->getAllMapPoints().size()
              << std::endl;
    f << "---------------------------" << std::endl;
    f << std::endl << "Map complexity" << std::endl;
    vector<Map *> maps      = p_atlas->getAllMaps();
    Map          *p_bestMap = maps[0];
    for (int mapIndex = 1; mapIndex < maps.size(); ++mapIndex)
    {
        if (p_bestMap->getAllKeyFrames().size() <
            maps[mapIndex]->getAllKeyFrames().size())
        {
            p_bestMap = maps[mapIndex];
        }
    }

    f << "KFs in map: " << p_bestMap->getAllKeyFrames().size() << std::endl;
    f << "MPs in map: " << p_bestMap->getAllMapPoints().size() << std::endl;

    f << "---------------------------" << std::endl;
    f << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << std::endl << "Place Recognition (mean$\\pm$std)" << std::endl;
    average   = calcAverage(p_loopClosing->dataQueryTimes_ms);
    deviation = calcDeviation(p_loopClosing->dataQueryTimes_ms, average);
    f << "Database Query: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Database Query: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->sim3EstimationTimes_ms);
    deviation = calcDeviation(p_loopClosing->sim3EstimationTimes_ms, average);
    f << "SE3 estimation: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "SE3 estimation: " << average << "$\\pm$" << deviation
              << std::endl;
    average = calcAverage(p_loopClosing->placeRecognitionTotalTimes_ms);
    deviation =
        calcDeviation(p_loopClosing->placeRecognitionTotalTimes_ms, average);
    f << "Total Place Recognition: " << average << "$\\pm$" << deviation
      << std::endl
      << std::endl;
    std::cout << "Total Place Recognition: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Loop Closing (mean$\\pm$std)" << std::endl;
    average   = calcAverage(p_loopClosing->loopFusionTimes_ms);
    deviation = calcDeviation(p_loopClosing->loopFusionTimes_ms, average);
    f << "Loop Fusion: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Loop Fusion: " << average << "$\\pm$" << deviation
              << std::endl;
    average = calcAverage(p_loopClosing->loopEssentialGraphTimes_ms);
    deviation =
        calcDeviation(p_loopClosing->loopEssentialGraphTimes_ms, average);
    f << "Essential Graph: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Essential Graph: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->loopTotalTimes_ms);
    deviation = calcDeviation(p_loopClosing->loopTotalTimes_ms, average);
    f << "Total Loop Closing: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    std::cout << "Total Loop Closing: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << "Numb exec: " << p_loopClosing->loopCount << std::endl;
    std::cout << "Num exec: " << p_loopClosing->loopCount << std::endl;
    average   = calcAverage(p_loopClosing->loopKeyFrameCounts);
    deviation = calcDeviation(p_loopClosing->loopKeyFrameCounts, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;

    f << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Map Merging (mean$\\pm$std)" << std::endl;
    average   = calcAverage(p_loopClosing->mergeMapsTimes_ms);
    deviation = calcDeviation(p_loopClosing->mergeMapsTimes_ms, average);
    f << "Merge Maps: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Merge Maps: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->weldingBaTimes_ms);
    deviation = calcDeviation(p_loopClosing->weldingBaTimes_ms, average);
    f << "Welding BA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Welding BA: " << average << "$\\pm$" << deviation
              << std::endl;
    average = calcAverage(p_loopClosing->mergeEssentialGraphTimes_ms);
    deviation =
        calcDeviation(p_loopClosing->mergeEssentialGraphTimes_ms, average);
    f << "Optimization Ess.: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Optimization Ess.: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->mergeTotalTimes_ms);
    deviation = calcDeviation(p_loopClosing->mergeTotalTimes_ms, average);
    f << "Total Map Merging: " << average << "$\\pm$" << deviation << std::endl
      << std::endl;
    std::cout << "Total Map Merging: " << average << "$\\pm$" << deviation
              << std::endl
              << std::endl;

    f << "Numb exec: " << p_loopClosing->mergeCount << std::endl;
    std::cout << "Num exec: " << p_loopClosing->mergeCount << std::endl;
    average   = calcAverage(p_loopClosing->mergeKeyFrameCounts);
    deviation = calcDeviation(p_loopClosing->mergeKeyFrameCounts, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->mergeMapPointCounts);
    deviation = calcDeviation(p_loopClosing->mergeMapPointCounts, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
              << std::endl;

    f << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    std::cout << std::endl << "Full GBA (mean$\\pm$std)" << std::endl;
    average   = calcAverage(p_loopClosing->gbaTimes_ms);
    deviation = calcDeviation(p_loopClosing->gbaTimes_ms, average);
    f << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "GBA: " << average << "$\\pm$" << deviation << std::endl;
    average   = calcAverage(p_loopClosing->updateMapTimes_ms);
    deviation = calcDeviation(p_loopClosing->updateMapTimes_ms, average);
    f << "Map Update: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Map Update: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->fullGbaTotalTimes_ms);
    deviation = calcDeviation(p_loopClosing->fullGbaTotalTimes_ms, average);
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
    average   = calcAverage(p_loopClosing->gbaKeyFrameCounts);
    deviation = calcDeviation(p_loopClosing->gbaKeyFrameCounts, average);
    f << "Number of KFs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of KFs: " << average << "$\\pm$" << deviation
              << std::endl;
    average   = calcAverage(p_loopClosing->gbaMapPointCounts);
    deviation = calcDeviation(p_loopClosing->gbaMapPointCounts, average);
    f << "Number of MPs: " << average << "$\\pm$" << deviation << std::endl;
    std::cout << "Number of MPs: " << average << "$\\pm$" << deviation
              << std::endl;

    f.close();
}
#endif

} // namespace core
} // namespace vs_graphs
