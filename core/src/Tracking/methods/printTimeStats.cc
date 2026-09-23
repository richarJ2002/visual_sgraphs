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

} // namespace core
} // namespace vs_graphs
