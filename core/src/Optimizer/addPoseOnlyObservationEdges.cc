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

#include "Frame.h"
#include "G2oTypes.h"
#include "MapPoint.h"
#include "Optimizer.h"

#include "private_functions.h"

#include <cmath>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

OptimizerStatus addPoseOnlyObservationEdges(
    Frame                             *p_frame_inout,
    VertexPose                        *p_poseVertex_in,
    g2o::SparseOptimizer              &optimizer_inout,
    std::vector<EdgeMonoOnlyPose *>   &edgesMonos_inout,
    std::vector<EdgeStereoOnlyPose *> &edgesStereos_inout,
    std::vector<size_t>               &monoEdgeIndices_inout,
    std::vector<size_t>               &stereoEdgeIndices_inout,
    int                               &initialMonoCorrespondenceCount_inout,
    int                               &initialStereoCorrespondenceCount_inout)
{
    const int  N             = p_frame_inout->keyPointCount;
    const int  leftCount     = p_frame_inout->leftKeyPointCount;
    const bool isRightCamera = (leftCount != -1);

    edgesMonos_inout.reserve(N);
    edgesStereos_inout.reserve(N);
    monoEdgeIndices_inout.reserve(N);
    stereoEdgeIndices_inout.reserve(N);

    const float thresholdHuberMono   = std::sqrt(5.991);
    const float thresholdHuberStereo = std::sqrt(7.815);

    {
        std::unique_lock<std::mutex> lock(MapPoint::globalMutex);

        for (int keyPointIndex = 0; keyPointIndex < N; keyPointIndex++)
        {
            MapPoint *p_mapPoint = p_frame_inout->mapPoints[keyPointIndex];
            if (p_mapPoint)
            {
                cv::KeyPoint keyPointUn;
                // Left monocular observation
                if ((!isRightCamera &&
                     p_frame_inout->uRight[keyPointIndex] < 0) ||
                    keyPointIndex < leftCount)
                {
                    if (keyPointIndex < leftCount) // pair left-right
                        keyPointUn = p_frame_inout->keyPoints[keyPointIndex];
                    else
                        keyPointUn =
                            p_frame_inout->keyPointsUndistorted[keyPointIndex];

                    initialMonoCorrespondenceCount_inout++;
                    p_frame_inout->outlierFlags[keyPointIndex] = false;

                    Eigen::Matrix<double, 2, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y;

                    Eigen::Vector3f mapPointWorldPos{};
                    if (p_mapPoint->getWorldPos(mapPointWorldPos) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    EdgeMonoOnlyPose *e =
                        new EdgeMonoOnlyPose(mapPointWorldPos, 0);

                    e->setVertex(0, p_poseVertex_in);
                    e->setMeasurement(observation);

                    // Add here uncerteinty
                    const float unc2 =
                        p_frame_inout->p_camera->uncertainty2(observation);

                    const float invSigma2 =
                        p_frame_inout->invLevelSigmaSquared[keyPointUn.octave] /
                        unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberMono);

                    optimizer_inout.addEdge(e);

                    edgesMonos_inout.push_back(e);
                    monoEdgeIndices_inout.push_back(keyPointIndex);
                }
                // Stereo observation
                else if (!isRightCamera)
                {
                    initialStereoCorrespondenceCount_inout++;
                    p_frame_inout->outlierFlags[keyPointIndex] = false;

                    keyPointUn =
                        p_frame_inout->keyPointsUndistorted[keyPointIndex];
                    const float rightKeyPointU =
                        p_frame_inout->uRight[keyPointIndex];
                    Eigen::Matrix<double, 3, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y,
                        rightKeyPointU;

                    Eigen::Vector3f mapPointWorldPos2{};
                    if (p_mapPoint->getWorldPos(mapPointWorldPos2) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    EdgeStereoOnlyPose *e =
                        new EdgeStereoOnlyPose(mapPointWorldPos2);

                    e->setVertex(0, p_poseVertex_in);
                    e->setMeasurement(observation);

                    // Add here uncerteinty
                    const float unc2 = p_frame_inout->p_camera->uncertainty2(
                        observation.head(2));

                    const float &invSigma2 =
                        p_frame_inout->invLevelSigmaSquared[keyPointUn.octave] /
                        unc2;
                    e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberStereo);

                    optimizer_inout.addEdge(e);

                    edgesStereos_inout.push_back(e);
                    stereoEdgeIndices_inout.push_back(keyPointIndex);
                }

                // Right monocular observation
                if (isRightCamera && keyPointIndex >= leftCount)
                {
                    initialMonoCorrespondenceCount_inout++;
                    p_frame_inout->outlierFlags[keyPointIndex] = false;

                    keyPointUn =
                        p_frame_inout
                            ->keyPointsRight[keyPointIndex - leftCount];
                    Eigen::Matrix<double, 2, 1> observation;
                    observation << keyPointUn.pt.x, keyPointUn.pt.y;

                    Eigen::Vector3f mapPointWorldPos3{};
                    if (p_mapPoint->getWorldPos(mapPointWorldPos3) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWorldPos returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    EdgeMonoOnlyPose *e =
                        new EdgeMonoOnlyPose(mapPointWorldPos3, 1);

                    e->setVertex(0, p_poseVertex_in);
                    e->setMeasurement(observation);

                    // Add here uncerteinty
                    const float unc2 =
                        p_frame_inout->p_camera->uncertainty2(observation);

                    const float invSigma2 =
                        p_frame_inout->invLevelSigmaSquared[keyPointUn.octave] /
                        unc2;
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *p_robustKernel =
                        new g2o::RobustKernelHuber;
                    e->setRobustKernel(p_robustKernel);
                    p_robustKernel->setDelta(thresholdHuberMono);

                    optimizer_inout.addEdge(e);

                    edgesMonos_inout.push_back(e);
                    monoEdgeIndices_inout.push_back(keyPointIndex);
                }
            }
        }
    }

    return OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
