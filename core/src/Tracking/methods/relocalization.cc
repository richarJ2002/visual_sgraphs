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
 * @file            relocalization.cc
 *
 * @brief           Implements Tracking::relocalization(), declared in
 *                  Tracking.h.
 */

#include "Tracking.h"

#include "KeyFrameDatabase.h"
#include "MLPnPsolver.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "System.h"

#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::relocalization(bool &isRelocalized_out)
{
    if (Verbose::printMess("Starting relocalization",
                           Verbose::VERBOSITY_NORMAL) !=
        VerboseStatus::VERBOSE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: printMess returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    // Compute Bag of Words Vector
    if (currentFrame.computeBagOfWords() != FrameStatus::FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeBagOfWords returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    // STRUCTURAL PRIORS: Use room centroids from S-Graph to guide
    // relocalization In office corridors, room/passage markers provide strong
    // topological priors
    std::vector<Eigen::Vector3f>  roomCentroids;
    std::vector<semantic::Room *> currentRooms;
    Map                          *p_currentMap = nullptr;
    if (p_atlas->getCurrentMap(p_currentMap) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_currentMap)
    {
        std::vector<semantic::Room *> rooms{};
        if (p_currentMap->getAllDetectedMapRooms(rooms) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllDetectedMapRooms returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Room *p_room : rooms)
        {
            bool roomIsBad{};
            if (p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room::BoundaryStatus roomBoundaryStatus{};
            if ((!roomIsBad) && p_room->getBoundaryStatus(roomBoundaryStatus) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getBoundaryStatus returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (!roomIsBad &&
                roomBoundaryStatus == semantic::Room::BoundaryStatus::COMPLETE)
            {
                Eigen::Vector3d roomCentroid{};
                if (p_room->getCentroid(roomCentroid) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                roomCentroids.push_back(Eigen::Vector3f(roomCentroid.x(),
                                                        roomCentroid.y(),
                                                        roomCentroid.z()));
                currentRooms.push_back(p_room);
            }
        }
    }

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for
    // relocalisation
    Map *p_atlasCurrentMap = nullptr;
    if (p_atlas->getCurrentMap(p_atlasCurrentMap) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<KeyFrame *> candidateKeyFrames{};
    if (p_keyFrameDatabase->detectRelocalizationCandidates(
            &currentFrame,
            p_atlasCurrentMap,
            candidateKeyFrames) !=
        KeyFrameDatabaseStatus::KEY_FRAME_DATABASE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: detectRelocalizationCandidates returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }

    if (candidateKeyFrames.empty())
    {
        if (Verbose::printMess("There are not candidates",
                               Verbose::VERBOSITY_NORMAL) !=
            VerboseStatus::VERBOSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: printMess returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        isRelocalized_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }

    const int keyFrameCount = candidateKeyFrames.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    std::vector<MLPnPsolver *> pnpSolvers;
    pnpSolvers.resize(keyFrameCount);

    std::vector<std::vector<MapPoint *>> vvpMapPointMatches;
    vvpMapPointMatches.resize(keyFrameCount);

    std::vector<bool> discardedFlags;
    discardedFlags.resize(keyFrameCount);

    int candidateCount = 0;

    for (int keyFrameIndex = 0; keyFrameIndex < keyFrameCount; keyFrameIndex++)
    {
        KeyFrame *p_keyFrame = candidateKeyFrames[keyFrameIndex];
        bool      keyFrameIsBad{};
        if (p_keyFrame->isBad(keyFrameIsBad) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (keyFrameIsBad)
            discardedFlags[keyFrameIndex] = true;
        else
        {
            int nmatches{};
            if (matcher.searchByBoW(p_keyFrame,
                                    currentFrame,
                                    vvpMapPointMatches[keyFrameIndex],
                                    nmatches) !=
                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: searchByBoW returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (nmatches < 15)
            {
                discardedFlags[keyFrameIndex] = true;
                continue;
            }
            else
            {
                MLPnPsolver *p_solver =
                    new MLPnPsolver(currentFrame,
                                    vvpMapPointMatches[keyFrameIndex]);
                if (p_solver
                        ->setRansacParameters(0.99, 10, 300, 6, 0.5, 5.991) !=
                    MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setRansacParameters returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                } // This solver needs at least 6 points
                pnpSolvers[keyFrameIndex] = p_solver;
                candidateCount++;
            }
        }
    }

    // STRUCTURAL PRIOR: Re-rank candidates by proximity to room centroids
    // This helps in repetitive corridors where visual appearance is similar
    if (!roomCentroids.empty() && !candidateKeyFrames.empty())
    {
        // Score candidates by: visual matches + proximity to known room
        // centroids
        std::vector<float> candidateScores(keyFrameCount, 0.0f);
        for (int keyFrameIndex = 0; keyFrameIndex < keyFrameCount;
             keyFrameIndex++)
        {
            if (discardedFlags[keyFrameIndex])
                continue;

            KeyFrame    *p_keyFrame = candidateKeyFrames[keyFrameIndex];
            Sophus::SE3f keyFramePose{};
            if (p_keyFrame->getPose(keyFramePose) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPose returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f keyFramePosition =
                keyFramePose.translation().head<3>();

            // Visual match score (normalized)
            int nmatches = vvpMapPointMatches[keyFrameIndex].size();
            candidateScores[keyFrameIndex] = nmatches * 1.0f;

            // Structural prior: proximity to room centroids
            for (size_t r = 0; r < roomCentroids.size(); r++)
            {
                float distance = (keyFramePosition - roomCentroids[r]).norm();
                // Boost score if KF is near a known room centroid (within 3m)
                if (distance < 3.0f)
                    candidateScores[keyFrameIndex] +=
                        (3.0f - distance) * 2.0f; // Max boost of 6
            }
        }

        // Re-sort candidates by combined score (highest first)
        std::vector<int> sortedIndices(keyFrameCount);
        for (int keyFrameIndex = 0; keyFrameIndex < keyFrameCount;
             keyFrameIndex++)
            sortedIndices[keyFrameIndex] = keyFrameIndex;
        std::sort(sortedIndices.begin(),
                  sortedIndices.end(),
                  [&](int a, int b)
                  { return candidateScores[a] > candidateScores[b]; });

        // Reorder vectors for processing
        std::vector<KeyFrame *> reorderedKeyFrames = candidateKeyFrames;
        std::vector<std::vector<MapPoint *>> reorderedMatches =
            vvpMapPointMatches;
        std::vector<MLPnPsolver *> reorderedSolvers   = pnpSolvers;
        std::vector<bool>          reorderedDiscarded = discardedFlags;

        for (int keyFrameIndex = 0; keyFrameIndex < keyFrameCount;
             keyFrameIndex++)
        {
            candidateKeyFrames[keyFrameIndex] =
                reorderedKeyFrames[sortedIndices[keyFrameIndex]];
            vvpMapPointMatches[keyFrameIndex] =
                reorderedMatches[sortedIndices[keyFrameIndex]];
            pnpSolvers[keyFrameIndex] =
                reorderedSolvers[sortedIndices[keyFrameIndex]];
            discardedFlags[keyFrameIndex] =
                reorderedDiscarded[sortedIndices[keyFrameIndex]];
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool       isMatched = false;
    ORBmatcher matcher2(0.9, true);

    while (candidateCount > 0 && !isMatched)
    {
        for (int keyFrameIndex = 0; keyFrameIndex < keyFrameCount;
             keyFrameIndex++)
        {
            if (discardedFlags[keyFrameIndex])
                continue;

            // Perform 5 Ransac Iterations
            std::vector<bool> inliersFlags;
            int               inlierCount;
            bool              areIterationsExhausted;

            MLPnPsolver    *p_solver = pnpSolvers[keyFrameIndex];
            Eigen::Matrix4f poseEigen_worldToCamera;
            bool            bTcw{};
            if (p_solver->iterate(5,
                                  areIterationsExhausted,
                                  inliersFlags,
                                  inlierCount,
                                  poseEigen_worldToCamera,
                                  bTcw) !=
                MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: iterate returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            // If Ransac reachs max. iterations discard keyframe
            if (areIterationsExhausted)
            {
                discardedFlags[keyFrameIndex] = true;
                candidateCount--;
            }

            // If a Camera Pose is computed, optimize
            if (bTcw)
            {
                Sophus::SE3f pose_worldToCamera(poseEigen_worldToCamera);
                if (currentFrame.setPose(pose_worldToCamera) !=
                    FrameStatus::FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setPose returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                // Tcw.copyTo(mCurrentFrame.poseTcw);

                std::set<MapPoint *> founds;

                const int np = inliersFlags.size();

                for (int j = 0; j < np; j++)
                {
                    if (inliersFlags[j])
                    {
                        currentFrame.mapPoints[j] =
                            vvpMapPointMatches[keyFrameIndex][j];
                        founds.insert(vvpMapPointMatches[keyFrameIndex][j]);
                    }
                    else
                        currentFrame.mapPoints[j] = nullptr;
                }

                int goodCount{};
                if (Optimizer::poseOptimization(&currentFrame, goodCount) !=
                    OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: poseOptimization returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }

                if (goodCount < 10)
                    continue;

                for (int io = 0; io < currentFrame.keyPointCount; io++)
                    if (currentFrame.outlierFlags[io])
                        currentFrame.mapPoints[io] =
                            static_cast<MapPoint *>(nullptr);

                // If few inliers, search by projection in a coarse window and
                // optimize again
                if (goodCount < 50)
                {
                    int nadditional{};
                    if (matcher2.searchByProjection(
                            currentFrame,
                            candidateKeyFrames[keyFrameIndex],
                            founds,
                            10,
                            100,
                            nadditional) !=
                        ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: searchByProjection returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    if (nadditional + goodCount >= 50)
                    {
                        int inlierCount2{};
                        if (Optimizer::poseOptimization(&currentFrame,
                                                        inlierCount2) !=
                            OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: poseOptimization returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        goodCount = inlierCount2;

                        // If many inliers but still not enough, search by
                        // projection again in a narrower window the camera has
                        // been already optimized with many points
                        if (goodCount > 30 && goodCount < 50)
                        {
                            founds.clear();
                            for (int ip = 0; ip < currentFrame.keyPointCount;
                                 ip++)
                                if (currentFrame.mapPoints[ip])
                                    founds.insert(currentFrame.mapPoints[ip]);
                            int matcher2ByProjection{};
                            if (matcher2.searchByProjection(
                                    currentFrame,
                                    candidateKeyFrames[keyFrameIndex],
                                    founds,
                                    3,
                                    64,
                                    matcher2ByProjection) !=
                                ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: searchByProjection returned a failure "
                                    "status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            nadditional = matcher2ByProjection;

                            // Final optimization
                            if (goodCount + nadditional >= 50)
                            {
                                int inlierCount3{};
                                if (Optimizer::poseOptimization(&currentFrame,
                                                                inlierCount3) !=
                                    OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: poseOptimization returned a "
                                        "failure status although it cannot "
                                        "fail; continuing as before.",
                                        __func__);
                                }
                                goodCount = inlierCount3;

                                for (int io = 0;
                                     io < currentFrame.keyPointCount;
                                     io++)
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
                if (goodCount >= relocalizationMinInliers)
                {
                    isMatched = true;
                    break;
                }
            }
        }
    }

    if (!isMatched)
    {
        isRelocalized_out = false;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }
    else
    {
        lastRelocFrameId = currentFrame.id;
        std::cout << "[Tracking] Relocalized!" << std::endl;
        isRelocalized_out = true;
        return TrackingStatus::TRACKING_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
