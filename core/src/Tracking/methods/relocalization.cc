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

#include "MLPnPsolver.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{

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

} // namespace core
} // namespace vs_graphs
