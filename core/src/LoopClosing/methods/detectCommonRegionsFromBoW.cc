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

#include "LoopClosing.h"

#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Sim3Solver.h"
#include "System.h"
#include "Tracking.h"
#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{

bool LoopClosing::detectCommonRegionsFromBoW(
    std::vector<KeyFrame *> &bowCandidates_in,
    KeyFrame               *&matchedKeyFrame_out,
    KeyFrame               *&lastCurrentKeyFrame_out,
    g2o::Sim3               &g2oScw_out,
    int                     &countCoincidenceCount_out,
    std::vector<MapPoint *> &mapPoints_out,
    std::vector<MapPoint *> &matchedMapPoints_out)
{
    int bowMatchCount           = 20;
    int bowInlierCount          = 15;
    int nSim3Inliers            = 20;
    int projectionMatchCount    = 50;
    int projectionOptMatchCount = 80;

    set<KeyFrame *> connectedKeyFrames = p_currentKF->getConnectedKeyFrames();

    int countCovisibleCount = 10;

    ORBmatcher matcherBow(0.9, true);
    ORBmatcher matcher(0.75, true);

    // Varibles to select the best numbe
    KeyFrame               *p_bestMatchedKeyFrame;
    int                     bestMatchesReprojCount    = 0;
    int                     bestCountCoindicendeCount = 0;
    g2o::Sim3               g2oBestScw;
    std::vector<MapPoint *> bestMapPoints;
    std::vector<MapPoint *> bestMatchedMapPoints;

    int         candidateCount = bowCandidates_in.size();
    vector<int> stageCounts(candidateCount, 0);
    vector<int> matchStageCounts(candidateCount, 0);

    int index = 0;
    // Verbose::PrintMess("BoW candidates: There are " +
    // to_string(vpBowCand.size()) + " possible candidates ",
    // Verbose::VERBOSITY_DEBUG);
    for (KeyFrame *p_keyFrame : bowCandidates_in)
    {
        if (!p_keyFrame || p_keyFrame->isBad())
            continue;

        // std::cout << "KF candidate: " << pKFi->id << std::endl;
        // Current KF against KF with covisibles version
        std::vector<KeyFrame *> covisibleKeyFrames =
            p_keyFrame->getBestCovisibilityKeyFrames(countCovisibleCount);
        if (covisibleKeyFrames.empty())
        {
            // std::cout << "Covisible list empty" << std::endl;
            covisibleKeyFrames.push_back(p_keyFrame);
        }
        else
        {
            covisibleKeyFrames.push_back(covisibleKeyFrames[0]);
            covisibleKeyFrames[0] = p_keyFrame;
        }

        bool isAbortedByNearKeyFrame = false;
        for (int covisibleKeyFrameIndex = 0;
             covisibleKeyFrameIndex < covisibleKeyFrames.size();
             ++covisibleKeyFrameIndex)
        {
            if (connectedKeyFrames.find(
                    covisibleKeyFrames[covisibleKeyFrameIndex]) !=
                connectedKeyFrames.end())
            {
                isAbortedByNearKeyFrame = true;
                break;
            }
        }
        if (isAbortedByNearKeyFrame)
        {
            // std::cout << "Check BoW aborted because is close to the matched
            // one " << std::endl;
            continue;
        }
        // std::cout << "Check BoW continue because is far to the matched one "
        // << std::endl;

        std::vector<std::vector<MapPoint *>> vvpMatchedMapPoints;
        vvpMatchedMapPoints.resize(covisibleKeyFrames.size());
        std::set<MapPoint *> matchedMapPoints;
        int                  numBoWMatches = 0;

        KeyFrame *p_mostBowMatchesKeyFrame = p_keyFrame;
        int       mostBowCountMatchCount   = 0;

        std::vector<MapPoint *> matchedPoints =
            std::vector<MapPoint *>(p_currentKF->getMapPointMatches().size(),
                                    static_cast<MapPoint *>(nullptr));
        std::vector<KeyFrame *> keyFrameMatchedMapPoints =
            std::vector<KeyFrame *>(p_currentKF->getMapPointMatches().size(),
                                    static_cast<KeyFrame *>(nullptr));

        int indexMostBowMatchesKeyFrameCount = 0;
        for (int covisibleKeyFrameIndex = 0;
             covisibleKeyFrameIndex < covisibleKeyFrames.size();
             ++covisibleKeyFrameIndex)
        {
            if (!covisibleKeyFrames[covisibleKeyFrameIndex] ||
                covisibleKeyFrames[covisibleKeyFrameIndex]->isBad())
                continue;

            int count = matcherBow.searchByBoW(
                p_currentKF,
                covisibleKeyFrames[covisibleKeyFrameIndex],
                vvpMatchedMapPoints[covisibleKeyFrameIndex]);
            if (count > mostBowCountMatchCount)
            {
                mostBowCountMatchCount           = count;
                indexMostBowMatchesKeyFrameCount = covisibleKeyFrameIndex;
            }
        }

        for (int covisibleKeyFrameIndex = 0;
             covisibleKeyFrameIndex < covisibleKeyFrames.size();
             ++covisibleKeyFrameIndex)
        {
            for (int matchIndex = 0;
                 matchIndex <
                 vvpMatchedMapPoints[covisibleKeyFrameIndex].size();
                 ++matchIndex)
            {
                MapPoint *p_matchedMapPoint =
                    vvpMatchedMapPoints[covisibleKeyFrameIndex][matchIndex];
                if (!p_matchedMapPoint || p_matchedMapPoint->isBad())
                    continue;

                if (matchedMapPoints.find(p_matchedMapPoint) ==
                    matchedMapPoints.end())
                {
                    matchedMapPoints.insert(p_matchedMapPoint);
                    numBoWMatches++;

                    matchedPoints[matchIndex] = p_matchedMapPoint;
                    keyFrameMatchedMapPoints[matchIndex] =
                        covisibleKeyFrames[covisibleKeyFrameIndex];
                }
            }
        }

        // pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];

        if (numBoWMatches >= bowMatchCount) // TODO pick a good threshold
        {
            // Geometric validation
            bool isFixedScale = isScaleFixed;
            if (p_tracker->sensor == System::IMU_MONOCULAR &&
                !p_currentKF->getMap()->getInertialBA2())
                isFixedScale = false;

            Sim3Solver solver = Sim3Solver(p_currentKF,
                                           p_mostBowMatchesKeyFrame,
                                           matchedPoints,
                                           isFixedScale,
                                           keyFrameMatchedMapPoints);
            solver.setRansacParameters(0.99,
                                       bowInlierCount,
                                       300); // at least 15 inliers

            bool            areIterationsExhausted = false;
            vector<bool>    inliersFlags;
            int             inlierCount;
            bool            hasConverged = false;
            Eigen::Matrix4f mTcm;
            while (!hasConverged && !areIterationsExhausted)
            {
                mTcm = solver.iterate(20,
                                      areIterationsExhausted,
                                      inliersFlags,
                                      inlierCount,
                                      hasConverged);
                // Verbose::PrintMess("BoW guess: Solver achieve " +
                // to_string(nInliers) + " geometrical inliers among " +
                // to_string(nBoWInliers) + " BoW matches",
                // Verbose::VERBOSITY_DEBUG);
            }

            if (hasConverged)
            {
                // std::cout << "Check BoW: SolverSim3 converged" << std::endl;

                // Verbose::PrintMess("BoW guess: Convergende with " +
                // to_string(nInliers) + " geometrical inliers among " +
                // to_string(nBoWInliers) + " BoW matches",
                // Verbose::VERBOSITY_DEBUG);
                //  Match by reprojection
                covisibleKeyFrames.clear();
                covisibleKeyFrames =
                    p_mostBowMatchesKeyFrame->getBestCovisibilityKeyFrames(
                        countCovisibleCount);
                covisibleKeyFrames.push_back(p_mostBowMatchesKeyFrame);
                set<KeyFrame *> checkKeyFrames(covisibleKeyFrames.begin(),
                                               covisibleKeyFrames.end());

                // std::cout << "There are " << vpCovKFi.size() <<" near KFs" <<
                // std::endl;

                set<MapPoint *>    mapPoints;
                vector<MapPoint *> candidateMapPoints;
                vector<KeyFrame *> keyFrames;
                for (KeyFrame *p_covisibleKeyFrame : covisibleKeyFrames)
                {
                    for (MapPoint *p_covisibleMapPoint :
                         p_covisibleKeyFrame->getMapPointMatches())
                    {
                        if (!p_covisibleMapPoint ||
                            p_covisibleMapPoint->isBad())
                            continue;

                        if (mapPoints.find(p_covisibleMapPoint) ==
                            mapPoints.end())
                        {
                            mapPoints.insert(p_covisibleMapPoint);
                            candidateMapPoints.push_back(p_covisibleMapPoint);
                            keyFrames.push_back(p_covisibleKeyFrame);
                        }
                    }
                }

                // std::cout << "There are " << vpKeyFrames.size() <<" KFs which
                // view all the mappoints" << std::endl;

                g2o::Sim3 gScm(solver.getEstimatedRotation().cast<double>(),
                               solver.getEstimatedTranslation().cast<double>(),
                               (double)solver.getEstimatedScale());
                g2o::Sim3 gSmw(
                    p_mostBowMatchesKeyFrame->getRotation().cast<double>(),
                    p_mostBowMatchesKeyFrame->getTranslation().cast<double>(),
                    1.0);
                g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current
                                              // from the world position
                Sophus::Sim3f correctedPose{};
                if (utils::converter::Converter::toSophus(gScw,
                                                          correctedPose) !=
                    utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
                {
                    // toSophus cannot fail; continue as before.
                }

                vector<MapPoint *> bowMatchedMapPoints;
                bowMatchedMapPoints.resize(
                    p_currentKF->getMapPointMatches().size(),
                    static_cast<MapPoint *>(nullptr));
                vector<KeyFrame *> matchedKeyFrames;
                matchedKeyFrames.resize(
                    p_currentKF->getMapPointMatches().size(),
                    static_cast<KeyFrame *>(nullptr));
                int numProjMatches =
                    matcher.searchByProjection(p_currentKF,
                                               correctedPose,
                                               candidateMapPoints,
                                               keyFrames,
                                               bowMatchedMapPoints,
                                               matchedKeyFrames,
                                               8,
                                               1.5);
                // cout <<"BoW: " << numProjMatches << " matches between " <<
                // vpMapPoints.size() << " points with coarse Sim3" << endl;

                if (numProjMatches >= projectionMatchCount)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> hessian7x7;

                    bool isFixedScale = isScaleFixed;
                    if (p_tracker->sensor == System::IMU_MONOCULAR &&
                        !p_currentKF->getMap()->getInertialBA2())
                        isFixedScale = false;

                    int optMatchCount =
                        Optimizer::optimizeSim3(p_currentKF,
                                                p_keyFrame,
                                                bowMatchedMapPoints,
                                                gScm,
                                                10,
                                                isScaleFixed,
                                                hessian7x7,
                                                true);

                    if (optMatchCount >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(
                            p_mostBowMatchesKeyFrame->getRotation()
                                .cast<double>(),
                            p_mostBowMatchesKeyFrame->getTranslation()
                                .cast<double>(),
                            1.0);
                        g2o::Sim3 gScw =
                            gScm * gSmw; // Similarity matrix of current from
                                         // the world position
                        Sophus::Sim3f correctedPose{};
                        if (utils::converter::Converter::toSophus(
                                gScw,
                                correctedPose) !=
                            utils::converter::ConverterStatus::
                                CONVERTER_STATUS_SUCCESS)
                        {
                            // toSophus cannot fail; continue as before.
                        }

                        vector<MapPoint *> bowMatchedMapPoints;
                        bowMatchedMapPoints.resize(
                            p_currentKF->getMapPointMatches().size(),
                            static_cast<MapPoint *>(nullptr));
                        int optimizedProjectionMatchCount =
                            matcher.searchByProjection(p_currentKF,
                                                       correctedPose,
                                                       candidateMapPoints,
                                                       bowMatchedMapPoints,
                                                       5,
                                                       1.0);

                        if (optimizedProjectionMatchCount >=
                            projectionOptMatchCount)
                        {
                            int maximumX = -1, minimumX = 1000000;
                            int maximumY = -1, minimumY = 1000000;
                            for (MapPoint *p_mapPoint : bowMatchedMapPoints)
                            {
                                if (!p_mapPoint || p_mapPoint->isBad())
                                {
                                    continue;
                                }

                                tuple<size_t, size_t> indexes =
                                    p_mapPoint->getIndexInKeyFrame(p_keyFrame);
                                int index = get<0>(indexes);
                                if (index >= 0)
                                {
                                    int coordinateX =
                                        p_keyFrame->keyPointsUndistorted[index]
                                            .pt.x;
                                    if (coordinateX < minimumX)
                                    {
                                        minimumX = coordinateX;
                                    }
                                    if (coordinateX > maximumX)
                                    {
                                        maximumX = coordinateX;
                                    }
                                    int coordinateY =
                                        p_keyFrame->keyPointsUndistorted[index]
                                            .pt.y;
                                    if (coordinateY < minimumY)
                                    {
                                        minimumY = coordinateY;
                                    }
                                    if (coordinateY > maximumY)
                                    {
                                        maximumY = coordinateY;
                                    }
                                }
                            }

                            int                countKeyFrameCount = 0;
                            // vpMatchedMPs = vpMatchedMP;
                            // vpMPs = vpMapPoints;
                            //  Check the Sim3 transformation with the current
                            //  KeyFrame covisibles
                            vector<KeyFrame *> currentCovisibleKeyFrames =
                                p_currentKF->getBestCovisibilityKeyFrames(
                                    countCovisibleCount);

                            int covisibleKeyFrameIndex = 0;
                            while (countKeyFrameCount < 3 &&
                                   covisibleKeyFrameIndex <
                                       currentCovisibleKeyFrames.size())
                            {
                                KeyFrame *p_currentCovisibleKeyFrame =
                                    currentCovisibleKeyFrames
                                        [covisibleKeyFrameIndex];
                                Sophus::SE3d mTjc =
                                    (p_currentCovisibleKeyFrame->getPose() *
                                     p_currentKF->getPoseInverse())
                                        .cast<double>();
                                g2o::Sim3 gSjc(mTjc.unit_quaternion(),
                                               mTjc.translation(),
                                               1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int       covisibleProjectionMatchCount = 0;
                                vector<MapPoint *> covisibleMatchedMapPoints;
                                bool isValid = detectCommonRegionsFromLastKF(
                                    p_currentCovisibleKeyFrame,
                                    p_mostBowMatchesKeyFrame,
                                    gSjw,
                                    covisibleProjectionMatchCount,
                                    candidateMapPoints,
                                    covisibleMatchedMapPoints);

                                if (isValid)
                                {
                                    Sophus::SE3f Tc_w = p_currentKF->getPose();
                                    Sophus::SE3f Tw_cj =
                                        p_currentCovisibleKeyFrame
                                            ->getPoseInverse();
                                    Sophus::SE3f    Tc_cj = Tc_w * Tw_cj;
                                    Eigen::Vector3f vectorDistance =
                                        Tc_cj.translation();
                                    countKeyFrameCount++;
                                }
                                covisibleKeyFrameIndex++;
                            }

                            if (countKeyFrameCount < 3)
                            {
                                stageCounts[index]      = 8;
                                matchStageCounts[index] = countKeyFrameCount;
                            }

                            if (bestMatchesReprojCount <
                                optimizedProjectionMatchCount)
                            {
                                bestMatchesReprojCount =
                                    optimizedProjectionMatchCount;
                                bestCountCoindicendeCount = countKeyFrameCount;
                                p_bestMatchedKeyFrame =
                                    p_mostBowMatchesKeyFrame;
                                g2oBestScw           = gScw;
                                bestMapPoints        = candidateMapPoints;
                                bestMatchedMapPoints = bowMatchedMapPoints;
                            }
                        }
                    }
                }
            }
        }
        index++;
    }

    if (bestMatchesReprojCount > 0)
    {
        lastCurrentKeyFrame_out   = p_currentKF;
        countCoincidenceCount_out = bestCountCoindicendeCount;
        matchedKeyFrame_out       = p_bestMatchedKeyFrame;
        matchedKeyFrame_out->setNotErase();
        g2oScw_out           = g2oBestScw;
        mapPoints_out        = bestMapPoints;
        matchedMapPoints_out = bestMatchedMapPoints;

        return countCoincidenceCount_out >= 3;
    }
    else
    {
        int maximumStage = -1;
        int maximumMatched;
        for (int stageCountIndex = 0; stageCountIndex < stageCounts.size();
             ++stageCountIndex)
        {
            if (stageCounts[stageCountIndex] > maximumStage)
            {
                maximumStage   = stageCounts[stageCountIndex];
                maximumMatched = matchStageCounts[stageCountIndex];
            }
        }
    }
    return false;
}

} // namespace core
} // namespace vs_graphs
