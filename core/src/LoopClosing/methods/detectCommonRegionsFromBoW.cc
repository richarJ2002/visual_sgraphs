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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

LoopClosingStatus LoopClosing::detectCommonRegionsFromBoW(
    std::vector<KeyFrame *> &bowCandidates_in,
    KeyFrame               *&matchedKeyFrame_out,
    KeyFrame               *&lastCurrentKeyFrame_out,
    g2o::Sim3               &g2oScw_out,
    int                     &countCoincidenceCount_out,
    std::vector<MapPoint *> &mapPoints_out,
    std::vector<MapPoint *> &matchedMapPoints_out,
    bool                    &isDetected_out)
{
    int bowMatchCount           = 20;
    int bowInlierCount          = 15;
    int nSim3Inliers            = 20;
    int projectionMatchCount    = 50;
    int projectionOptMatchCount = 80;

    std::set<KeyFrame *> connectedKeyFrames{};
    if (p_currentKF->getConnectedKeyFrames(connectedKeyFrames) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getConnectedKeyFrames returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

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

    int              candidateCount = bowCandidates_in.size();
    std::vector<int> stageCounts(candidateCount, 0);
    std::vector<int> matchStageCounts(candidateCount, 0);

    int index = 0;
    // Verbose::PrintMess("BoW candidates: There are " +
    // to_string(vpBowCand.size()) + " possible candidates ",
    // Verbose::VERBOSITY_DEBUG);
    for (KeyFrame *p_keyFrame : bowCandidates_in)
    {
        bool keyFrameIsBad{};
        if (!(!p_keyFrame) && p_keyFrame->isBad(keyFrameIsBad) !=
                                  KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (!p_keyFrame || keyFrameIsBad)
            continue;

        // std::cout << "KF candidate: " << pKFi->id << std::endl;
        // Current KF against KF with covisibles version
        std::vector<KeyFrame *> covisibleKeyFrames{};
        if (p_keyFrame->getBestCovisibilityKeyFrames(countCovisibleCount,
                                                     covisibleKeyFrames) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: getBestCovisibilityKeyFrames returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
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

        std::vector<MapPoint *> currentKFMapPointMatches{};
        if (p_currentKF->getMapPointMatches(currentKFMapPointMatches) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<MapPoint *> matchedPoints =
            std::vector<MapPoint *>(currentKFMapPointMatches.size(),
                                    static_cast<MapPoint *>(nullptr));
        std::vector<MapPoint *> currentKFMapPointMatches2{};
        if (p_currentKF->getMapPointMatches(currentKFMapPointMatches2) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<KeyFrame *> keyFrameMatchedMapPoints =
            std::vector<KeyFrame *>(currentKFMapPointMatches2.size(),
                                    static_cast<KeyFrame *>(nullptr));

        int indexMostBowMatchesKeyFrameCount = 0;
        for (int covisibleKeyFrameIndex = 0;
             covisibleKeyFrameIndex < covisibleKeyFrames.size();
             ++covisibleKeyFrameIndex)
        {
            bool isBad2{};
            if (!(!covisibleKeyFrames[covisibleKeyFrameIndex]) &&
                covisibleKeyFrames[covisibleKeyFrameIndex]->isBad(isBad2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!covisibleKeyFrames[covisibleKeyFrameIndex] || isBad2)
                continue;

            int count{};
            if (matcherBow.searchByBoW(
                    p_currentKF,
                    covisibleKeyFrames[covisibleKeyFrameIndex],
                    vvpMatchedMapPoints[covisibleKeyFrameIndex],
                    count) != ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: searchByBoW returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
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
                bool matchedMapPointIsBad{};
                if (!(!p_matchedMapPoint) &&
                    p_matchedMapPoint->isBad(matchedMapPointIsBad) !=
                        MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (!p_matchedMapPoint || matchedMapPointIsBad)
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
            bool isFixedScale   = isScaleFixed;
            Map *p_currentKFMap = nullptr;
            if ((p_tracker->sensor == System::IMU_MONOCULAR) &&
                p_currentKF->getMap(p_currentKFMap) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            bool inertialBA2{};
            if ((p_tracker->sensor == System::IMU_MONOCULAR) &&
                p_currentKFMap->getInertialBA2(inertialBA2) !=
                    MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getInertialBA2 returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_tracker->sensor == System::IMU_MONOCULAR && !inertialBA2)
                isFixedScale = false;

            Sim3Solver solver = Sim3Solver(p_currentKF,
                                           p_mostBowMatchesKeyFrame,
                                           matchedPoints,
                                           isFixedScale,
                                           keyFrameMatchedMapPoints);
            if (solver.setRansacParameters(0.99, bowInlierCount, 300) !=
                Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: setRansacParameters returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            } // at least 15 inliers

            bool              areIterationsExhausted = false;
            std::vector<bool> inliersFlags;
            int               inlierCount;
            bool              hasConverged = false;
            Eigen::Matrix4f   mTcm;
            while (!hasConverged && !areIterationsExhausted)
            {
                Eigen::Matrix4f solverTransform{};
                if (solver.iterate(20,
                                   areIterationsExhausted,
                                   inliersFlags,
                                   inlierCount,
                                   hasConverged,
                                   solverTransform) !=
                    Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: iterate returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                mTcm = solverTransform;
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
                std::vector<KeyFrame *>
                    mostBowMatchesKeyFrameBestCovisibilityKeyFrames{};
                if (p_mostBowMatchesKeyFrame->getBestCovisibilityKeyFrames(
                        countCovisibleCount,
                        mostBowMatchesKeyFrameBestCovisibilityKeyFrames) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getBestCovisibilityKeyFrames returned a failure "
                        "status although it cannot fail; continuing as before.",
                        __func__);
                }
                covisibleKeyFrames =
                    mostBowMatchesKeyFrameBestCovisibilityKeyFrames;
                covisibleKeyFrames.push_back(p_mostBowMatchesKeyFrame);
                std::set<KeyFrame *> checkKeyFrames(covisibleKeyFrames.begin(),
                                                    covisibleKeyFrames.end());

                // std::cout << "There are " << vpCovKFi.size() <<" near KFs" <<
                // std::endl;

                std::set<MapPoint *>    mapPoints;
                std::vector<MapPoint *> candidateMapPoints;
                std::vector<KeyFrame *> keyFrames;
                for (KeyFrame *p_covisibleKeyFrame : covisibleKeyFrames)
                {
                    std::vector<MapPoint *> covisibleKeyFrameMapPointMatches{};
                    if (p_covisibleKeyFrame->getMapPointMatches(
                            covisibleKeyFrameMapPointMatches) !=
                        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMapPointMatches returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    for (MapPoint *p_covisibleMapPoint :
                         covisibleKeyFrameMapPointMatches)
                    {
                        bool covisibleMapPointIsBad{};
                        if (!(!p_covisibleMapPoint) &&
                            p_covisibleMapPoint->isBad(
                                covisibleMapPointIsBad) !=
                                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: isBad returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        if (!p_covisibleMapPoint || covisibleMapPointIsBad)
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

                Eigen::Matrix3f solverEstimatedRotation{};
                if (solver.getEstimatedRotation(solverEstimatedRotation) !=
                    Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getEstimatedRotation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3f solverEstimatedTranslation{};
                if (solver.getEstimatedTranslation(
                        solverEstimatedTranslation) !=
                    Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getEstimatedTranslation returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                float solverEstimatedScale{};
                if (solver.getEstimatedScale(solverEstimatedScale) !=
                    Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getEstimatedScale returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                g2o::Sim3       gScm(solverEstimatedRotation.cast<double>(),
                               solverEstimatedTranslation.cast<double>(),
                               static_cast<double>(solverEstimatedScale));
                Eigen::Matrix3f mostBowMatchesKeyFrameRotation{};
                if (p_mostBowMatchesKeyFrame->getRotation(
                        mostBowMatchesKeyFrameRotation) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRotation returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3f mostBowMatchesKeyFrameTranslation{};
                if (p_mostBowMatchesKeyFrame->getTranslation(
                        mostBowMatchesKeyFrameTranslation) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getTranslation returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                g2o::Sim3 gSmw(mostBowMatchesKeyFrameRotation.cast<double>(),
                               mostBowMatchesKeyFrameTranslation.cast<double>(),
                               1.0);
                g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current
                                              // from the world position
                Sophus::Sim3f correctedPose{};
                if (utils::converter::Converter::toSophus(gScw,
                                                          correctedPose) !=
                    utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: toSophus returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                std::vector<MapPoint *> bowMatchedMapPoints;
                std::vector<MapPoint *> currentKFMapPointMatches3{};
                if (p_currentKF->getMapPointMatches(
                        currentKFMapPointMatches3) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMapPointMatches returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                bowMatchedMapPoints.resize(currentKFMapPointMatches3.size(),
                                           static_cast<MapPoint *>(nullptr));
                std::vector<KeyFrame *> matchedKeyFrames;
                std::vector<MapPoint *> currentKFMapPointMatches4{};
                if (p_currentKF->getMapPointMatches(
                        currentKFMapPointMatches4) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMapPointMatches returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                matchedKeyFrames.resize(currentKFMapPointMatches4.size(),
                                        static_cast<KeyFrame *>(nullptr));
                int numProjMatches{};
                if (matcher.searchByProjection(p_currentKF,
                                               correctedPose,
                                               candidateMapPoints,
                                               keyFrames,
                                               bowMatchedMapPoints,
                                               matchedKeyFrames,
                                               8,
                                               numProjMatches,
                                               1.5) !=
                    ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: searchByProjection returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                // cout <<"BoW: " << numProjMatches << " matches between " <<
                // vpMapPoints.size() << " points with coarse Sim3" << endl;

                if (numProjMatches >= projectionMatchCount)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> hessian7x7;

                    bool isFixedScale    = isScaleFixed;
                    Map *p_currentKFMap2 = nullptr;
                    if ((p_tracker->sensor == System::IMU_MONOCULAR) &&
                        p_currentKF->getMap(p_currentKFMap2) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    bool inertialBA22{};
                    if ((p_tracker->sensor == System::IMU_MONOCULAR) &&
                        p_currentKFMap2->getInertialBA2(inertialBA22) !=
                            MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getInertialBA2 returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_tracker->sensor == System::IMU_MONOCULAR &&
                        !inertialBA22)
                        isFixedScale = false;

                    int optMatchCount{};
                    if (Optimizer::optimizeSim3(p_currentKF,
                                                p_keyFrame,
                                                bowMatchedMapPoints,
                                                gScm,
                                                10,
                                                isScaleFixed,
                                                hessian7x7,
                                                optMatchCount,
                                                true) !=
                        OptimizerStatus::OPTIMIZER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: optimizeSim3 returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    if (optMatchCount >= nSim3Inliers)
                    {
                        Eigen::Matrix3f mostBowMatchesKeyFrameRotation2{};
                        if (p_mostBowMatchesKeyFrame->getRotation(
                                mostBowMatchesKeyFrameRotation2) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getRotation returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        Eigen::Vector3f mostBowMatchesKeyFrameTranslation2{};
                        if (p_mostBowMatchesKeyFrame->getTranslation(
                                mostBowMatchesKeyFrameTranslation2) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getTranslation returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        g2o::Sim3 gSmw(
                            mostBowMatchesKeyFrameRotation2.cast<double>(),
                            mostBowMatchesKeyFrameTranslation2.cast<double>(),
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
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: toSophus returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }

                        std::vector<MapPoint *> bowMatchedMapPoints;
                        std::vector<MapPoint *> currentKFMapPointMatches5{};
                        if (p_currentKF->getMapPointMatches(
                                currentKFMapPointMatches5) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getMapPointMatches returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        bowMatchedMapPoints.resize(
                            currentKFMapPointMatches5.size(),
                            static_cast<MapPoint *>(nullptr));
                        int optimizedProjectionMatchCount{};
                        if (matcher.searchByProjection(
                                p_currentKF,
                                correctedPose,
                                candidateMapPoints,
                                bowMatchedMapPoints,
                                5,
                                optimizedProjectionMatchCount,
                                1.0) !=
                            ORBmatcherStatus::ORBMATCHER_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: searchByProjection returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }

                        if (optimizedProjectionMatchCount >=
                            projectionOptMatchCount)
                        {
                            int maximumX = -1, minimumX = 1000000;
                            int maximumY = -1, minimumY = 1000000;
                            for (MapPoint *p_mapPoint : bowMatchedMapPoints)
                            {
                                bool mapPointIsBad{};
                                if (!(!p_mapPoint) &&
                                    p_mapPoint->isBad(mapPointIsBad) !=
                                        MapPointStatus::
                                            MAP_POINT_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: isBad returned a failure status "
                                        "although it cannot fail; continuing "
                                        "as before.",
                                        __func__);
                                }
                                if (!p_mapPoint || mapPointIsBad)
                                {
                                    continue;
                                }

                                std::tuple<int, int> mapPointIndexInKeyFrame{};
                                if (p_mapPoint->getIndexInKeyFrame(
                                        p_keyFrame,
                                        mapPointIndexInKeyFrame) !=
                                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: getIndexInKeyFrame returned a "
                                        "failure status although it cannot "
                                        "fail; continuing as before.",
                                        __func__);
                                }
                                std::tuple<size_t, size_t> indexes =
                                    mapPointIndexInKeyFrame;
                                int index = std::get<0>(indexes);
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

                            int                     countKeyFrameCount = 0;
                            // vpMatchedMPs = vpMatchedMP;
                            // vpMPs = vpMapPoints;
                            //  Check the Sim3 transformation with the current
                            //  KeyFrame covisibles
                            std::vector<KeyFrame *> currentCovisibleKeyFrames{};
                            if (p_currentKF->getBestCovisibilityKeyFrames(
                                    countCovisibleCount,
                                    currentCovisibleKeyFrames) !=
                                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getBestCovisibilityKeyFrames returned "
                                    "a failure status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }

                            int covisibleKeyFrameIndex = 0;
                            while (countKeyFrameCount < 3 &&
                                   covisibleKeyFrameIndex <
                                       currentCovisibleKeyFrames.size())
                            {
                                KeyFrame *p_currentCovisibleKeyFrame =
                                    currentCovisibleKeyFrames
                                        [covisibleKeyFrameIndex];
                                Sophus::SE3f currentCovisibleKeyFramePose{};
                                if (p_currentCovisibleKeyFrame->getPose(
                                        currentCovisibleKeyFramePose) !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: getPose returned a failure status "
                                        "although it cannot fail; continuing "
                                        "as before.",
                                        __func__);
                                }
                                Sophus::SE3f currentKFPoseInverse{};
                                if (p_currentKF->getPoseInverse(
                                        currentKFPoseInverse) !=
                                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: getPoseInverse returned a failure "
                                        "status although it cannot fail; "
                                        "continuing as before.",
                                        __func__);
                                }
                                Sophus::SE3d mTjc =
                                    (currentCovisibleKeyFramePose *
                                     currentKFPoseInverse)
                                        .cast<double>();
                                g2o::Sim3 gSjc(mTjc.unit_quaternion(),
                                               mTjc.translation(),
                                               1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int       covisibleProjectionMatchCount = 0;
                                std::vector<MapPoint *>
                                     covisibleMatchedMapPoints;
                                bool isValid{};
                                if (detectCommonRegionsFromLastKF(
                                        p_currentCovisibleKeyFrame,
                                        p_mostBowMatchesKeyFrame,
                                        gSjw,
                                        covisibleProjectionMatchCount,
                                        candidateMapPoints,
                                        covisibleMatchedMapPoints,
                                        isValid) !=
                                    LoopClosingStatus::
                                        LOOP_CLOSING_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: detectCommonRegionsFromLastKF "
                                        "returned a failure status although it "
                                        "cannot fail; continuing as before.",
                                        __func__);
                                }

                                if (isValid)
                                {
                                    Sophus::SE3f Tc_w{};
                                    if (p_currentKF->getPose(Tc_w) !=
                                        KeyFrameStatus::
                                            KEY_FRAME_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: getPose returned a failure "
                                            "status although it cannot fail; "
                                            "continuing as before.",
                                            __func__);
                                    }
                                    Sophus::SE3f Tw_cj{};
                                    if (p_currentCovisibleKeyFrame
                                            ->getPoseInverse(Tw_cj) !=
                                        KeyFrameStatus::
                                            KEY_FRAME_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: getPoseInverse returned a "
                                            "failure status although it cannot "
                                            "fail; continuing as before.",
                                            __func__);
                                    }
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
        if (matchedKeyFrame_out->setNotErase() !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setNotErase returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        g2oScw_out           = g2oBestScw;
        mapPoints_out        = bestMapPoints;
        matchedMapPoints_out = bestMatchedMapPoints;

        isDetected_out = countCoincidenceCount_out >= 3;
        return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
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
    isDetected_out = false;
    return LoopClosingStatus::LOOP_CLOSING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
