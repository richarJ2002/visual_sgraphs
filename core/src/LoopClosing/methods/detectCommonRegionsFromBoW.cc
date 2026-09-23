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
#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{

bool LoopClosing::detectCommonRegionsFromBoW(
    std::vector<KeyFrame *> &vpBowCand,
    KeyFrame               *&pMatchedKF2,
    KeyFrame               *&pLastCurrentKF,
    g2o::Sim3               &g2oScw,
    int                     &nNumCoincidences,
    std::vector<MapPoint *> &vpMPs,
    std::vector<MapPoint *> &vpMatchedMPs)
{
    int nBoWMatches     = 20;
    int nBoWInliers     = 15;
    int nSim3Inliers    = 20;
    int nProjMatches    = 50;
    int nProjOptMatches = 80;

    set<KeyFrame *> spConnectedKeyFrames = p_currentKF->getConnectedKeyFrames();

    int nNumCovisibles = 10;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);

    // Varibles to select the best numbe
    KeyFrame               *pBestMatchedKF;
    int                     nBestMatchesReproj   = 0;
    int                     nBestNumCoindicendes = 0;
    g2o::Sim3               g2oBestScw;
    std::vector<MapPoint *> vpBestMapPoints;
    std::vector<MapPoint *> vpBestMatchedMapPoints;

    int         numCandidates = vpBowCand.size();
    vector<int> vnStage(numCandidates, 0);
    vector<int> vnMatchesStage(numCandidates, 0);

    int index = 0;
    // Verbose::PrintMess("BoW candidates: There are " +
    // to_string(vpBowCand.size()) + " possible candidates ",
    // Verbose::VERBOSITY_DEBUG);
    for (KeyFrame *pKFi : vpBowCand)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        // std::cout << "KF candidate: " << pKFi->mnId << std::endl;
        // Current KF against KF with covisibles version
        std::vector<KeyFrame *> vpCovKFi =
            pKFi->getBestCovisibilityKeyFrames(nNumCovisibles);
        if (vpCovKFi.empty())
        {
            // std::cout << "Covisible list empty" << std::endl;
            vpCovKFi.push_back(pKFi);
        }
        else
        {
            vpCovKFi.push_back(vpCovKFi[0]);
            vpCovKFi[0] = pKFi;
        }

        bool bAbortByNearKF = false;
        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            if (spConnectedKeyFrames.find(vpCovKFi[j]) !=
                spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;
                break;
            }
        }
        if (bAbortByNearKF)
        {
            // std::cout << "Check BoW aborted because is close to the matched
            // one " << std::endl;
            continue;
        }
        // std::cout << "Check BoW continue because is far to the matched one "
        // << std::endl;

        std::vector<std::vector<MapPoint *>> vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<MapPoint *> spMatchedMPi;
        int                  numBoWMatches = 0;

        KeyFrame *pMostBoWMatchesKF  = pKFi;
        int       nMostBoWNumMatches = 0;

        std::vector<MapPoint *> vpMatchedPoints =
            std::vector<MapPoint *>(p_currentKF->getMapPointMatches().size(),
                                    static_cast<MapPoint *>(nullptr));
        std::vector<KeyFrame *> vpKeyFrameMatchedMP =
            std::vector<KeyFrame *>(p_currentKF->getMapPointMatches().size(),
                                    static_cast<KeyFrame *>(nullptr));

        int nIndexMostBoWMatchesKF = 0;
        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            if (!vpCovKFi[j] || vpCovKFi[j]->isBad())
                continue;

            int num = matcherBoW.searchByBoW(p_currentKF,
                                             vpCovKFi[j],
                                             vvpMatchedMPs[j]);
            if (num > nMostBoWNumMatches)
            {
                nMostBoWNumMatches     = num;
                nIndexMostBoWMatchesKF = j;
            }
        }

        for (int j = 0; j < vpCovKFi.size(); ++j)
        {
            for (int k = 0; k < vvpMatchedMPs[j].size(); ++k)
            {
                MapPoint *pMPi_j = vvpMatchedMPs[j][k];
                if (!pMPi_j || pMPi_j->isBad())
                    continue;

                if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;

                    vpMatchedPoints[k]     = pMPi_j;
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }
            }
        }

        // pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];

        if (numBoWMatches >= nBoWMatches) // TODO pick a good threshold
        {
            // Geometric validation
            bool bFixedScale = fixScale;
            if (p_tracker->sensor == System::IMU_MONOCULAR &&
                !p_currentKF->getMap()->getInertialBA2())
                bFixedScale = false;

            Sim3Solver solver = Sim3Solver(p_currentKF,
                                           pMostBoWMatchesKF,
                                           vpMatchedPoints,
                                           bFixedScale,
                                           vpKeyFrameMatchedMP);
            solver.setRansacParameters(0.99,
                                       nBoWInliers,
                                       300); // at least 15 inliers

            bool            bNoMore = false;
            vector<bool>    vbInliers;
            int             nInliers;
            bool            bConverge = false;
            Eigen::Matrix4f mTcm;
            while (!bConverge && !bNoMore)
            {
                mTcm =
                    solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
                // Verbose::PrintMess("BoW guess: Solver achieve " +
                // to_string(nInliers) + " geometrical inliers among " +
                // to_string(nBoWInliers) + " BoW matches",
                // Verbose::VERBOSITY_DEBUG);
            }

            if (bConverge)
            {
                // std::cout << "Check BoW: SolverSim3 converged" << std::endl;

                // Verbose::PrintMess("BoW guess: Convergende with " +
                // to_string(nInliers) + " geometrical inliers among " +
                // to_string(nBoWInliers) + " BoW matches",
                // Verbose::VERBOSITY_DEBUG);
                //  Match by reprojection
                vpCovKFi.clear();
                vpCovKFi = pMostBoWMatchesKF->getBestCovisibilityKeyFrames(
                    nNumCovisibles);
                vpCovKFi.push_back(pMostBoWMatchesKF);
                set<KeyFrame *> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                // std::cout << "There are " << vpCovKFi.size() <<" near KFs" <<
                // std::endl;

                set<MapPoint *>    spMapPoints;
                vector<MapPoint *> vpMapPoints;
                vector<KeyFrame *> vpKeyFrames;
                for (KeyFrame *pCovKFi : vpCovKFi)
                {
                    for (MapPoint *pCovMPij : pCovKFi->getMapPointMatches())
                    {
                        if (!pCovMPij || pCovMPij->isBad())
                            continue;

                        if (spMapPoints.find(pCovMPij) == spMapPoints.end())
                        {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }

                // std::cout << "There are " << vpKeyFrames.size() <<" KFs which
                // view all the mappoints" << std::endl;

                g2o::Sim3 gScm(solver.getEstimatedRotation().cast<double>(),
                               solver.getEstimatedTranslation().cast<double>(),
                               (double)solver.getEstimatedScale());
                g2o::Sim3 gSmw(
                    pMostBoWMatchesKF->getRotation().cast<double>(),
                    pMostBoWMatchesKF->getTranslation().cast<double>(),
                    1.0);
                g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current
                                              // from the world position
                Sophus::Sim3f correctedPose =
                    utils::converter::Converter::toSophus(gScw);

                vector<MapPoint *> vpMatchedMP;
                vpMatchedMP.resize(p_currentKF->getMapPointMatches().size(),
                                   static_cast<MapPoint *>(nullptr));
                vector<KeyFrame *> vpMatchedKF;
                vpMatchedKF.resize(p_currentKF->getMapPointMatches().size(),
                                   static_cast<KeyFrame *>(nullptr));
                int numProjMatches = matcher.searchByProjection(p_currentKF,
                                                                correctedPose,
                                                                vpMapPoints,
                                                                vpKeyFrames,
                                                                vpMatchedMP,
                                                                vpMatchedKF,
                                                                8,
                                                                1.5);
                // cout <<"BoW: " << numProjMatches << " matches between " <<
                // vpMapPoints.size() << " points with coarse Sim3" << endl;

                if (numProjMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    bool bFixedScale = fixScale;
                    if (p_tracker->sensor == System::IMU_MONOCULAR &&
                        !p_currentKF->getMap()->getInertialBA2())
                        bFixedScale = false;

                    int numOptMatches = Optimizer::optimizeSim3(p_currentKF,
                                                                pKFi,
                                                                vpMatchedMP,
                                                                gScm,
                                                                10,
                                                                fixScale,
                                                                mHessian7x7,
                                                                true);

                    if (numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(
                            pMostBoWMatchesKF->getRotation().cast<double>(),
                            pMostBoWMatchesKF->getTranslation().cast<double>(),
                            1.0);
                        g2o::Sim3 gScw =
                            gScm * gSmw; // Similarity matrix of current from
                                         // the world position
                        Sophus::Sim3f correctedPose =
                            utils::converter::Converter::toSophus(gScw);

                        vector<MapPoint *> vpMatchedMP;
                        vpMatchedMP.resize(
                            p_currentKF->getMapPointMatches().size(),
                            static_cast<MapPoint *>(nullptr));
                        int numProjOptMatches =
                            matcher.searchByProjection(p_currentKF,
                                                       correctedPose,
                                                       vpMapPoints,
                                                       vpMatchedMP,
                                                       5,
                                                       1.0);

                        if (numProjOptMatches >= nProjOptMatches)
                        {
                            int max_x = -1, min_x = 1000000;
                            int max_y = -1, min_y = 1000000;
                            for (MapPoint *pMPi : vpMatchedMP)
                            {
                                if (!pMPi || pMPi->isBad())
                                {
                                    continue;
                                }

                                tuple<size_t, size_t> indexes =
                                    pMPi->getIndexInKeyFrame(pKFi);
                                int index = get<0>(indexes);
                                if (index >= 0)
                                {
                                    int coord_x =
                                        pKFi->keyPointsUndistorted[index].pt.x;
                                    if (coord_x < min_x)
                                    {
                                        min_x = coord_x;
                                    }
                                    if (coord_x > max_x)
                                    {
                                        max_x = coord_x;
                                    }
                                    int coord_y =
                                        pKFi->keyPointsUndistorted[index].pt.y;
                                    if (coord_y < min_y)
                                    {
                                        min_y = coord_y;
                                    }
                                    if (coord_y > max_y)
                                    {
                                        max_y = coord_y;
                                    }
                                }
                            }

                            int                nNumKFs = 0;
                            // vpMatchedMPs = vpMatchedMP;
                            // vpMPs = vpMapPoints;
                            //  Check the Sim3 transformation with the current
                            //  KeyFrame covisibles
                            vector<KeyFrame *> vpCurrentCovKFs =
                                p_currentKF->getBestCovisibilityKeyFrames(
                                    nNumCovisibles);

                            int j = 0;
                            while (nNumKFs < 3 && j < vpCurrentCovKFs.size())
                            {
                                KeyFrame    *pKFj = vpCurrentCovKFs[j];
                                Sophus::SE3d mTjc =
                                    (pKFj->getPose() *
                                     p_currentKF->getPoseInverse())
                                        .cast<double>();
                                g2o::Sim3          gSjc(mTjc.unit_quaternion(),
                                               mTjc.translation(),
                                               1.0);
                                g2o::Sim3          gSjw = gSjc * gScw;
                                int                numProjMatches_j = 0;
                                vector<MapPoint *> vpMatchedMPs_j;
                                bool bValid = detectCommonRegionsFromLastKF(
                                    pKFj,
                                    pMostBoWMatchesKF,
                                    gSjw,
                                    numProjMatches_j,
                                    vpMapPoints,
                                    vpMatchedMPs_j);

                                if (bValid)
                                {
                                    Sophus::SE3f Tc_w  = p_currentKF->getPose();
                                    Sophus::SE3f Tw_cj = pKFj->getPoseInverse();
                                    Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                    Eigen::Vector3f vector_dist =
                                        Tc_cj.translation();
                                    nNumKFs++;
                                }
                                j++;
                            }

                            if (nNumKFs < 3)
                            {
                                vnStage[index]        = 8;
                                vnMatchesStage[index] = nNumKFs;
                            }

                            if (nBestMatchesReproj < numProjOptMatches)
                            {
                                nBestMatchesReproj     = numProjOptMatches;
                                nBestNumCoindicendes   = nNumKFs;
                                pBestMatchedKF         = pMostBoWMatchesKF;
                                g2oBestScw             = gScw;
                                vpBestMapPoints        = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                            }
                        }
                    }
                }
            }
        }
        index++;
    }

    if (nBestMatchesReproj > 0)
    {
        pLastCurrentKF   = p_currentKF;
        nNumCoincidences = nBestNumCoindicendes;
        pMatchedKF2      = pBestMatchedKF;
        pMatchedKF2->setNotErase();
        g2oScw       = g2oBestScw;
        vpMPs        = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;

        return nNumCoincidences >= 3;
    }
    else
    {
        int maxStage = -1;
        int maxMatched;
        for (int i = 0; i < vnStage.size(); ++i)
        {
            if (vnStage[i] > maxStage)
            {
                maxStage   = vnStage[i];
                maxMatched = vnMatchesStage[i];
            }
        }
    }
    return false;
}

} // namespace core
} // namespace vs_graphs
