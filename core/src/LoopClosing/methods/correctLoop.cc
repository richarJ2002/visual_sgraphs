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

#include "Optimizer.h"

#include <chrono>
#include <mutex>
#include <thread>

namespace vs_graphs
{
namespace core
{

void LoopClosing::correctLoop()
{
    // Avoid new keyframes are inserted while correcting the loop
    p_localMapper->requestStop();
    p_localMapper->emptyQueue();

    /* Stop and reclaim any global bundle-adjustment worker before mutation. */
    stopGlobalBundleAdjustment();

    // Wait until Local Mapping has effectively stopped
    while (!p_localMapper->isStopped())
        usleep(1000);

    // Ensure current keyframe is updated
    p_currentKF->updateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected
    // Sim3 pose by propagation
    currentConnectedKFs = p_currentKF->getVectorCovisibleKeyFrames();
    currentConnectedKFs.push_back(p_currentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[p_currentKF] = mg2oLoopScw;
    Sophus::SE3f Twc           = p_currentKF->getPoseInverse();
    Sophus::SE3f Tcw           = p_currentKF->getPose();
    g2o::Sim3    g2oScw(Tcw.unit_quaternion().cast<double>(),
                     Tcw.translation().cast<double>(),
                     1.0);
    NonCorrectedSim3[p_currentKF] = g2oScw;

    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3
    // (scale translation)
    Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(),
                              mg2oLoopScw.translation() / mg2oLoopScw.scale());
    p_currentKF->setPose(correctedTcw.cast<float>());

    Map *pLoopMap = p_currentKF->getMap();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartFusion =
        std::chrono::steady_clock::now();
#endif

    {
        // Get Map Mutex
        unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

        const bool bImuInit = pLoopMap->isImuInitialized();

        for (vector<KeyFrame *>::iterator vit  = currentConnectedKFs.begin(),
                                          vend = currentConnectedKFs.end();
             vit != vend;
             vit++)
        {
            KeyFrame *pKFi = *vit;

            if (pKFi != p_currentKF)
            {
                Sophus::SE3f Tiw = pKFi->getPose();
                Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
                g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;
                // Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi] = g2oCorrectedSiw;

                // Update keyframe pose with corrected Sim3. First transform
                // Sim3 to SE3 (scale translation)
                Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),
                                          g2oCorrectedSiw.translation() /
                                              g2oCorrectedSiw.scale());
                pKFi->setPose(correctedTiw.cast<float>());

                // Pose without correction
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(),
                                 Tiw.translation().cast<double>(),
                                 1.0);
                NonCorrectedSim3[pKFi] = g2oSiw;
            }
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so
        // that they align with the other side of the loop
        for (KeyFrameAndPose::iterator mit  = CorrectedSim3.begin(),
                                       mend = CorrectedSim3.end();
             mit != mend;
             mit++)
        {
            KeyFrame *pKFi            = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

            // Update keyframe pose with corrected Sim3. First transform Sim3 to
            // SE3 (scale translation)
            /*Sophus::SE3d
            correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation()
            / g2oCorrectedSiw.scale());
            pKFi->setPose(correctedTiw.cast<float>());*/

            vector<MapPoint *> vpMPsi = pKFi->getMapPointMatches();
            for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
            {
                MapPoint *pMPi = vpMPsi[iMP];
                if (!pMPi)
                    continue;
                if (pMPi->isBad())
                    continue;
                if (pMPi->correctedByKeyFrameId == p_currentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with
                // corrected pose
                Eigen::Vector3d P3Dw = pMPi->getWorldPos().cast<double>();
                Eigen::Vector3d eigCorrectedP3Dw =
                    g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                pMPi->setWorldPos(eigCorrectedP3Dw.cast<float>());
                pMPi->correctedByKeyFrameId        = p_currentKF->mnId;
                pMPi->correctedReferenceKeyFrameId = pKFi->mnId;
                pMPi->updateNormalAndDepth();
            }

            // Correct velocity according to orientation correction
            if (bImuInit)
            {
                Eigen::Quaternionf Rcor =
                    (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation())
                        .cast<float>();
                pKFi->setVelocity(Rcor * pKFi->getVelocity());
            }

            // Make sure connections are updated
            pKFi->updateConnections();
        }
        // TODO Check this index increasement
        p_atlas->getCurrentMap()->increaseChangeIndex();

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (size_t i = 0; i < loopMatchedMPs.size(); i++)
        {
            if (loopMatchedMPs[i])
            {
                MapPoint *pLoopMP = loopMatchedMPs[i];
                MapPoint *pCurMP  = p_currentKF->getMapPoint(i);
                if (pCurMP)
                    pCurMP->replace(pLoopMP);
                else
                {
                    p_currentKF->addMapPoint(pLoopMP, i);
                    pLoopMP->addObservation(p_currentKF, i);
                    pLoopMP->computeDistinctiveDescriptors();
                }
            }
        }
        // cout << "LC: end replacing duplicated" << endl;
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    searchAndFuse(CorrectedSim3, loopMapPoints);

    // After the MapPoint fusion, new links in the covisibility graph will
    // appear attaching both sides of the loop
    map<KeyFrame *, set<KeyFrame *>> LoopConnections;

    for (vector<KeyFrame *>::iterator vit  = currentConnectedKFs.begin(),
                                      vend = currentConnectedKFs.end();
         vit != vend;
         vit++)
    {
        KeyFrame          *pKFi = *vit;
        vector<KeyFrame *> vpPreviousNeighbors =
            pKFi->getVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->updateConnections();
        LoopConnections[pKFi] = pKFi->getConnectedKeyFrames();
        for (vector<KeyFrame *>::iterator
                 vit_prev  = vpPreviousNeighbors.begin(),
                 vend_prev = vpPreviousNeighbors.end();
             vit_prev != vend_prev;
             vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for (vector<KeyFrame *>::iterator vit2  = currentConnectedKFs.begin(),
                                          vend2 = currentConnectedKFs.end();
             vit2 != vend2;
             vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    bool bFixedScale = fixScale;
    // TODO CHECK; Solo para el monocular inertial
    if (p_tracker->sensor == System::IMU_MONOCULAR &&
        !p_currentKF->getMap()->getInertialBA2())
        bFixedScale = false;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndFusion =
        std::chrono::steady_clock::now();

    double timeFusion =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndFusion - time_StartFusion)
            .count();
    vdLoopFusion_ms.push_back(timeFusion);
#endif
    // cout << "Optimize essential graph" << endl;
    if (pLoopMap->isInertial() && pLoopMap->isImuInitialized())
    {
        Optimizer::optimizeEssentialGraph4DoF(pLoopMap,
                                              p_loopMatchedKF,
                                              p_currentKF,
                                              NonCorrectedSim3,
                                              CorrectedSim3,
                                              LoopConnections);
    }
    else
    {
        // cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
        Optimizer::optimizeEssentialGraph(pLoopMap,
                                          p_loopMatchedKF,
                                          p_currentKF,
                                          NonCorrectedSim3,
                                          CorrectedSim3,
                                          LoopConnections,
                                          bFixedScale);
    }
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndOpt =
        std::chrono::steady_clock::now();

    double timeOptEss =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            time_EndOpt - time_EndFusion)
            .count();
    vdLoopOptEss_ms.push_back(timeOptEss);
#endif

    p_atlas->informNewBigChange();

    // Add loop edge
    p_loopMatchedKF->addLoopEdge(p_currentKF);
    p_currentKF->addLoopEdge(p_loopMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment (Only if few
    // keyframes, if not it would take too much time)
    if (!pLoopMap->isImuInitialized() ||
        (pLoopMap->getKeyFrameCount() < 200 && p_atlas->countMaps() == 1))
    {
        std::unique_lock<std::mutex> globalBundleAdjustmentLock(mMutexGBA);
        runningGBA    = true;
        finishedGBA   = false;
        correctionGBA = numCorrection;
        globalBundleAdjustmentStopRequested.store(false,
                                                  std::memory_order_release);

        p_threadGBA = new thread(&LoopClosing::runGlobalBundleAdjustment,
                                 this,
                                 pLoopMap,
                                 p_currentKF->mnId,
                                 fullBundleAdjustmentIndex);
    }

    // Loop closed. Release Local Mapping.
    p_localMapper->release();

    lastLoopKeyFrameId =
        p_currentKF
            ->mnId; // TODO old varible, it is not use in the new algorithm
}

} // namespace core
} // namespace vs_graphs
