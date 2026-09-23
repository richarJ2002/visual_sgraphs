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
#include "Semantic/SemanticVerify.h"

#include "../private_functions.h"

#include <mutex>
#include <thread>

namespace vs_graphs
{
namespace core
{

semantic::SemanticMergeDecision LoopClosing::mergeLocal()
{
    /* ---------------------------------------------------------------------- *
     * SECTION 1 - INITIALISATION
     *
     * Merge Policy
     * ------------------------------------------------------------------------
     * The current map is treated as the authoritative map and therefore
     * survives the merge. The matched map is transformed into the coordinate
     * frame of the current map before all of its contents are appended into
     * the current map. Once all objects have been transferred, the merge map
     * is marked as bad and removed from the atlas.
     * ---------------------------------------------------------------------- */

    /* Constant used to determine the number of temporal keyframes */
    constexpr int kNumTemporalKFs = 25;

    /* Extract the system parameters */
    p_sysParams = types::SystemParams::getParams();

    /* Reject stale place-recognition candidates before stopping other workers.
     */
    if (p_currentKF == nullptr || p_mergeMatchedKF == nullptr ||
        p_currentKF->isBad() || p_mergeMatchedKF->isBad())
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    Map *pCurrentMap = p_currentKF->getMap();
    Map *pMergeMap   = p_mergeMatchedKF->getMap();

    if (pCurrentMap == nullptr || pMergeMap == nullptr ||
        pCurrentMap == pMergeMap || pCurrentMap->isBad() ||
        pMergeMap->isBad() || !p_atlas->isActiveMap(pCurrentMap) ||
        !p_atlas->isActiveMap(pMergeMap))
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 2 - STOP GLOBAL BUNDLE ADJUSTMENT
     *
     * The merge modifies map ownership, poses and graph connectivity.
     * Therefore no optimisation thread is allowed to access these objects
     * while the merge is taking place.
     * ---------------------------------------------------------------------- */

    /* Flag to indicate if bundle adjustment should be relaunched */
    bool bRelaunchBA = false;

    bRelaunchBA = stopGlobalBundleAdjustment();

    /* ---------------------------------------------------------------------- *
     * SECTION 3 - STOP LOCAL MAPPING
     * ---------------------------------------------------------------------- */

    /* Request stop */
    p_localMapper->requestStop();

    /* Wait until local mapper stops */
    while (!p_localMapper->isStopped())
    {
        usleep(1000);
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 4 - IDENTIFY SOURCE AND DESTINATION MAPS
     *
     * pCurrentMap
     *      Survives the merge.
     *
     * pMergeMap
     *      Is transformed into the current-map frame before being appended
     *      into pCurrentMap.
     * ---------------------------------------------------------------------- */

    /*
     * Prevent semantic worker threads from modifying either graph while map
     * frames, ownership, and cross-entity references are being changed.
     */
    std::unique_lock<std::mutex> semanticUpdateLock =
        p_atlas->acquireSemanticUpdateLock();

    /* Revalidate after quiescing workers; retained retired maps keep stale
     * raw pointers alive, so pointer non-nullness alone is insufficient. */
    if (p_currentKF->isBad() || p_mergeMatchedKF->isBad() ||
        p_currentKF->getMap() != pCurrentMap ||
        p_mergeMatchedKF->getMap() != pMergeMap ||
        !p_atlas->isActiveMap(pCurrentMap) || !p_atlas->isActiveMap(pMergeMap))
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        return semantic::SemanticMergeDecision::REJECT;
    }

    const Sophus::SE3d Twc = p_currentKF->getPoseInverse().cast<double>();
    const g2o::Sim3    g2oNonCorrectedSwc(Twc.unit_quaternion(),
                                       Twc.translation(),
                                       1.0);
    const g2o::Sim3    g2oSwCurrentWMerge = g2oNonCorrectedSwc * mg2oMergeScw;
    const g2o::Sim3    g2oSwMergeWCurrent = g2oSwCurrentWMerge.inverse();

    const semantic::SemanticMergeGateResult semanticMergeGate =
        semantic::SemanticVerify::evaluateMapMergeGate(
            pCurrentMap,
            pMergeMap,
            g2oSwCurrentWMerge,
            semantic::SemanticVerify::configFromSystemParams());
    const std::string floorVerificationResult = semanticMergeGate.floorDecision;
    std::cout << "[SemanticMergeGate] surviving_map=" << pCurrentMap->getId()
              << " absorbed_map=" << pMergeMap->getId() << " decision="
              << semantic::SemanticVerify::mergeDecisionName(
                     semanticMergeGate.decision)
              << " reason="
              << semantic::SemanticVerify::mergeReasonName(
                     semanticMergeGate.reason)
              << " shared_rooms=" << semanticMergeGate.sharedRoomCount
              << " aligned_rooms=" << semanticMergeGate.alignedRoomCount
              << " matched_walls=" << semanticMergeGate.matchedWallCount
              << " matched_passages=" << semanticMergeGate.matchedPassageCount
              << " committed=0" << std::endl;
    if (semanticMergeGate.decision != semantic::SemanticMergeDecision::ACCEPT)
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        if (bRelaunchBA)
        {
            relaunchGlobalBundleAdjustment(pCurrentMap);
        }
        return semanticMergeGate.decision;
    }

    /* Discard queued keyframes only after every merge rejection gate passed. */
    p_localMapper->emptyQueue();

    /* Update the connections of the current keyframe */
    p_currentKF->updateConnections();

    /* ---------------------------------------------------------------------- *
     * SECTION 5 - BUILD THE CURRENT-MAP LOCAL WINDOW
     *
     * This window forms the fixed reference side of the weld.
     * ---------------------------------------------------------------------- */

    std::set<KeyFrame *> spLocalWindowKFs;
    std::set<MapPoint *> spLocalWindowMPs;

    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    if (pCurrentMap->isInertial() && pMergeMap->isInertial())
    {
        /* ------------------------------------------------------------------ *
         * Walk backwards through the temporal chain
         * ------------------------------------------------------------------ */

        KeyFrame *pKFi      = p_currentKF;
        int       nInserted = 0;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);

            const std::set<MapPoint *> spMPs = pKFi->getMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());

            pKFi = pKFi->p_prevKF;
            nInserted++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards through the temporal chain
         * ------------------------------------------------------------------ */

        pKFi      = p_currentKF->p_nextKF;
        nInserted = 0;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spLocalWindowKFs.insert(pKFi);

            const std::set<MapPoint *> spMPs = pKFi->getMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());

            pKFi = pKFi->p_nextKF;
            nInserted++;
        }
    }
    else
    {
        spLocalWindowKFs.insert(p_currentKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    std::vector<KeyFrame *> vpCovisibleKFs =
        p_currentKF->getBestCovisibilityKeyFrames(kNumTemporalKFs);

    /* Insert keyframes with best connections into local window */
    spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    spLocalWindowKFs.insert(p_currentKF);

    constexpr int kMaxExpansionIterations = 5;

    int nExpansion = 0;

    /*!
     * If there is not enough keyframes in the local window, then we look at the
     * keyframes currently in spLocalWindowKF and find there best covisibility
     * keyframes. We do this to gaurantee there is enough keyframes in the
     * `spLocalWindowKFs`. We attmemp to reach that amount
     * `kMaxExpansionIterations`. In other words, we are willing to expand the
     * window with `kMaxExpansionIterations` iterations to reach
     * `kNumTemporalKFs` in `spLocalWindowKFs`, if the current keyframe doesn't
     * have enough covisible keyframes attached to it.
     */
    while (spLocalWindowKFs.size() < kNumTemporalKFs &&
           nExpansion < kMaxExpansionIterations)
    {
        std::vector<KeyFrame *> vpNewCovisible;

        for (KeyFrame *pKFi : spLocalWindowKFs)
        {
            const auto vpCovisible =
                pKFi->getBestCovisibilityKeyFrames(kNumTemporalKFs / 2);

            for (KeyFrame *pKFcov : vpCovisible)
            {
                if (!pKFcov)
                    continue;

                if (pKFcov->isBad())
                    continue;

                if (spLocalWindowKFs.count(pKFcov))
                    continue;

                vpNewCovisible.push_back(pKFcov);
            }
        }

        spLocalWindowKFs.insert(vpNewCovisible.begin(), vpNewCovisible.end());

        ++nExpansion;
    }

    /* ---------------------------------------------------------------------- *
     * Collect all landmarks observed by the current-map welding window.
     * ---------------------------------------------------------------------- */

    for (KeyFrame *pKFi : spLocalWindowKFs)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        const std::set<MapPoint *> spMPs = pKFi->getMapPoints();

        /* Insert all the map points into the map point local window */
        spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 6 - BUILD THE MERGE-MAP LOCAL WINDOW
     *
     * These keyframes will be transformed into the current-map frame before
     * being transferred into the surviving map. Essentially, repeat above
     * steps but with the merge map.
     * ---------------------------------------------------------------------- */

    std::set<KeyFrame *> spMergeConnectedKFs;
    std::set<MapPoint *> spMapPointMerge;

    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    if (pCurrentMap->isInertial() && pMergeMap->isInertial())
    {
        KeyFrame *pKFi      = p_mergeMatchedKF;
        int       nInserted = 0;

        /* ------------------------------------------------------------------ *
         * Walk backwards
         * ------------------------------------------------------------------ */

        while (pKFi && nInserted < (kNumTemporalKFs / 2))
        {
            spMergeConnectedKFs.insert(pKFi);

            pKFi = pKFi->p_prevKF;

            nInserted++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards
         * ------------------------------------------------------------------ */

        pKFi = p_mergeMatchedKF->p_nextKF;

        while (pKFi && nInserted < kNumTemporalKFs)
        {
            spMergeConnectedKFs.insert(pKFi);

            pKFi = pKFi->p_nextKF;

            nInserted++;
        }
    }
    else
    {
        spMergeConnectedKFs.insert(p_mergeMatchedKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    vpCovisibleKFs =
        p_mergeMatchedKF->getBestCovisibilityKeyFrames(kNumTemporalKFs);

    /* Insert keyframes with best connections into local window */
    spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    spMergeConnectedKFs.insert(p_mergeMatchedKF);

    /* Reset counter */
    nExpansion = 0;

    /*!
     * If there is not enough keyframes in the merge connected window, then we
     * look at the keyframes currently in spMergeConnectedKFs and find there
     * best covisibility keyframes. We do this to gaurantee there is enough
     * keyframes in the `spMergeConnectedKFs`. We attmemp to reach that amount
     * `kMaxExpansionIterations`. In other words, we are willing to expand the
     * window with `kMaxExpansionIterations` iterations to reach
     * `kNumTemporalKFs` in `spMergeConnectedKFs`, if the current keyframe
     * doesn't have enough covisible keyframes attached to it.
     */
    while (spMergeConnectedKFs.size() < kNumTemporalKFs &&
           nExpansion < kMaxExpansionIterations)
    {
        std::vector<KeyFrame *> vpNewCovisible;

        for (KeyFrame *pKFi : spMergeConnectedKFs)
        {
            const auto vpCovisible =
                pKFi->getBestCovisibilityKeyFrames(kNumTemporalKFs / 2);

            for (KeyFrame *pKFcov : vpCovisible)
            {
                if (!pKFcov)
                    continue;

                if (pKFcov->isBad())
                    continue;

                if (spMergeConnectedKFs.count(pKFcov))
                    continue;

                vpNewCovisible.push_back(pKFcov);
            }
        }

        spMergeConnectedKFs.insert(vpNewCovisible.begin(),
                                   vpNewCovisible.end());

        ++nExpansion;
    }

    /* ---------------------------------------------------------------------- *
     * Collect all landmarks observed by the imported welding window.
     * ---------------------------------------------------------------------- */

    for (KeyFrame *pKFi : spMergeConnectedKFs)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        const auto spMPs = pKFi->getMapPoints();

        /* Insert all the map points into the map point local window */
        spMapPointMerge.insert(spMPs.begin(), spMPs.end());
    }

    /*!
     * Search and fuse only accepts std::vector<MapPoint *>, not
     * std::set<MapPoint *>. Hence, need to move the `spLocalWindowMPs` to a
     * new object to enable search and fuse. With `spLocalWindowMPs` being a
     * set this does mean that `vpCheckFuseMapPoint` will have no duplicates.
     */

    /* Init list of fused map points */
    std::vector<MapPoint *> vpCheckFuseMapPoint;

    /* Reserve the memory for the fused map points */
    vpCheckFuseMapPoint.reserve(spLocalWindowMPs.size());

    /* Copy the map points from the local window */
    vpCheckFuseMapPoint.assign(spLocalWindowMPs.begin(),
                               spLocalWindowMPs.end());

    /* ---------------------------------------------------------------------- *
     * SECTION 7 - COMPUTE THE MAP-TO-MAP SIMILARITY TRANSFORM
     *
     *
     * Purpose
     * ----------------------------------------------------------------------
     *
     * Compute the similarity transform required to express all geometry from
     * the merge-map world frame inside the surviving current-map world frame.
     *
     *
     * Coordinate Frames
     * ----------------------------------------------------------------------
     *
     *      Merge World -----> Current Camera -----> Current World
     *          |                 mg2oMergeScw            Twc
     *
     *
     * Result
     * ----------------------------------------------------------------------
     *
     * g2oSwCurrentWMerge :
     *      Maps merge-world coordinates into current-world coordinates.
     *
     * g2oSwMergeWCurrent :
     *      Inverse transform used when correcting imported keyframe poses.
     *
     *
     * ---------------------------------------------------------------------- */

    /* The map-to-map transforms were computed before the floor preflight so a
     * rejected merge could leave poses and ownership untouched. */

    /* ---------------------------------------------------------------------- *
     * SECTION 8 - CORRECT IMPORTED KEYFRAME POSES
     *
     * Every keyframe belonging to the merge-map welding window is transformed
     * into the current-map reference frame.
     *
     * During this stage no ownership changes occur. The corrected poses are
     * simply cached for later insertion into the surviving map. These
     * transforms are stored in: `vNonCorrectedSim3` and `vCorrectedSim3`.
     * ---------------------------------------------------------------------- */

    /* Stores merge side keyframe's original pose before applying correction */
    KeyFrameAndPose vNonCorrectedSim3;

    /* Stores the corrected merge keyframe pose */
    KeyFrameAndPose vCorrectedSim3;

    /* Iterate through every merge keyframe in the merge connected KF list */
    for (KeyFrame *pKFi : spMergeConnectedKFs)
    {
        /* Skip invalid keyframes */
        if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
        {
            continue;
        }

        /* Extract the current pose of the merge keyframe iteration */
        const Sophus::SE3d TiwMerge = pKFi->getPose().cast<double>();

        /* Convert to a g2o::Sim3 object type */
        const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                    TiwMerge.translation(),
                                    1.0);

        /* Find the transform from current world to keyframe iteration (i) */
        const g2o::Sim3 g2oSiwCurrent = g2oSiwMerge * g2oSwMergeWCurrent;

        /* Store transforms */
        vNonCorrectedSim3[pKFi] = g2oSiwMerge;
        vCorrectedSim3[pKFi]    = g2oSiwCurrent;

        /* Find the scale of transform */
        const double s = g2oSiwCurrent.scale();

        /* Find the transform from merge to current map */
        pKFi->correctedScale = s;
        pKFi->tcwMerge       = Sophus::SE3d(g2oSiwCurrent.rotation(),
                                      g2oSiwCurrent.translation() / s)
                             .cast<float>();

        /* If there is IMU, extract velocity */
        if (pCurrentMap->isImuInitialized())
        {
            const Eigen::Quaternionf Rcor =
                (g2oSiwCurrent.rotation().inverse() * g2oSiwMerge.rotation())
                    .cast<float>();
            pKFi->vwbMerge = Rcor * pKFi->getVelocity();
        }
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 9 - CORRECT IMPORTED MAP POINTS
     *
     * Transform every imported landmark into the current-map coordinate frame.
     *
     * Position:
     *      Full Sim3 transformation.
     *
     * Normal:
     *      Rotation only.
     *
     * Invalid landmarks are removed from the temporary welding set.
     * ---------------------------------------------------------------------- */

    /* Iterate through all the mapped points in the merge map */
    for (auto itMP = spMapPointMerge.begin(); itMP != spMapPointMerge.end();)
    {
        /* Copy the map points */
        MapPoint *pMPi = *itMP;

        /* If the mapped points are invalud, erase and skip */
        if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
        {
            itMP = spMapPointMerge.erase(itMP);
            continue;
        }

        /* Extract position of point */
        const Eigen::Vector3d P3DwMerge = pMPi->getWorldPos().cast<double>();

        /* Transform the point into the current map world frame */
        pMPi->posMerge = g2oSwCurrentWMerge.map(P3DwMerge).cast<float>();

        /* Transform the points surface normal into current map world frame */
        pMPi->normalVectorMerge =
            g2oSwCurrentWMerge.rotation().cast<float>() * pMPi->getNormal();

        /* Step to next mapped point */
        itMP++;
    }

    /* Existing current-map landmarks are used as fusion candidates. */
    vpCheckFuseMapPoint.assign(spLocalWindowMPs.begin(),
                               spLocalWindowMPs.end());

    /* ---------------------------------------------------------------------- *
     * SECTION 10 - TRANSFER THE WELDING WINDOW
     *
     * Both maps are locked simultaneously using std::scoped_lock to guarantee a
     * deadlock-free ownership transfer.
     *
     * The current map remains the authoritative map throughout this section.
     * ---------------------------------------------------------------------- */
    {
        /*!
         * Lock both maps with deadlock-safe acquisition while ownership moves
         */
        std::scoped_lock mapLocks(pCurrentMap->mMutexMapUpdate,
                                  pMergeMap->mMutexMapUpdate);

        /* ------------------------------------------------------------------ *
         * SECTION 11 - TRANSFER CORRECTED KEYFRAMES
         *
         * Every corrected merge-map keyframe is:
         *
         *      1. Pose corrected.
         *      2. Assigned to the current map.
         *      3. Added to the current-map container.
         *      4. Removed from the merge-map container.
         * ------------------------------------------------------------------ */

        /* For every keyframe in merge map, iterate through and transfer */
        for (KeyFrame *pKFi : spMergeConnectedKFs)
        {
            /* Skip invalud keyframes */
            if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
            {
                continue;
            }

            /* Store the old pose of the keyframe */
            pKFi->tcwBefMerge = pKFi->getPose();
            pKFi->twcBefMerge = pKFi->getPoseInverse();

            /* Apply corrected world-to-camera pose in the current-map frame */
            pKFi->setPose(pKFi->tcwMerge);

            /* Change keyframe's internal owning-map pointer to current map */
            pKFi->updateMap(pCurrentMap);

            /* Record which current keyframe triggered this merge correction */
            pKFi->mergeCorrectedKeyFrameId = p_currentKF->mnId;

            /* Insert the same keyframe pointer into surviving map container */
            pCurrentMap->addKeyFrame(pKFi);

            /* Remove the keyframe pointer from the old merge-map container */
            pMergeMap->eraseKeyFrame(pKFi);

            /* If there is IMU, add velocity */
            if (pCurrentMap->isImuInitialized())
            {
                pKFi->setVelocity(pKFi->vwbMerge);
            }
        }

        /* ------------------------------------------------------------------ *
         * SECTION 12 - TRANSFER CORRECTED MAP POINTS
         *
         * The imported landmarks have already been transformed into the
         * current-map frame and therefore only require ownership transfer.
         * ------------------------------------------------------------------ */

        /* Iterate over every merge-map point selected for transfer */
        for (MapPoint *pMPi : spMapPointMerge)
        {
            /* Skip null, invalid, or no-longer merge-owned map points */
            if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
            {
                continue;
            }

            /* Apply position expressed in the surviving current-map frame */
            pMPi->setWorldPos(pMPi->posMerge);

            /* Apply the normal rotated into the surviving current-map frame */
            pMPi->setNormalVector(pMPi->normalVectorMerge);

            /* Change the map point's internal owner to the current map */
            pMPi->updateMap(pCurrentMap);

            /* Register the same map-point pointer in the surviving map */
            pCurrentMap->addMapPoint(pMPi);

            /* Remove the map-point pointer from the obsolete merge map */
            pMergeMap->eraseMapPoint(pMPi);
        }

        /* Set the map to be the current map */
        p_atlas->changeMap(pCurrentMap);

        /* Incrase index tracking the amount of times the maps been changed */
        pCurrentMap->increaseChangeIndex();
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 13 - REBUILD THE IMPORTED SPANNING TREE
     *
     * The imported spanning tree is attached beneath the surviving current
     * keyframe before the previous parent chain is reversed.
     *
     * This preserves graph connectivity while preventing cyclic parent
     * relationships.
     *
     * The links are updated so the imported merge-map keyframes belong to one
     * connected keyframe graph rooted in the surviving current map. You first
     * transfer and reconnect the merge-map keyframes into the current map’s
     * keyframe graph, then (IN SECTION 14) recompute the covisibility graph
     * from their shared map-point observations
     * ---------------------------------------------------------------------- */

    /* If the oriign keyframe of the merp map is valid */
    if (pMergeMap->getOriginKeyFrame())
    {
        /* Allow the former merge-map root to become a normal tree child */
        pMergeMap->getOriginKeyFrame()->setFirstConnection(false);
    }

    /* Init variables to track the new child and parent keyframes */
    KeyFrame *pNewChild  = nullptr;
    KeyFrame *pNewParent = nullptr;

    /* Start with the original parent of the matched merge keyframe */
    pNewChild = p_mergeMatchedKF->getParent();

    /* The matched merge keyframe becomes the first reversed parent */
    pNewParent = p_mergeMatchedKF;

    /* Attach the matched merge keyframe beneath the current keyframe */
    p_mergeMatchedKF->changeParent(p_currentKF);

    /* Reverse each edge along the original merge-map parent chain */
    while (pNewChild)
    {
        /* Remove the old child edge before reversing its direction */
        pNewChild->eraseChild(pNewParent);

        /* Save the next original parent before changing this relation */
        KeyFrame *pOldParent = pNewChild->getParent();

        /* Make the former parent a child of the previous keyframe */
        pNewChild->changeParent(pNewParent);

        /* Advance the new-parent pointer one level up the old chain */
        pNewParent = pNewChild;

        /* Continue with the next parent from the original tree chain */
        pNewChild = pOldParent;
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 14 - REFRESH COVISIBILITY GRAPH
     *
     * Refresh the imported covisibility graph before searching for duplicate
     * landmarks between the two welding windows.
     * ---------------------------------------------------------------------- */

    /* Refresh links for the matched merge-side keyframe */
    p_mergeMatchedKF->updateConnections();

    /* Init list of connected keyframes in merge map */
    std::vector<KeyFrame *> vpMergeConnectedKFs;

    /* Retrieve keyframes covisible with the matched merge keyframe */
    vpMergeConnectedKFs = p_mergeMatchedKF->getVectorCovisibleKeyFrames();

    /* Include the matched merge keyframe in the fusion set */
    vpMergeConnectedKFs.push_back(p_mergeMatchedKF);

    /* Fuse duplicate current-map points into corrected merge keyframes */
    searchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

    /* Refresh covisibility links for current-map local keyframes */
    for (KeyFrame *pKFi : spLocalWindowKFs)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        pKFi->updateConnections();
    }

    /* Refresh covisibility links for imported merge-side keyframes */
    for (KeyFrame *pKFi : spMergeConnectedKFs)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        if (!pKFi || pKFi->isBad())
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        pKFi->updateConnections();
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 15 - LOCAL MERGE OPTIMISATION
     *
     * Perform a local optimisation immediately after the welding window has
     * been merged.
     *
     * Depending on the sensor configuration either:
     *
     *      • MergeInertialBA()
     *      • LoopClosureLocalBundleAdjustment()
     *
     * is executed.
     * ---------------------------------------------------------------------- */

    /* Shared stop flag passed to the selected optimisation routine */
    bool bStop = false;

    /* Init list of local keyframes in current window */
    std::vector<KeyFrame *> vpLocalCurrentWindowKFs;

    /* Remove keyframes stored by any previous merge operation */
    vpLocalCurrentWindowKFs.clear();

    /* Remove merge-connected keyframes stored by earlier processing */
    vpMergeConnectedKFs.clear();

    /* Copy current-side local keyframes into the optimiser vector */
    std::copy(spLocalWindowKFs.begin(),
              spLocalWindowKFs.end(),
              std::back_inserter(vpLocalCurrentWindowKFs));

    /* Copy merge-side connected keyframes into the optimiser vector */
    std::copy(spMergeConnectedKFs.begin(),
              spMergeConnectedKFs.end(),
              std::back_inserter(vpMergeConnectedKFs));

    /* Check whether the active sensor configuration includes an IMU */
    if (p_tracker->sensor == System::IMU_MONOCULAR ||
        p_tracker->sensor == System::IMU_STEREO ||
        p_tracker->sensor == System::IMU_RGBD)
    {
        /* Refine the merged region using visual and inertial constraints */
        Optimizer::mergeInertialBA(p_currentKF,
                                   p_mergeMatchedKF,
                                   &bStop,
                                   pCurrentMap,
                                   vCorrectedSim3);
    }
    else
    {
        /* Refine the merged region using visual observations only */
        Optimizer::loopClosureLocalBundleAdjustment(p_mergeMatchedKF,
                                                    vpMergeConnectedKFs,
                                                    vpLocalCurrentWindowKFs,
                                                    &bStop);
    }

    /* Resume local mapping after merge optimisation is complete */
    p_localMapper->release();

    /* ---------------------------------------------------------------------- *
     * SECTION 16 - RETRIEVE THE REMAINING MERGE MAP
     *
     * The local welding window has already been transferred.
     *
     * This section retrieves every remaining object that still belongs to the
     * obsolete merge map.
     * ---------------------------------------------------------------------- */

    /* Copy all planes currently owned by the merge map */
    std::vector<geometric::Plane *> vpCurrentMapPlanes =
        pMergeMap->getAllPlanes();

    /* Copy all keyframes currently owned by the merge map */
    std::vector<KeyFrame *> vpCurrentMapKFs = pMergeMap->getAllKeyFrames();

    const bool hasValidRemainingMergeKeyFrame =
        std::any_of(vpCurrentMapKFs.begin(),
                    vpCurrentMapKFs.end(),
                    [pMergeMap](KeyFrame *p_keyFrame_in)
                    {
                        return p_keyFrame_in != nullptr &&
                               !p_keyFrame_in->isBad() &&
                               p_keyFrame_in->getMap() == pMergeMap;
                    });

    /* Copy all map points currently owned by the merge map */
    std::vector<MapPoint *> vpCurrentMapMPs = pMergeMap->getAllMapPoints();

    /* Copy all markers currently owned by the merge map */
    std::vector<semantic::Marker *> vpCurrentMapMarkers =
        pMergeMap->getAllMarkers();

    /* Copy all passages currently owned by the merge map */
    std::vector<vs_graphs::core::semantic::Passage *> vpCurrentMapPassages =
        pMergeMap->getAllPassages();

    /* Copy all detected rooms currently owned by the merge map */
    std::vector<semantic::Room *> vpCurrentDetectedMapRooms =
        pMergeMap->getAllDetectedMapRooms();

    /* Copy all marker-based rooms currently owned by the merge map */
    std::vector<semantic::Room *> vpCurrentMarkerBasedMapRooms =
        pMergeMap->getAllMarkerBasedMapRooms();

    /* Copy all floors currently owned by the merge map */
    std::vector<semantic::Floor *> vpCurrentMapFloors =
        pMergeMap->getAllFloors();

    /* Stop local mapping before any remaining ownership is transferred. */
    p_localMapper->requestStop();

    while (!p_localMapper->isStopped())
    {
        usleep(1000);
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 17 - CORRECT REMAINING MERGE-MAP GEOMETRY
     *
     * Every remaining keyframe and landmark that was not part of the welding
     * window is transformed into the surviving current-map frame prior to graph
     * optimisation.
     * ---------------------------------------------------------------------- */

    /* Process remaining keyframes only when the merge map is not empty */
    if (hasValidRemainingMergeKeyFrame)
    {
        /* Apply monocular scale correction to the remaining merge map */
        if (p_tracker->sensor == System::MONOCULAR)
        {
            /* Lock the merge map while updating its poses and landmarks */
            std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate);

            /* Correct each remaining merge keyframe into current world */
            for (KeyFrame *pKFi : vpCurrentMapKFs)
            {
                /* Skip invalid keyframes or keyframes no longer in this map */
                if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
                {
                    continue;
                }

                /* Read the keyframe pose in the merge map world frame */
                const Sophus::SE3d TiwMerge = pKFi->getPose().cast<double>();

                /* Convert the rigid keyframe pose into a unit scale Sim3 */
                const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                            TiwMerge.translation(),
                                            1.0);

                /* Express the keyframe pose in the current world frame */
                const g2o::Sim3 g2oSiwCurrent =
                    g2oSiwMerge * g2oSwMergeWCurrent;

                /* Store the original keyframe pose before correction */
                vNonCorrectedSim3[pKFi] = g2oSiwMerge;

                /* Store the corrected keyframe pose for later processing */
                vCorrectedSim3[pKFi] = g2oSiwCurrent;

                /* Extract the scale introduced by the map correction */
                const double s = g2oSiwCurrent.scale();

                /* Store the applied scale in the keyframe */
                pKFi->correctedScale = s;

                /* Preserve the original world to camera pose */
                pKFi->tcwBefMerge = pKFi->getPose();

                /* Preserve the original camera to world pose */
                pKFi->twcBefMerge = pKFi->getPoseInverse();

                /* Apply the corrected rigid pose in the current world frame */
                pKFi->setPose(Sophus::SE3d(g2oSiwCurrent.rotation(),
                                           g2oSiwCurrent.translation() / s)
                                  .cast<float>());

                /* Rotate velocity when the surviving map uses inertial data */
                if (pCurrentMap->isImuInitialized())
                {
                    /* Compute the rotation from old to corrected world frame */
                    const Eigen::Quaternionf Rcor =
                        (g2oSiwCurrent.rotation().inverse() *
                         g2oSiwMerge.rotation())
                            .cast<float>();

                    /* Express the keyframe velocity in the corrected frame */
                    pKFi->setVelocity(Rcor * pKFi->getVelocity());
                }
            }

            /* Correct each remaining merge landmark into current world */
            for (MapPoint *pMPi : vpCurrentMapMPs)
            {
                /* Skip invalid points or points no longer in this map */
                if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
                {
                    continue;
                }

                /* Read the landmark position in the merge world frame */
                const Eigen::Vector3d P3DwMerge =
                    pMPi->getWorldPos().cast<double>();

                const Eigen::Vector3f normal_mergeWorld = pMPi->getNormal();

                /* Transform the landmark into the current world frame */
                pMPi->setWorldPos(
                    g2oSwCurrentWMerge.map(P3DwMerge).cast<float>());

                pMPi->setNormalVector(
                    g2oSwCurrentWMerge.rotation().cast<float>() *
                    normal_mergeWorld);

                /* Refresh the point normal and valid viewing depth range */
                pMPi->updateNormalAndDepth();
            }
        }

        /* ------------------------------------------------------------------ *
         * SECTION 18 - OPTIMISE THE COMPLETE MERGED GRAPH
         *
         * Once every remaining object has been corrected into the current-map
         * frame, optimise the complete merged graph before changing ownership.
         * ------------------------------------------------------------------ */

        if (p_tracker->sensor != System::MONOCULAR)
        {
            Optimizer::optimizeEssentialGraph(p_mergeMatchedKF,
                                              pMergeMap,
                                              vpLocalCurrentWindowKFs,
                                              vpMergeConnectedKFs,
                                              vpCurrentMapKFs,
                                              vpCurrentMapMPs,
                                              g2oSwCurrentWMerge);

            /*!
             * Skeleton topology is derived from the live Voxblox volume. The
             * map-revision notification invalidates that volume after this
             * correction, so no stale topology snapshot is retained here.
             */
        }
    }

    const bool semanticGeometryWasOptimized =
        hasValidRemainingMergeKeyFrame &&
        p_tracker->sensor != System::MONOCULAR;

    bool semanticGeometryWasPropagated = semanticGeometryWasOptimized;

    /*!
     * Small source maps and monocular merges do not run the merge essential
     * graph. Propagate the final welding-BA pose deltas directly so semantic
     * geometry follows the corrected imported keyframes rather than receiving
     * only the coarse map-level Sim3.
     */
    if (!semanticGeometryWasOptimized)
    {
        KeyFrameAndPose finalKeyFramePoses_WorldToCamera;

        for (const auto &[p_keyFrame, poseBefore_WorldToCamera] :
             vNonCorrectedSim3)
        {
            (void)poseBefore_WorldToCamera;

            if (p_keyFrame == nullptr || p_keyFrame->isBad())
            {
                continue;
            }

            const Sophus::SE3d poseAfter_WorldToCamera =
                p_keyFrame->getPose().cast<double>();

            /*
             * A monocular map merge can change scale. ORB-SLAM stores the
             * corrected keyframe as SE3 by dividing the Sim3 translation by
             * its scale, so reconstruct the corresponding Sim3 before
             * deriving the semantic world-frame correction. Using a unit
             * scale here would leave planes, rooms and passages at their old
             * size while MapPoints receive the full map Sim3.
             */
            double poseAfterScale = 1.0;

            const auto correctedPoseIterator = vCorrectedSim3.find(p_keyFrame);

            if (correctedPoseIterator != vCorrectedSim3.end() &&
                std::isfinite(correctedPoseIterator->second.scale()) &&
                std::abs(correctedPoseIterator->second.scale()) > 1e-12)
            {
                poseAfterScale = correctedPoseIterator->second.scale();
            }

            finalKeyFramePoses_WorldToCamera.insert_or_assign(
                p_keyFrame,
                g2o::Sim3(poseAfter_WorldToCamera.unit_quaternion(),
                          poseAfterScale *
                              poseAfter_WorldToCamera.translation(),
                          poseAfterScale));
        }

        utils::utils::Utils::propagateSemanticPoseCorrections(
            pMergeMap,
            vNonCorrectedSim3,
            finalKeyFramePoses_WorldToCamera,
            g2oSwCurrentWMerge);

        semanticGeometryWasPropagated = true;
    }

    /*
     * A map can retain orphan landmarks after every keyframe in its local
     * window has already moved. Those points still require the map-level Sim3
     * even though there is no essential graph left to optimize.
     */
    if (!hasValidRemainingMergeKeyFrame)
    {
        for (MapPoint *p_mapPoint : vpCurrentMapMPs)
        {
            if (p_mapPoint == nullptr || p_mapPoint->isBad() ||
                p_mapPoint->getMap() != pMergeMap)
            {
                continue;
            }

            const Eigen::Vector3f position_mergeWorld_m =
                p_mapPoint->getWorldPos();

            const Eigen::Vector3f normal_mergeWorld = p_mapPoint->getNormal();

            p_mapPoint->setWorldPos(
                g2oSwCurrentWMerge.map(position_mergeWorld_m.cast<double>())
                    .cast<float>());

            p_mapPoint->setNormalVector(
                g2oSwCurrentWMerge.rotation().cast<float>() *
                normal_mergeWorld);

            p_mapPoint->updateNormalAndDepth();
        }
    }

    /* ------------------------------------------------------------------ *
     * SECTION 19 - TRANSFER THE REMAINING MAP CONTENTS
     *
     * Every remaining object stored inside the obsolete merge map is
     * transferred into the surviving current map.
     *
     * Object Types
     * ------------------------------------------------------------------
     *  - KeyFrames
     *  - MapPoints
     *  - Planes
     *  - Passages
     *  - Detected Rooms
     *  - Marker Rooms
     *  - Markers
     * ------------------------------------------------------------------ */
    {
        const bool primarySemanticGeometryWasCorrected =
            semanticGeometryWasOptimized || semanticGeometryWasPropagated;

        int nextPlaneId = 0;
        for (geometric::Plane *p_existingPlane : pCurrentMap->getAllPlanes())
        {
            if (p_existingPlane != nullptr)
            {
                nextPlaneId =
                    std::max(nextPlaneId, p_existingPlane->getId() + 1);
            }
        }

        int nextMarkerId = 0;
        for (semantic::Marker *p_existingMarker : pCurrentMap->getAllMarkers())
        {
            if (p_existingMarker != nullptr)
            {
                nextMarkerId =
                    std::max(nextMarkerId, p_existingMarker->getId() + 1);
            }
        }

        // Get Merge Map Mutex
        std::scoped_lock mapLocks(pCurrentMap->mMutexMapUpdate,
                                  pMergeMap->mMutexMapUpdate);

        // Loop over the KeyFrames of the current map and move them to the
        // new map
        for (KeyFrame *pKFi : vpCurrentMapKFs)
        {
            if (!pKFi || pKFi->isBad() || pKFi->getMap() != pMergeMap)
                continue;

            pKFi->updateMap(pCurrentMap);
            pCurrentMap->addKeyFrame(pKFi);
            pMergeMap->eraseKeyFrame(pKFi);
        }

        // Loop over the MapPoints of the current map and move them to the
        // new map
        for (MapPoint *pMPi : vpCurrentMapMPs)
        {
            if (!pMPi || pMPi->isBad() || pMPi->getMap() != pMergeMap)
                continue;

            pMPi->updateMap(pCurrentMap);
            pCurrentMap->addMapPoint(pMPi);
            pMergeMap->eraseMapPoint(pMPi);
        }

        /* -------------------------------------------------------------- *
         * SECTION 20 - TRANSFER SEMANTIC OBJECTS
         *
         * Semantic objects are transformed into the current-map reference
         * frame before ownership is transferred.
         *
         * Geometry is preserved while map ownership is updated.
         * -------------------------------------------------------------- */
        for (geometric::Plane *plane : vpCurrentMapPlanes)
        {
            /* Skip invalid planes */
            if (plane == nullptr || plane->isBad())
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                plane->applyTransform(g2oSwCurrentWMerge);
            }

            /* Update the map the plane belongs to */
            plane->setMap(pCurrentMap);

            /*!
             * Take index size of planes in new map to find an id to add to
             * the map which hasn't been taken.
             */
            plane->setId(nextPlaneId++);

            /* Add the plane to the map new merged plane to the new map */
            pCurrentMap->addMapPlane(plane);

            /* Remove the current plane from the old map */
            pMergeMap->eraseMapPlane(plane);
        }

        // Loop over the Markers of the current map and move them to the new
        // map
        for (semantic::Marker *pMarker : vpCurrentMapMarkers)
        {
            if (!pMarker)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                pMarker->applyTransform(g2oSwCurrentWMerge);
            }

            pMarker->setMap(pCurrentMap);
            pMarker->setId(nextMarkerId++);
            pCurrentMap->addMapMarker(pMarker);
            pMergeMap->eraseMapMarker(pMarker);
        }

        /*!
         * Loop over the passages of the primary map and move them to the
         * secondary map.
         */
        for (vs_graphs::core::semantic::Passage *passage : vpCurrentMapPassages)
        {
            /* Skip invalid rooms */
            if (passage == nullptr)
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                passage->applyTransform(g2oSwCurrentWMerge);
            }

            semantic::Passage *p_retainedPassage = nullptr;
            for (semantic::Passage *p_existingPassage :
                 pCurrentMap->getAllPassages())
            {
                if (p_existingPassage != nullptr &&
                    p_existingPassage->getId() == passage->getId())
                {
                    p_retainedPassage = p_existingPassage;
                    break;
                }
            }

            pMergeMap->eraseMapPassage(passage);
            if (p_retainedPassage != nullptr)
            {
                p_retainedPassage->mergeFromDuplicate(passage);
                for (semantic::Room *p_room : vpCurrentDetectedMapRooms)
                {
                    if (p_room != nullptr)
                    {
                        p_room->replacePassageAssociation(passage,
                                                          p_retainedPassage);
                    }
                }
                for (semantic::Room *p_room : vpCurrentMarkerBasedMapRooms)
                {
                    if (p_room != nullptr)
                    {
                        p_room->replacePassageAssociation(passage,
                                                          p_retainedPassage);
                    }
                }
                passage->setBad();
                continue;
            }

            passage->setMap(pCurrentMap);
            pCurrentMap->addMapPassage(passage);
        }

        /*!
         * Loop over the rooms of the primary map and move them to the
         * secondary map.
         */
        for (vs_graphs::core::semantic::Room *room : vpCurrentDetectedMapRooms)
        {
            /* Skip invalid rooms */
            if (room == nullptr || room->isBad())
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                room->applyTransform(g2oSwCurrentWMerge);
            }

            /* Set the map of the room in the current map */
            room->setMap(pCurrentMap);

            /* Add the room to the current map */
            pCurrentMap->addDetectedMapRoom(room);

            /* Remove the room from the merged map */
            pMergeMap->eraseDetectedMapRoom(room);
        }

        // Loop over the Marker-based Rooms of the current map and move them
        // to the new map
        for (vs_graphs::core::semantic::Room *pRoom :
             vpCurrentMarkerBasedMapRooms)
        {
            if (!pRoom)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                pRoom->applyTransform(g2oSwCurrentWMerge);
            }

            pRoom->setMap(pCurrentMap);
            pCurrentMap->addCandidateMapRoom(pRoom);
            pMergeMap->eraseMarkerBasedMapRoom(pRoom);
        }

        for (semantic::Floor *p_floor : vpCurrentMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            if (!semanticGeometryWasPropagated)
            {
                p_floor->applyTransform(g2oSwCurrentWMerge);
            }
            pMergeMap->eraseMapFloor(p_floor);
            semantic::Floor *p_retainedFloor = nullptr;
            for (semantic::Floor *p_existingFloor : pCurrentMap->getAllFloors())
            {
                if (p_existingFloor != nullptr &&
                    p_existingFloor->getId() == p_floor->getId())
                {
                    p_retainedFloor = p_existingFloor;
                    break;
                }
            }
            if (p_retainedFloor != nullptr)
            {
                mergeFloorEvidenceAndRooms(p_retainedFloor, p_floor);
                continue;
            }
            p_floor->setMap(pCurrentMap);
            pCurrentMap->addMapFloor(p_floor);
        }

        collapseMergedFloors(pCurrentMap);

        /*
         * Voxblox topology is derived from a TSDF/ESDF volume and is not an
         * independently mergeable landmark set. Appending snapshots from two
         * map frames creates disconnected duplicate edges and false wall
         * crossings. The external Voxblox node receives the map-revision event,
         * clears its volume, and supplies a fresh snapshot after reintegration.
         */
        pCurrentMap->setSkeletonClusterPoints({});
        pCurrentMap->setSkeletonEdges({});

        /* Rebuild imported room-wall index entries before fusion. */
        for (semantic::Room *p_room : pCurrentMap->getAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (geometric::Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    pCurrentMap->addRoomWallPlane(p_wall);
                }
            }
        }

        /* Fuse only after every semantic relationship is visible. */
        if (p_sysParams->semSeg.reassociate.enabled)
        {
            utils::utils::Utils::reAssociateSemanticPlanes(p_atlas);
        }

        std::vector<semantic::Room *> importedRooms = vpCurrentDetectedMapRooms;
        importedRooms.insert(importedRooms.end(),
                             vpCurrentMarkerBasedMapRooms.begin(),
                             vpCurrentMarkerBasedMapRooms.end());

        /* Stable semantic identity reconciliation is a merge invariant, not
         * an optional geometry-reassociation feature. */
        utils::utils::Utils::fuseDuplicateRoomsAfterMerge(pCurrentMap,
                                                          importedRooms);

        if (p_sysParams->semSeg.reassociate.enabled)
        {
            utils::utils::Utils::reAssociateRooms(p_atlas);
            utils::utils::Utils::reAssociatePassages(p_atlas);
        }
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 22 - FINALISE THE MERGE
     *
     * Merge Complete
     * ----------------------------------------------------------------------
     *      - Current map survives.
     *      - Merge map contains no remaining objects.
     *      - Atlas ownership updated.
     *      - Merge edge inserted.
     *      - Obsolete map removed from the atlas.
     *
     * After this point every surviving SLAM object belongs exclusively to
     * pCurrentMap.
     * ---------------------------------------------------------------------- */

    p_mergeMatchedKF->addMergeEdge(p_currentKF);
    p_currentKF->addMergeEdge(p_mergeMatchedKF);

    pCurrentMap->increaseChangeIndex();

    /*!
     * A map merge changes the world-frame poses of previously integrated
     * observations. Notify derived mapping consumers only after ownership,
     * semantic reconciliation, and graph connectivity are fully committed.
     * Voxblox uses this revision to discard TSDF/ESDF state expressed in the
     * pre-merge coordinate frame.
     */
    pCurrentMap->informNewBigChange();

    /* All surviving objects now belong to pCurrentMap. */
    p_atlas->changeMap(pCurrentMap);
    p_atlas->setMapBad(pMergeMap);
    p_atlas->removeBadMaps();

    std::cout << "[SemanticMergeGate] surviving_map=" << pCurrentMap->getId()
              << " absorbed_map=" << pMergeMap->getId()
              << " decision=ACCEPT reason=ALIGNED"
              << " floor=" << floorVerificationResult << " committed=1"
              << std::endl;

    semanticUpdateLock.unlock();
    p_localMapper->release();

    if (bRelaunchBA &&
        (!pCurrentMap->isImuInitialized() ||
         (pCurrentMap->getKeyFrameCount() < 200 && p_atlas->countMaps() == 1)))
    {
        relaunchGlobalBundleAdjustment(pCurrentMap);
    }

    return semantic::SemanticMergeDecision::ACCEPT;
}

} // namespace core
} // namespace vs_graphs
