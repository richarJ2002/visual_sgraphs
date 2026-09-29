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
#include "LocalMapping.h"
#include "System.h"
#include "Tracking.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"

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
    constexpr int COUNT_TEMPORAL_KEY_FRAMES = 25;

    /* Extract the system parameters */
    types::SystemParams *p_params = nullptr;
    if (types::SystemParams::getParams(p_params) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }
    p_sysParams = p_params;

    /* Reject stale place-recognition candidates before stopping other workers.
     */
    if (p_currentKF == nullptr || p_mergeMatchedKF == nullptr ||
        p_currentKF->isBad() || p_mergeMatchedKF->isBad())
    {
        return semantic::SemanticMergeDecision::REJECT;
    }

    Map *p_currentMap = p_currentKF->getMap();
    Map *p_mergeMap   = p_mergeMatchedKF->getMap();

    if (p_currentMap == nullptr || p_mergeMap == nullptr ||
        p_currentMap == p_mergeMap || p_currentMap->isBad() ||
        p_mergeMap->isBad() || !p_atlas->isActiveMap(p_currentMap) ||
        !p_atlas->isActiveMap(p_mergeMap))
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
    bool shouldRelaunchBa = false;

    shouldRelaunchBa = stopGlobalBundleAdjustment();

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
        p_currentKF->getMap() != p_currentMap ||
        p_mergeMatchedKF->getMap() != p_mergeMap ||
        !p_atlas->isActiveMap(p_currentMap) ||
        !p_atlas->isActiveMap(p_mergeMap))
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

    semantic::SemanticVerifyConfig configuration2{};
    if (semantic::SemanticVerify::configFromSystemParams(configuration2) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        // configFromSystemParams cannot fail; continue as before.
    }
    semantic::SemanticMergeGateResult semanticMergeGate{};
    if (semantic::SemanticVerify::evaluateMapMergeGate(p_currentMap,
                                                       p_mergeMap,
                                                       g2oSwCurrentWMerge,
                                                       semanticMergeGate,
                                                       configuration2) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        // evaluateMapMergeGate cannot fail; continue as before.
    }
    const std::string floorVerificationResult = semanticMergeGate.floorDecision;
    const char       *p_name                  = nullptr;
    if (semantic::SemanticVerify::mergeDecisionName(semanticMergeGate.decision,
                                                    p_name) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        // mergeDecisionName cannot fail; continue as before.
    }
    const char *p_name2 = nullptr;
    if (semantic::SemanticVerify::mergeReasonName(semanticMergeGate.reason,
                                                  p_name2) !=
        semantic::SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS)
    {
        // mergeReasonName cannot fail; continue as before.
    }
    std::cout << "[SemanticMergeGate] surviving_map=" << p_currentMap->getId()
              << " absorbed_map=" << p_mergeMap->getId()
              << " decision=" << p_name << " reason=" << p_name2
              << " shared_rooms=" << semanticMergeGate.sharedRoomCount
              << " aligned_rooms=" << semanticMergeGate.alignedRoomCount
              << " matched_walls=" << semanticMergeGate.matchedWallCount
              << " matched_passages=" << semanticMergeGate.matchedPassageCount
              << " committed=0" << std::endl;
    if (semanticMergeGate.decision != semantic::SemanticMergeDecision::ACCEPT)
    {
        semanticUpdateLock.unlock();
        p_localMapper->release();
        if (shouldRelaunchBa)
        {
            relaunchGlobalBundleAdjustment(p_currentMap);
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

    std::set<KeyFrame *> localWindowKeyFrames;
    std::set<MapPoint *> localWindowMapPoints;

    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    if (p_currentMap->isInertial() && p_mergeMap->isInertial())
    {
        /* ------------------------------------------------------------------ *
         * Walk backwards through the temporal chain
         * ------------------------------------------------------------------ */

        KeyFrame *p_keyFrame    = p_currentKF;
        int       insertedCount = 0;

        while (p_keyFrame && insertedCount < COUNT_TEMPORAL_KEY_FRAMES)
        {
            localWindowKeyFrames.insert(p_keyFrame);

            const std::set<MapPoint *> mapPoints = p_keyFrame->getMapPoints();
            localWindowMapPoints.insert(mapPoints.begin(), mapPoints.end());

            p_keyFrame = p_keyFrame->p_prevKF;
            insertedCount++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards through the temporal chain
         * ------------------------------------------------------------------ */

        p_keyFrame    = p_currentKF->p_nextKF;
        insertedCount = 0;

        while (p_keyFrame && insertedCount < COUNT_TEMPORAL_KEY_FRAMES)
        {
            localWindowKeyFrames.insert(p_keyFrame);

            const std::set<MapPoint *> mapPoints = p_keyFrame->getMapPoints();
            localWindowMapPoints.insert(mapPoints.begin(), mapPoints.end());

            p_keyFrame = p_keyFrame->p_nextKF;
            insertedCount++;
        }
    }
    else
    {
        localWindowKeyFrames.insert(p_currentKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    std::vector<KeyFrame *> covisibleKeyFrames =
        p_currentKF->getBestCovisibilityKeyFrames(COUNT_TEMPORAL_KEY_FRAMES);

    /* Insert keyframes with best connections into local window */
    localWindowKeyFrames.insert(covisibleKeyFrames.begin(),
                                covisibleKeyFrames.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    localWindowKeyFrames.insert(p_currentKF);

    constexpr int MAXIMUM_EXPANSION_ITERATIONS = 5;

    int expansionCount = 0;

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
    while (localWindowKeyFrames.size() < COUNT_TEMPORAL_KEY_FRAMES &&
           expansionCount < MAXIMUM_EXPANSION_ITERATIONS)
    {
        std::vector<KeyFrame *> newCovisibles;

        for (KeyFrame *p_keyFrame : localWindowKeyFrames)
        {
            const auto covisibles = p_keyFrame->getBestCovisibilityKeyFrames(
                COUNT_TEMPORAL_KEY_FRAMES / 2);

            for (KeyFrame *p_covisibleKeyFrame : covisibles)
            {
                if (!p_covisibleKeyFrame)
                    continue;

                if (p_covisibleKeyFrame->isBad())
                    continue;

                if (localWindowKeyFrames.count(p_covisibleKeyFrame))
                    continue;

                newCovisibles.push_back(p_covisibleKeyFrame);
            }
        }

        localWindowKeyFrames.insert(newCovisibles.begin(), newCovisibles.end());

        ++expansionCount;
    }

    /* ---------------------------------------------------------------------- *
     * Collect all landmarks observed by the current-map welding window.
     * ---------------------------------------------------------------------- */

    for (KeyFrame *p_keyFrame : localWindowKeyFrames)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
        if (!p_keyFrame || p_keyFrame->isBad())
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        const std::set<MapPoint *> mapPoints = p_keyFrame->getMapPoints();

        /* Insert all the map points into the map point local window */
        localWindowMapPoints.insert(mapPoints.begin(), mapPoints.end());
    }

    /* ---------------------------------------------------------------------- *
     * SECTION 6 - BUILD THE MERGE-MAP LOCAL WINDOW
     *
     * These keyframes will be transformed into the current-map frame before
     * being transferred into the surviving map. Essentially, repeat above
     * steps but with the merge map.
     * ---------------------------------------------------------------------- */

    std::set<KeyFrame *> mergeConnectedKeyFrames;
    std::set<MapPoint *> mapPointMerges;

    /*!
     * If using IMU, construct temporal inertial chain. Otherwise, start with
     * current keyframe for local window.
     */
    if (p_currentMap->isInertial() && p_mergeMap->isInertial())
    {
        KeyFrame *p_keyFrame    = p_mergeMatchedKF;
        int       insertedCount = 0;

        /* ------------------------------------------------------------------ *
         * Walk backwards
         * ------------------------------------------------------------------ */

        while (p_keyFrame && insertedCount < (COUNT_TEMPORAL_KEY_FRAMES / 2))
        {
            mergeConnectedKeyFrames.insert(p_keyFrame);

            p_keyFrame = p_keyFrame->p_prevKF;

            insertedCount++;
        }

        /* ------------------------------------------------------------------ *
         * Walk forwards
         * ------------------------------------------------------------------ */

        p_keyFrame = p_mergeMatchedKF->p_nextKF;

        while (p_keyFrame && insertedCount < COUNT_TEMPORAL_KEY_FRAMES)
        {
            mergeConnectedKeyFrames.insert(p_keyFrame);

            p_keyFrame = p_keyFrame->p_nextKF;

            insertedCount++;
        }
    }
    else
    {
        mergeConnectedKeyFrames.insert(p_mergeMatchedKF);
    }

    /* ---------------------------------------------------------------------- *
     * Expand the local window using covisibility.
     * ---------------------------------------------------------------------- */

    /* Create list of strongest covisibility connectsion to current keyframe */
    covisibleKeyFrames = p_mergeMatchedKF->getBestCovisibilityKeyFrames(
        COUNT_TEMPORAL_KEY_FRAMES);

    /* Insert keyframes with best connections into local window */
    mergeConnectedKeyFrames.insert(covisibleKeyFrames.begin(),
                                   covisibleKeyFrames.end());

    /*!
     * Insert the current keyframe as an unconditional safety measure.
     * `splocalWindowKF` is a set, hence duplicates will not be inserted.
     */
    mergeConnectedKeyFrames.insert(p_mergeMatchedKF);

    /* Reset counter */
    expansionCount = 0;

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
    while (mergeConnectedKeyFrames.size() < COUNT_TEMPORAL_KEY_FRAMES &&
           expansionCount < MAXIMUM_EXPANSION_ITERATIONS)
    {
        std::vector<KeyFrame *> newCovisibles;

        for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
        {
            const auto covisibles = p_keyFrame->getBestCovisibilityKeyFrames(
                COUNT_TEMPORAL_KEY_FRAMES / 2);

            for (KeyFrame *p_covisibleKeyFrame : covisibles)
            {
                if (!p_covisibleKeyFrame)
                    continue;

                if (p_covisibleKeyFrame->isBad())
                    continue;

                if (mergeConnectedKeyFrames.count(p_covisibleKeyFrame))
                    continue;

                newCovisibles.push_back(p_covisibleKeyFrame);
            }
        }

        mergeConnectedKeyFrames.insert(newCovisibles.begin(),
                                       newCovisibles.end());

        ++expansionCount;
    }

    /* ---------------------------------------------------------------------- *
     * Collect all landmarks observed by the imported welding window.
     * ---------------------------------------------------------------------- */

    for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
    {
        /* Skip invalid keyframes. (Shouldn't need this but good for safety) */
        if (!p_keyFrame || p_keyFrame->isBad())
        {
            continue;
        }

        /* Extract the map points from the keyframe */
        const auto mapPoints = p_keyFrame->getMapPoints();

        /* Insert all the map points into the map point local window */
        mapPointMerges.insert(mapPoints.begin(), mapPoints.end());
    }

    /*!
     * Search and fuse only accepts std::vector<MapPoint *>, not
     * std::set<MapPoint *>. Hence, need to move the `spLocalWindowMPs` to a
     * new object to enable search and fuse. With `spLocalWindowMPs` being a
     * set this does mean that `vpCheckFuseMapPoint` will have no duplicates.
     */

    /* Init list of fused map points */
    std::vector<MapPoint *> checkFuseMapPoints;

    /* Reserve the memory for the fused map points */
    checkFuseMapPoints.reserve(localWindowMapPoints.size());

    /* Copy the map points from the local window */
    checkFuseMapPoints.assign(localWindowMapPoints.begin(),
                              localWindowMapPoints.end());

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
    for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
    {
        /* Skip invalid keyframes */
        if (!p_keyFrame || p_keyFrame->isBad() ||
            p_keyFrame->getMap() != p_mergeMap)
        {
            continue;
        }

        /* Extract the current pose of the merge keyframe iteration */
        const Sophus::SE3d TiwMerge = p_keyFrame->getPose().cast<double>();

        /* Convert to a g2o::Sim3 object type */
        const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                    TiwMerge.translation(),
                                    1.0);

        /* Find the transform from current world to keyframe iteration (i) */
        const g2o::Sim3 g2oSiwCurrent = g2oSiwMerge * g2oSwMergeWCurrent;

        /* Store transforms */
        vNonCorrectedSim3[p_keyFrame] = g2oSiwMerge;
        vCorrectedSim3[p_keyFrame]    = g2oSiwCurrent;

        /* Find the scale of transform */
        const double s = g2oSiwCurrent.scale();

        /* Find the transform from merge to current map */
        p_keyFrame->correctedScale = s;
        p_keyFrame->tcwMerge       = Sophus::SE3d(g2oSiwCurrent.rotation(),
                                            g2oSiwCurrent.translation() / s)
                                   .cast<float>();

        /* If there is IMU, extract velocity */
        if (p_currentMap->isImuInitialized())
        {
            const Eigen::Quaternionf Rcor =
                (g2oSiwCurrent.rotation().inverse() * g2oSiwMerge.rotation())
                    .cast<float>();
            p_keyFrame->vwbMerge = Rcor * p_keyFrame->getVelocity();
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
    for (auto itMapPoint = mapPointMerges.begin();
         itMapPoint != mapPointMerges.end();)
    {
        /* Copy the map points */
        MapPoint *p_currentMapPoint = *itMapPoint;

        /* If the mapped points are invalud, erase and skip */
        if (!p_currentMapPoint || p_currentMapPoint->isBad() ||
            p_currentMapPoint->getMap() != p_mergeMap)
        {
            itMapPoint = mapPointMerges.erase(itMapPoint);
            continue;
        }

        /* Extract position of point */
        const Eigen::Vector3d P3DwMerge =
            p_currentMapPoint->getWorldPos().cast<double>();

        /* Transform the point into the current map world frame */
        p_currentMapPoint->posMerge =
            g2oSwCurrentWMerge.map(P3DwMerge).cast<float>();

        /* Transform the points surface normal into current map world frame */
        p_currentMapPoint->normalVectorMerge =
            g2oSwCurrentWMerge.rotation().cast<float>() *
            p_currentMapPoint->getNormal();

        /* Step to next mapped point */
        itMapPoint++;
    }

    /* Existing current-map landmarks are used as fusion candidates. */
    checkFuseMapPoints.assign(localWindowMapPoints.begin(),
                              localWindowMapPoints.end());

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
        std::scoped_lock mapLocks(p_currentMap->mapUpdateMutex,
                                  p_mergeMap->mapUpdateMutex);

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
        for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
        {
            /* Skip invalud keyframes */
            if (!p_keyFrame || p_keyFrame->isBad() ||
                p_keyFrame->getMap() != p_mergeMap)
            {
                continue;
            }

            /* Store the old pose of the keyframe */
            p_keyFrame->tcwBefMerge = p_keyFrame->getPose();
            p_keyFrame->twcBefMerge = p_keyFrame->getPoseInverse();

            /* Apply corrected world-to-camera pose in the current-map frame */
            p_keyFrame->setPose(p_keyFrame->tcwMerge);

            /* Change keyframe's internal owning-map pointer to current map */
            p_keyFrame->updateMap(p_currentMap);

            /* Record which current keyframe triggered this merge correction */
            p_keyFrame->mergeCorrectedKeyFrameId = p_currentKF->id;

            /* Insert the same keyframe pointer into surviving map container */
            p_currentMap->addKeyFrame(p_keyFrame);

            /* Remove the keyframe pointer from the old merge-map container */
            p_mergeMap->eraseKeyFrame(p_keyFrame);

            /* If there is IMU, add velocity */
            if (p_currentMap->isImuInitialized())
            {
                p_keyFrame->setVelocity(p_keyFrame->vwbMerge);
            }
        }

        /* ------------------------------------------------------------------ *
         * SECTION 12 - TRANSFER CORRECTED MAP POINTS
         *
         * The imported landmarks have already been transformed into the
         * current-map frame and therefore only require ownership transfer.
         * ------------------------------------------------------------------ */

        /* Iterate over every merge-map point selected for transfer */
        for (MapPoint *p_currentMapPoint : mapPointMerges)
        {
            /* Skip null, invalid, or no-longer merge-owned map points */
            if (!p_currentMapPoint || p_currentMapPoint->isBad() ||
                p_currentMapPoint->getMap() != p_mergeMap)
            {
                continue;
            }

            /* Apply position expressed in the surviving current-map frame */
            p_currentMapPoint->setWorldPos(p_currentMapPoint->posMerge);

            /* Apply the normal rotated into the surviving current-map frame */
            p_currentMapPoint->setNormalVector(
                p_currentMapPoint->normalVectorMerge);

            /* Change the map point's internal owner to the current map */
            p_currentMapPoint->updateMap(p_currentMap);

            /* Register the same map-point pointer in the surviving map */
            p_currentMap->addMapPoint(p_currentMapPoint);

            /* Remove the map-point pointer from the obsolete merge map */
            p_mergeMap->eraseMapPoint(p_currentMapPoint);
        }

        /* Set the map to be the current map */
        p_atlas->changeMap(p_currentMap);

        /* Incrase index tracking the amount of times the maps been changed */
        p_currentMap->increaseChangeIndex();
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
    if (p_mergeMap->getOriginKeyFrame())
    {
        /* Allow the former merge-map root to become a normal tree child */
        p_mergeMap->getOriginKeyFrame()->setFirstConnection(false);
    }

    /* Init variables to track the new child and parent keyframes */
    KeyFrame *p_newChild  = nullptr;
    KeyFrame *p_newParent = nullptr;

    /* Start with the original parent of the matched merge keyframe */
    p_newChild = p_mergeMatchedKF->getParent();

    /* The matched merge keyframe becomes the first reversed parent */
    p_newParent = p_mergeMatchedKF;

    /* Attach the matched merge keyframe beneath the current keyframe */
    p_mergeMatchedKF->changeParent(p_currentKF);

    /* Reverse each edge along the original merge-map parent chain */
    while (p_newChild)
    {
        /* Remove the old child edge before reversing its direction */
        p_newChild->eraseChild(p_newParent);

        /* Save the next original parent before changing this relation */
        KeyFrame *p_oldParent = p_newChild->getParent();

        /* Make the former parent a child of the previous keyframe */
        p_newChild->changeParent(p_newParent);

        /* Advance the new-parent pointer one level up the old chain */
        p_newParent = p_newChild;

        /* Continue with the next parent from the original tree chain */
        p_newChild = p_oldParent;
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
    searchAndFuse(vCorrectedSim3, checkFuseMapPoints);

    /* Refresh covisibility links for current-map local keyframes */
    for (KeyFrame *p_keyFrame : localWindowKeyFrames)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        if (!p_keyFrame || p_keyFrame->isBad())
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        p_keyFrame->updateConnections();
    }

    /* Refresh covisibility links for imported merge-side keyframes */
    for (KeyFrame *p_keyFrame : mergeConnectedKeyFrames)
    {
        /* Skip null keyframes and keyframes marked as invalid */
        if (!p_keyFrame || p_keyFrame->isBad())
        {
            continue;
        }

        /* Recompute graph connections from shared map-point observations */
        p_keyFrame->updateConnections();
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
    bool shouldStop = false;

    /* Init list of local keyframes in current window */
    std::vector<KeyFrame *> localCurrentWindowKeyFrames;

    /* Remove keyframes stored by any previous merge operation */
    localCurrentWindowKeyFrames.clear();

    /* Remove merge-connected keyframes stored by earlier processing */
    vpMergeConnectedKFs.clear();

    /* Copy current-side local keyframes into the optimiser vector */
    std::copy(localWindowKeyFrames.begin(),
              localWindowKeyFrames.end(),
              std::back_inserter(localCurrentWindowKeyFrames));

    /* Copy merge-side connected keyframes into the optimiser vector */
    std::copy(mergeConnectedKeyFrames.begin(),
              mergeConnectedKeyFrames.end(),
              std::back_inserter(vpMergeConnectedKFs));

    /* Check whether the active sensor configuration includes an IMU */
    if (p_tracker->sensor == System::IMU_MONOCULAR ||
        p_tracker->sensor == System::IMU_STEREO ||
        p_tracker->sensor == System::IMU_RGBD)
    {
        /* Refine the merged region using visual and inertial constraints */
        Optimizer::mergeInertialBA(p_currentKF,
                                   p_mergeMatchedKF,
                                   &shouldStop,
                                   p_currentMap,
                                   vCorrectedSim3);
    }
    else
    {
        /* Refine the merged region using visual observations only */
        Optimizer::loopClosureLocalBundleAdjustment(p_mergeMatchedKF,
                                                    vpMergeConnectedKFs,
                                                    localCurrentWindowKeyFrames,
                                                    &shouldStop);
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
    std::vector<geometric::Plane *> currentMapPlanes =
        p_mergeMap->getAllPlanes();

    /* Copy all keyframes currently owned by the merge map */
    std::vector<KeyFrame *> currentMapKeyFrames = p_mergeMap->getAllKeyFrames();

    const bool hasValidRemainingMergeKeyFrame =
        std::any_of(currentMapKeyFrames.begin(),
                    currentMapKeyFrames.end(),
                    [p_mergeMap](KeyFrame *p_keyFrame_in)
                    {
                        return p_keyFrame_in != nullptr &&
                               !p_keyFrame_in->isBad() &&
                               p_keyFrame_in->getMap() == p_mergeMap;
                    });

    /* Copy all map points currently owned by the merge map */
    std::vector<MapPoint *> currentMapMapPoints = p_mergeMap->getAllMapPoints();

    /* Copy all markers currently owned by the merge map */
    std::vector<semantic::Marker *> currentMapMarkers =
        p_mergeMap->getAllMarkers();

    /* Copy all passages currently owned by the merge map */
    std::vector<vs_graphs::core::semantic::Passage *> currentMapPassages =
        p_mergeMap->getAllPassages();

    /* Copy all detected rooms currently owned by the merge map */
    std::vector<semantic::Room *> currentDetectedMapRooms =
        p_mergeMap->getAllDetectedMapRooms();

    /* Copy all marker-based rooms currently owned by the merge map */
    std::vector<semantic::Room *> currentMarkerBasedMapRooms =
        p_mergeMap->getAllMarkerBasedMapRooms();

    /* Copy all floors currently owned by the merge map */
    std::vector<semantic::Floor *> currentMapFloors =
        p_mergeMap->getAllFloors();

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
            std::unique_lock<std::mutex> mergeLock(p_mergeMap->mapUpdateMutex);

            /* Correct each remaining merge keyframe into current world */
            for (KeyFrame *p_keyFrame : currentMapKeyFrames)
            {
                /* Skip invalid keyframes or keyframes no longer in this map */
                if (!p_keyFrame || p_keyFrame->isBad() ||
                    p_keyFrame->getMap() != p_mergeMap)
                {
                    continue;
                }

                /* Read the keyframe pose in the merge map world frame */
                const Sophus::SE3d TiwMerge =
                    p_keyFrame->getPose().cast<double>();

                /* Convert the rigid keyframe pose into a unit scale Sim3 */
                const g2o::Sim3 g2oSiwMerge(TiwMerge.unit_quaternion(),
                                            TiwMerge.translation(),
                                            1.0);

                /* Express the keyframe pose in the current world frame */
                const g2o::Sim3 g2oSiwCurrent =
                    g2oSiwMerge * g2oSwMergeWCurrent;

                /* Store the original keyframe pose before correction */
                vNonCorrectedSim3[p_keyFrame] = g2oSiwMerge;

                /* Store the corrected keyframe pose for later processing */
                vCorrectedSim3[p_keyFrame] = g2oSiwCurrent;

                /* Extract the scale introduced by the map correction */
                const double s = g2oSiwCurrent.scale();

                /* Store the applied scale in the keyframe */
                p_keyFrame->correctedScale = s;

                /* Preserve the original world to camera pose */
                p_keyFrame->tcwBefMerge = p_keyFrame->getPose();

                /* Preserve the original camera to world pose */
                p_keyFrame->twcBefMerge = p_keyFrame->getPoseInverse();

                /* Apply the corrected rigid pose in the current world frame */
                p_keyFrame->setPose(
                    Sophus::SE3d(g2oSiwCurrent.rotation(),
                                 g2oSiwCurrent.translation() / s)
                        .cast<float>());

                /* Rotate velocity when the surviving map uses inertial data */
                if (p_currentMap->isImuInitialized())
                {
                    /* Compute the rotation from old to corrected world frame */
                    const Eigen::Quaternionf Rcor =
                        (g2oSiwCurrent.rotation().inverse() *
                         g2oSiwMerge.rotation())
                            .cast<float>();

                    /* Express the keyframe velocity in the corrected frame */
                    p_keyFrame->setVelocity(Rcor * p_keyFrame->getVelocity());
                }
            }

            /* Correct each remaining merge landmark into current world */
            for (MapPoint *p_currentMapPoint : currentMapMapPoints)
            {
                /* Skip invalid points or points no longer in this map */
                if (!p_currentMapPoint || p_currentMapPoint->isBad() ||
                    p_currentMapPoint->getMap() != p_mergeMap)
                {
                    continue;
                }

                /* Read the landmark position in the merge world frame */
                const Eigen::Vector3d P3DwMerge =
                    p_currentMapPoint->getWorldPos().cast<double>();

                const Eigen::Vector3f normal_mergeWorld =
                    p_currentMapPoint->getNormal();

                /* Transform the landmark into the current world frame */
                p_currentMapPoint->setWorldPos(
                    g2oSwCurrentWMerge.map(P3DwMerge).cast<float>());

                p_currentMapPoint->setNormalVector(
                    g2oSwCurrentWMerge.rotation().cast<float>() *
                    normal_mergeWorld);

                /* Refresh the point normal and valid viewing depth range */
                p_currentMapPoint->updateNormalAndDepth();
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
                                              p_mergeMap,
                                              localCurrentWindowKeyFrames,
                                              vpMergeConnectedKFs,
                                              currentMapKeyFrames,
                                              currentMapMapPoints,
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

        if (utils::utils::Utils::propagateSemanticPoseCorrections(
                p_mergeMap,
                vNonCorrectedSim3,
                finalKeyFramePoses_WorldToCamera,
                g2oSwCurrentWMerge) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            // propagateSemanticPoseCorrections cannot fail; continue as before.
        }

        semanticGeometryWasPropagated = true;
    }

    /*
     * A map can retain orphan landmarks after every keyframe in its local
     * window has already moved. Those points still require the map-level Sim3
     * even though there is no essential graph left to optimize.
     */
    if (!hasValidRemainingMergeKeyFrame)
    {
        for (MapPoint *p_mapPoint : currentMapMapPoints)
        {
            if (p_mapPoint == nullptr || p_mapPoint->isBad() ||
                p_mapPoint->getMap() != p_mergeMap)
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
        for (geometric::Plane *p_existingPlane : p_currentMap->getAllPlanes())
        {
            if (p_existingPlane != nullptr)
            {
                int existingPlaneGetId{};
                if (p_existingPlane->getId(existingPlaneGetId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                nextPlaneId = std::max(nextPlaneId, existingPlaneGetId + 1);
            }
        }

        int nextMarkerId = 0;
        for (semantic::Marker *p_existingMarker : p_currentMap->getAllMarkers())
        {
            if (p_existingMarker != nullptr)
            {
                int existingMarkerId{};
                if (p_existingMarker->getId(existingMarkerId) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                nextMarkerId = std::max(nextMarkerId, existingMarkerId + 1);
            }
        }

        // Get Merge Map Mutex
        std::scoped_lock mapLocks(p_currentMap->mapUpdateMutex,
                                  p_mergeMap->mapUpdateMutex);

        // Loop over the KeyFrames of the current map and move them to the
        // new map
        for (KeyFrame *p_keyFrame : currentMapKeyFrames)
        {
            if (!p_keyFrame || p_keyFrame->isBad() ||
                p_keyFrame->getMap() != p_mergeMap)
                continue;

            p_keyFrame->updateMap(p_currentMap);
            p_currentMap->addKeyFrame(p_keyFrame);
            p_mergeMap->eraseKeyFrame(p_keyFrame);
        }

        // Loop over the MapPoints of the current map and move them to the
        // new map
        for (MapPoint *p_currentMapPoint : currentMapMapPoints)
        {
            if (!p_currentMapPoint || p_currentMapPoint->isBad() ||
                p_currentMapPoint->getMap() != p_mergeMap)
                continue;

            p_currentMapPoint->updateMap(p_currentMap);
            p_currentMap->addMapPoint(p_currentMapPoint);
            p_mergeMap->eraseMapPoint(p_currentMapPoint);
        }

        /* -------------------------------------------------------------- *
         * SECTION 20 - TRANSFER SEMANTIC OBJECTS
         *
         * Semantic objects are transformed into the current-map reference
         * frame before ownership is transferred.
         *
         * Geometry is preserved while map ownership is updated.
         * -------------------------------------------------------------- */
        for (geometric::Plane *p_plane : currentMapPlanes)
        {
            /* Skip invalid planes */
            bool planeIsBad{};
            if (!(p_plane == nullptr) &&
                p_plane->isBad(planeIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_plane == nullptr || planeIsBad)
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                if (p_plane->applyTransform(g2oSwCurrentWMerge) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // applyTransform cannot fail; continue as before.
                }
            }

            /* Update the map the plane belongs to */
            if (p_plane->setMap(p_currentMap) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }

            /*!
             * Take index size of planes in new map to find an id to add to
             * the map which hasn't been taken.
             */
            if (p_plane->setId(nextPlaneId++) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // setId cannot fail; continue as before.
            }

            /* Add the plane to the map new merged plane to the new map */
            p_currentMap->addMapPlane(p_plane);

            /* Remove the current plane from the old map */
            p_mergeMap->eraseMapPlane(p_plane);
        }

        // Loop over the Markers of the current map and move them to the new
        // map
        for (semantic::Marker *p_marker : currentMapMarkers)
        {
            if (!p_marker)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                if (p_marker->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    // applyTransform cannot fail; continue as before.
                }
            }

            if (p_marker->setMap(p_currentMap) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }
            if (p_marker->setId(nextMarkerId++) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // setId cannot fail; continue as before.
            }
            p_currentMap->addMapMarker(p_marker);
            p_mergeMap->eraseMapMarker(p_marker);
        }

        /*!
         * Loop over the passages of the primary map and move them to the
         * secondary map.
         */
        for (vs_graphs::core::semantic::Passage *p_passage : currentMapPassages)
        {
            /* Skip invalid rooms */
            if (p_passage == nullptr)
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                if (p_passage->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // applyTransform cannot fail; continue as before.
                }
            }

            semantic::Passage *p_retainedPassage = nullptr;
            for (semantic::Passage *p_existingPassage :
                 p_currentMap->getAllPassages())
            {
                int existingPassageId{};
                if ((p_existingPassage != nullptr) &&
                    p_existingPassage->getId(existingPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int passageId{};
                if ((p_existingPassage != nullptr) &&
                    p_passage->getId(passageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                if (p_existingPassage != nullptr &&
                    existingPassageId == passageId)
                {
                    p_retainedPassage = p_existingPassage;
                    break;
                }
            }

            p_mergeMap->eraseMapPassage(p_passage);
            if (p_retainedPassage != nullptr)
            {
                bool retainedPassageWasGeometryReplaced{};
                if (p_retainedPassage->mergeFromDuplicate(
                        p_passage,
                        retainedPassageWasGeometryReplaced) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    retainedPassageWasGeometryReplaced =
                        false; // rejected input reads as before
                }
                for (semantic::Room *p_room : currentDetectedMapRooms)
                {
                    if (p_room != nullptr)
                    {
                        bool roomWasAssociationReplaced{};
                        if (p_room->replacePassageAssociation(
                                p_passage,
                                p_retainedPassage,
                                roomWasAssociationReplaced) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            roomWasAssociationReplaced =
                                false; // rejected input reads as before
                        }
                    }
                }
                for (semantic::Room *p_room : currentMarkerBasedMapRooms)
                {
                    if (p_room != nullptr)
                    {
                        bool roomWasAssociationReplaced2{};
                        if (p_room->replacePassageAssociation(
                                p_passage,
                                p_retainedPassage,
                                roomWasAssociationReplaced2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            roomWasAssociationReplaced2 =
                                false; // rejected input reads as before
                        }
                    }
                }
                if (p_passage->setBad() !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // setBad cannot fail; continue as before.
                }
                continue;
            }

            if (p_passage->setMap(p_currentMap) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }
            p_currentMap->addMapPassage(p_passage);
        }

        /*!
         * Loop over the rooms of the primary map and move them to the
         * secondary map.
         */
        for (vs_graphs::core::semantic::Room *p_currentDetectedRoom :
             currentDetectedMapRooms)
        {
            /* Skip invalid rooms */
            bool currentDetectedRoomIsBad{};
            if (!(p_currentDetectedRoom == nullptr) &&
                p_currentDetectedRoom->isBad(currentDetectedRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_currentDetectedRoom == nullptr || currentDetectedRoomIsBad)
            {
                continue;
            }

            /*!
             * Transform the semantic geometry from the old map frame into
             * the merged map frame before changing ownership.
             */
            if (!primarySemanticGeometryWasCorrected)
            {
                if (p_currentDetectedRoom->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // applyTransform cannot fail; continue as before.
                }
            }

            /* Set the map of the room in the current map */
            if (p_currentDetectedRoom->setMap(p_currentMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }

            /* Add the room to the current map */
            p_currentMap->addDetectedMapRoom(p_currentDetectedRoom);

            /* Remove the room from the merged map */
            p_mergeMap->eraseDetectedMapRoom(p_currentDetectedRoom);
        }

        // Loop over the Marker-based Rooms of the current map and move them
        // to the new map
        for (vs_graphs::core::semantic::Room *pRoom :
             currentMarkerBasedMapRooms)
        {
            if (!pRoom)
                continue;

            if (!primarySemanticGeometryWasCorrected)
            {
                if (pRoom->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // applyTransform cannot fail; continue as before.
                }
            }

            if (pRoom->setMap(p_currentMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }
            p_currentMap->addCandidateMapRoom(pRoom);
            p_mergeMap->eraseMarkerBasedMapRoom(pRoom);
        }

        for (semantic::Floor *p_floor : currentMapFloors)
        {
            if (p_floor == nullptr)
            {
                continue;
            }

            if (!semanticGeometryWasPropagated)
            {
                if (p_floor->applyTransform(g2oSwCurrentWMerge) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    // applyTransform cannot fail; continue as before.
                }
            }
            p_mergeMap->eraseMapFloor(p_floor);
            semantic::Floor *p_retainedFloor = nullptr;
            for (semantic::Floor *p_existingFloor :
                 p_currentMap->getAllFloors())
            {
                int existingFloorId{};
                if ((p_existingFloor != nullptr) &&
                    p_existingFloor->getId(existingFloorId) !=
                        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int floorId{};
                if ((p_existingFloor != nullptr) &&
                    p_floor->getId(floorId) !=
                        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                if (p_existingFloor != nullptr && existingFloorId == floorId)
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
            if (p_floor->setMap(p_currentMap) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }
            p_currentMap->addMapFloor(p_floor);
        }

        collapseMergedFloors(p_currentMap);

        /*
         * Voxblox topology is derived from a TSDF/ESDF volume and is not an
         * independently mergeable landmark set. Appending snapshots from two
         * map frames creates disconnected duplicate edges and false wall
         * crossings. The external Voxblox node receives the map-revision event,
         * clears its volume, and supplies a fresh snapshot after reintegration.
         */
        p_currentMap->setSkeletonClusterPoints({});
        p_currentMap->setSkeletonEdges({});

        /* Rebuild imported room-wall index entries before fusion. */
        for (semantic::Room *p_room : p_currentMap->getAllRooms())
        {
            bool roomIsBad{};
            if (!(p_room == nullptr) &&
                p_room->isBad(roomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_room == nullptr || roomIsBad)
            {
                continue;
            }

            std::vector<geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWalls cannot fail; continue as before.
            }
            for (geometric::Plane *p_wall : roomWalls)
            {
                bool wallIsBad{};
                if ((p_wall != nullptr) &&
                    p_wall->isBad(wallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (p_wall != nullptr && !wallIsBad)
                {
                    p_currentMap->addRoomWallPlane(p_wall);
                }
            }
        }

        /* Fuse only after every semantic relationship is visible. */
        if (p_sysParams->semSeg.reassociate.enabled)
        {
            if (utils::utils::Utils::reAssociateSemanticPlanes(p_atlas) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // reAssociateSemanticPlanes cannot fail; continue as before.
            }
        }

        std::vector<semantic::Room *> importedRooms = currentDetectedMapRooms;
        importedRooms.insert(importedRooms.end(),
                             currentMarkerBasedMapRooms.begin(),
                             currentMarkerBasedMapRooms.end());

        /* Stable semantic identity reconciliation is a merge invariant, not
         * an optional geometry-reassociation feature. */
        if (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(p_currentMap,
                                                              importedRooms) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            // fuseDuplicateRoomsAfterMerge cannot fail; continue as before.
        }

        if (p_sysParams->semSeg.reassociate.enabled)
        {
            if (utils::utils::Utils::reAssociateRooms(p_atlas) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // reAssociateRooms cannot fail; continue as before.
            }
            if (utils::utils::Utils::reAssociatePassages(p_atlas) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // reAssociatePassages cannot fail; continue as before.
            }
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

    p_currentMap->increaseChangeIndex();

    /*!
     * A map merge changes the world-frame poses of previously integrated
     * observations. Notify derived mapping consumers only after ownership,
     * semantic reconciliation, and graph connectivity are fully committed.
     * Voxblox uses this revision to discard TSDF/ESDF state expressed in the
     * pre-merge coordinate frame.
     */
    p_currentMap->informNewBigChange();

    /* All surviving objects now belong to pCurrentMap. */
    p_atlas->changeMap(p_currentMap);
    p_atlas->setMapBad(p_mergeMap);
    p_atlas->removeBadMaps();

    std::cout << "[SemanticMergeGate] surviving_map=" << p_currentMap->getId()
              << " absorbed_map=" << p_mergeMap->getId()
              << " decision=ACCEPT reason=ALIGNED"
              << " floor=" << floorVerificationResult << " committed=1"
              << std::endl;

    semanticUpdateLock.unlock();
    p_localMapper->release();

    if (shouldRelaunchBa &&
        (!p_currentMap->isImuInitialized() ||
         (p_currentMap->getKeyFrameCount() < 200 && p_atlas->countMaps() == 1)))
    {
        relaunchGlobalBundleAdjustment(p_currentMap);
    }

    return semantic::SemanticMergeDecision::ACCEPT;
}

} // namespace core
} // namespace vs_graphs
