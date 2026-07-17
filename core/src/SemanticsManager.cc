/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "SemanticsManager.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace ORB_SLAM3
{
SemanticsManager::SemanticsManager(Atlas *pAtlas)
{
    /* Store the address of the atlas map */
    mpAtlas = pAtlas;

    /* Get the system parameters */
    sysParams = SystemParams::GetParams();
}

void SemanticsManager::Run(void)
{
    while (true)
    {
        /* Find the current time of the loop */
        const auto start = std::chrono::high_resolution_clock::now();

        /* Validate the low-level semantic planes */
        Plane *mainGroundPlane = mpAtlas->GetBiggestGroundPlane();

        /* If there is a ground plane, find its transform and filter planes */
        if (mainGroundPlane != nullptr)
        {
            /* Find the transform from ground plane to horizontal */
            mPlanePoseMat = computePlaneToHorizontal(mainGroundPlane);

            /* Filter ground planes */
            filterGroundPlanes(mainGroundPlane);

            /* Filter the wall planes */
            filterWallPlanes();
        }

        /* Reassociate semantic planes after SLAM optimization */
        if (sysParams->sem_seg.reassociate.enabled)
        {
            Utils::reAssociateSemanticPlanes(mpAtlas);
        }

        /*!
         * Use free-space evidence to create and update rooms.
         *
         * @note         This is the preferred wall-to-room association method.
         */
        if (sysParams->room_seg.method ==
            SystemParams::room_seg::Method::FREE_SPACE)
        {
            detectRoom_FreeSpaceCluster();
        }
        else if (sysParams->room_seg.method ==
                 SystemParams::room_seg::Method::GNN)
        {
            detectRoom_GNN();
        }

        /*!
         * Enforce the semantic hierarchy.
         *
         * @note        wall which was not captured by the free-space room
         *              detector receives either an existing room or a new
         *              provisional structural element.
         */
        associateAllWallsToRooms();

        /*!
         * Keep the unsafe centroid-only reAssociateRooms()
         * disabled for now.
         */
        reAssociateRooms();

        /*  Detect passages after room-wall membership is current */
        if (sysParams->sem_seg.enable_passage_detection)
        {
            detectDoorsAndDoorways(mpAtlas);
            updatePassages(mpAtlas);
            associatePassagesToRooms();
        }

        /* Associate every valid room/SE with the floor */
        getUpdatedFloors();

        /* Find the time after it took to run the loop */
        const auto end = std::chrono::high_resolution_clock::now();

        /* Calculate the elapsed time */
        const std::chrono::duration<double> elapsed = end - start;

        /* Find how much longer in the loop is left */
        const double remainingSeconds =
            static_cast<double>(runInterval) - elapsed.count();

        /* If there is remaining time, sleep until next loop cycle */
        if (remainingSeconds > 0.0)
        {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(remainingSeconds));
        }
    }
}

std::vector<std::vector<Eigen::Vector3d>>
    SemanticsManager::getLatestSkeletonCluster(void)
{
    /* Lock the skeleton cluster */
    unique_lock<std::mutex> lock(mMutexNewRooms);

    /* Get the latest skeleton cluster from Atlas */
    return mpAtlas->GetSkeletoClusterPoints();
}

std::vector<ORB_SLAM3::Room *>
    SemanticsManager::getLatestGNNRoomCandidates(void)
{
    // [TODO]
}

void SemanticsManager::filterWallPlanes(void)
{
    /* Iterate through all the planes and filter the walls */
    for (const auto &plane : mpAtlas->GetAllPlanes())
    {
        /* Skip planes which are not classed as walls */
        if (plane->getExpectedPlaneType() ==
            ORB_SLAM3::Plane::planeVariant::WALL)
        {
            /*!
             * Wall validation based on the mPlanePoseMat only works if the
             * ground plane is set. Needs the correction matrix: mPlanePoseMat.
             */
            Eigen::Vector3f transformedPlaneCoefficients =
                transformPlaneEqToGroundReference(
                    plane->getGlobalEquation().coeffs());

            /*!
             * If the transformed plane is vertical based on absolute value,
             * then assign semantic, otherwise ignore threshold should be
             * leniently set (ideally with correct ground plane reference, this
             * value should be close to 0.00)
             */
            if (abs(transformedPlaneCoefficients(1)) >
                sysParams->sem_seg.max_tilt_wall)
            {
                plane->resetPlaneSemantics();
            }
        }
    }
}

void SemanticsManager::filterGroundPlanes(Plane *groundPlane)
{
    // discard ground planes that have height above a threshold from the biggest
    // ground plane [TODO] - Whether to use biggest ground plane or lowest
    // ground plane?

    /* Get the median height of the plane to compute the threshold */
    float threshY = computeGroundPlaneHeight(groundPlane) -
                    sysParams->sem_seg.max_step_elevation;

    /* Extract the main associated ground plane */
    int groundPlaneId = groundPlane->getId();

    /* Go through all ground planes to check validity */
    for (const auto &plane : mpAtlas->GetAllPlanes())
    {
        /* Skip planes not classed as ground, or are the main ground plane */
        if (plane->getExpectedPlaneType() !=
                ORB_SLAM3::Plane::planeVariant::GROUND ||
            plane->getId() == groundPlaneId)
        {
            continue;
        }

        /* If planes above inverted y threshold, then reset plane semantics */
        if (computeGroundPlaneHeight(plane) < threshY)
        {
            plane->resetPlaneSemantics();
            continue;
        }

        /* Find trnsform of the plane */
        Eigen::Vector3f transformedPlaneCoefficients =
            transformPlaneEqToGroundReference(
                plane->getGlobalEquation().coeffs());

        /*!
         * If the transformed plane is horizontal based on absolute value, then
         * assign semantic, otherwise ignore threshold should be leniently set
         * (ideally with correct ground plane reference, this value should be
         * close to 0.00)
         */
        if (abs(transformedPlaneCoefficients(0)) >
            sysParams->sem_seg.max_tilt_ground)
        {
            plane->resetPlaneSemantics();
        }
    }
}
void SemanticsManager::detectOpenPassagesFromSkeletonEdges(
    const std::vector<ORB_SLAM3::Plane *> &wallPlanes)
{
    /* Minimum distance required between each edge endpoint and the wall */
    constexpr double minimumSideDistance = 0.10;

    /* Reject skeleton edges travelling almost parallel to the wall */
    constexpr double minimumEdgeNormalAlignment = 0.35;

    /* Minimum empty radius around the crossing in the wall point cloud */
    constexpr double minimumOpeningRadius = 0.18;

    /* Tolerance around the finite wall-cloud bounds */
    constexpr double wallBoundsMargin = 0.25;

    /* Nearby candidates on parallel walls are treated as the same passage */
    constexpr double duplicatePassageDistance = 0.75;
    constexpr double duplicateNormalAlignment = 0.90;

    /* Reject skeleton crossings at implausible heights */
    constexpr double minimumCrossingHeight = 0.20;
    constexpr double maximumCrossingHeight = 2.30;

    /*!
     * Preferred centre height for an open doorway.
     *
     * The crossing points determine the horizontal position. This height is
     * used when the skeleton crossings do not span enough of the doorway to
     * estimate its vertical centre reliably.
     */
    constexpr double preferredPassageHeight = 1.00;

    /* Restrict the final passage centre to a sensible doorway-centre range */
    constexpr double minimumPassageCentreHeight = 0.75;
    constexpr double maximumPassageCentreHeight = 1.25;

    /*!
     * Use the measured crossing-height midpoint only when the crossings cover
     * a meaningful vertical portion of the doorway.
     */
    constexpr double minimumMeasuredHeightSpan = 0.50;

    /* Extract the latest raw Voxblox sparse-graph edges */
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        skeletonEdges = mpAtlas->GetSkeletonEdges();

    if (skeletonEdges.empty())
    {
        std::cout << "[PassageDebug] No skeleton edges available." << std::endl;

        return;
    }

    /* ---------------------------------------------------------------------- *
     * PREPARE THE GROUND PLANE
     * ---------------------------------------------------------------------- */

    ORB_SLAM3::Plane *groundPlane = mpAtlas->GetBiggestGroundPlane();

    Eigen::Vector4d groundEquation = Eigen::Vector4d::Zero();

    Eigen::Vector3d groundNormal = Eigen::Vector3d::Zero();

    bool hasValidGroundEquation = false;

    if (groundPlane != nullptr && !groundPlane->isBad())
    {
        groundEquation = groundPlane->getGlobalEquation().coeffs();

        const double groundNormalNorm = groundEquation.head<3>().norm();

        if (std::isfinite(groundNormalNorm) && groundNormalNorm > 1e-8)
        {
            groundEquation /= groundNormalNorm;

            groundNormal = groundEquation.head<3>();

            hasValidGroundEquation = true;
        }
    }

    /* ---------------------------------------------------------------------- *
     * PASSAGE CANDIDATE TYPE
     * ---------------------------------------------------------------------- */

    struct PassageCandidate
    {
        ORB_SLAM3::Plane *wall = nullptr;

        Eigen::Vector3d crossingPoint = Eigen::Vector3d::Zero();

        double      openingRadius     = 0.0;
        double      edgeWallAlignment = 0.0;
        std::size_t crossingCount     = 0;
    };

    std::vector<PassageCandidate> passageCandidates;

    passageCandidates.reserve(wallPlanes.size());

    /* ---------------------------------------------------------------------- *
     * FIND CROSSINGS FOR EACH WALL
     * ---------------------------------------------------------------------- */

    for (ORB_SLAM3::Plane *wall : wallPlanes)
    {
        if (wall == nullptr || wall->isBad())
        {
            continue;
        }

        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud =
            wall->getMapClouds();

        if (wallCloud == nullptr || wallCloud->empty())
        {
            continue;
        }

        /* Extract and normalise the wall equation */
        Eigen::Vector4d wallEquation = wall->getGlobalEquation().coeffs();

        const double wallNormalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(wallNormalNorm) || wallNormalNorm < 1e-8)
        {
            continue;
        }

        wallEquation /= wallNormalNorm;

        const Eigen::Vector3d wallNormal = wallEquation.head<3>();

        const Eigen::Vector3d wallCentroid = wall->getCentroid().cast<double>();

        /*
         * Construct two axes lying inside the wall plane.
         */
        const Eigen::Vector3d wallAxisU =
            wallNormal.unitOrthogonal().normalized();

        const Eigen::Vector3d wallAxisV =
            wallNormal.cross(wallAxisU).normalized();

        double minimumU = std::numeric_limits<double>::max();

        double maximumU = std::numeric_limits<double>::lowest();

        double minimumV = std::numeric_limits<double>::max();

        double maximumV = std::numeric_limits<double>::lowest();

        std::size_t validWallPointCount = 0;

        /* Calculate the finite wall bounds */
        for (const pcl::PointXYZRGBA &point : wallCloud->points)
        {
            if (!pcl::isFinite(point))
            {
                continue;
            }

            const Eigen::Vector3d wallPoint(static_cast<double>(point.x),
                                            static_cast<double>(point.y),
                                            static_cast<double>(point.z));

            const Eigen::Vector3d relativePoint = wallPoint - wallCentroid;

            const double coordinateU = relativePoint.dot(wallAxisU);

            const double coordinateV = relativePoint.dot(wallAxisV);

            minimumU = std::min(minimumU, coordinateU);

            maximumU = std::max(maximumU, coordinateU);

            minimumV = std::min(minimumV, coordinateV);

            maximumV = std::max(maximumV, coordinateV);

            validWallPointCount++;
        }

        if (validWallPointCount == 0)
        {
            continue;
        }

        /* Store every accepted skeleton breach through this wall */
        std::vector<Eigen::Vector3d> acceptedCrossingPoints;

        acceptedCrossingPoints.reserve(skeletonEdges.size());

        double maximumOpeningRadius = 0.0;
        double edgeAlignmentSum     = 0.0;

        std::size_t infinitePlaneCrossings = 0;
        std::size_t finiteWallCrossings    = 0;
        std::size_t openingCrossings       = 0;

        /* Test every skeleton edge against the current wall */
        for (const auto &skeletonEdge : skeletonEdges)
        {
            const Eigen::Vector3d &edgeStart = skeletonEdge.first;

            const Eigen::Vector3d &edgeEnd = skeletonEdge.second;

            if (!edgeStart.allFinite() || !edgeEnd.allFinite())
            {
                continue;
            }

            const Eigen::Vector3d edgeVector = edgeEnd - edgeStart;

            const double edgeLength = edgeVector.norm();

            if (!std::isfinite(edgeLength) || edgeLength < 1e-6)
            {
                continue;
            }

            const Eigen::Vector3d edgeDirection = edgeVector / edgeLength;

            /* Reject edges which merely graze the wall */
            const double edgeNormalAlignment =
                std::abs(edgeDirection.dot(wallNormal));

            if (edgeNormalAlignment < minimumEdgeNormalAlignment)
            {
                continue;
            }

            const double startDistance =
                wallNormal.dot(edgeStart) + wallEquation(3);

            const double endDistance =
                wallNormal.dot(edgeEnd) + wallEquation(3);

            const bool crossesPlane = (startDistance <= -minimumSideDistance &&
                                       endDistance >= minimumSideDistance) ||
                                      (endDistance <= -minimumSideDistance &&
                                       startDistance >= minimumSideDistance);

            if (!crossesPlane)
            {
                continue;
            }

            infinitePlaneCrossings++;

            const double denominator = startDistance - endDistance;

            if (std::abs(denominator) < 1e-8)
            {
                continue;
            }

            const double interpolation = startDistance / denominator;

            if (interpolation < 0.0 || interpolation > 1.0)
            {
                continue;
            }

            Eigen::Vector3d crossingPoint =
                edgeStart + interpolation * edgeVector;

            /* Project exactly onto the supporting wall */
            const double planeResidual =
                wallNormal.dot(crossingPoint) + wallEquation(3);

            crossingPoint -= planeResidual * wallNormal;

            /* Reject crossings which are too close to the ground or too high */
            if (hasValidGroundEquation)
            {
                const double crossingHeight = std::abs(
                    groundNormal.dot(crossingPoint) + groundEquation(3));

                if (!std::isfinite(crossingHeight) ||
                    crossingHeight < minimumCrossingHeight ||
                    crossingHeight > maximumCrossingHeight)
                {
                    continue;
                }
            }

            /* Check the finite wall bounds */
            const Eigen::Vector3d relativeCrossing =
                crossingPoint - wallCentroid;

            const double crossingU = relativeCrossing.dot(wallAxisU);

            const double crossingV = relativeCrossing.dot(wallAxisV);

            const bool insideFiniteWall =
                crossingU >= minimumU - wallBoundsMargin &&
                crossingU <= maximumU + wallBoundsMargin &&
                crossingV >= minimumV - wallBoundsMargin &&
                crossingV <= maximumV + wallBoundsMargin;

            if (!insideFiniteWall)
            {
                continue;
            }

            finiteWallCrossings++;

            /*!
             * Measure the nearest wall point using only displacement inside the
             * wall surface.
             */
            double nearestWallPointDistance =
                std::numeric_limits<double>::max();

            for (const pcl::PointXYZRGBA &wallPointPcl : wallCloud->points)
            {
                if (!pcl::isFinite(wallPointPcl))
                {
                    continue;
                }

                const Eigen::Vector3d wallPoint(
                    static_cast<double>(wallPointPcl.x),
                    static_cast<double>(wallPointPcl.y),
                    static_cast<double>(wallPointPcl.z));

                const Eigen::Vector3d difference = wallPoint - crossingPoint;

                const Eigen::Vector3d inPlaneDifference =
                    difference - difference.dot(wallNormal) * wallNormal;

                nearestWallPointDistance = std::min(nearestWallPointDistance,
                                                    inPlaneDifference.norm());
            }

            /* Reject crossings through populated wall regions */
            if (!std::isfinite(nearestWallPointDistance) ||
                nearestWallPointDistance < minimumOpeningRadius)
            {
                continue;
            }

            openingCrossings++;

            acceptedCrossingPoints.push_back(crossingPoint);

            maximumOpeningRadius =
                std::max(maximumOpeningRadius, nearestWallPointDistance);

            edgeAlignmentSum += edgeNormalAlignment;
        }

        std::cout << "[PassageDebug] Wall#" << wall->getId()
                  << ": plane crossings=" << infinitePlaneCrossings
                  << ", finite crossings=" << finiteWallCrossings
                  << ", opening crossings=" << openingCrossings << "."
                  << std::endl;

        if (acceptedCrossingPoints.empty())
        {
            continue;
        }

        /* ------------------------------------------------------------------ *
         * CALCULATE THE DOORWAY CENTRE
         * ------------------------------------------------------------------ */

        /*!
         * Start with the ordinary mean of all crossing points. This provides
         * the horizontal position of the opening.
         */
        Eigen::Vector3d passageCentre = Eigen::Vector3d::Zero();

        for (const Eigen::Vector3d &crossingPoint : acceptedCrossingPoints)
        {
            passageCentre += crossingPoint;
        }

        passageCentre /= static_cast<double>(acceptedCrossingPoints.size());

        double selectedPassageHeight = preferredPassageHeight;

        if (hasValidGroundEquation)
        {
            double minimumMeasuredHeight = std::numeric_limits<double>::max();

            double maximumMeasuredHeight =
                std::numeric_limits<double>::lowest();

            for (const Eigen::Vector3d &crossingPoint : acceptedCrossingPoints)
            {
                const double measuredHeight = std::abs(
                    groundNormal.dot(crossingPoint) + groundEquation(3));

                minimumMeasuredHeight =
                    std::min(minimumMeasuredHeight, measuredHeight);

                maximumMeasuredHeight =
                    std::max(maximumMeasuredHeight, measuredHeight);
            }

            const double measuredHeightSpan =
                maximumMeasuredHeight - minimumMeasuredHeight;

            /*!
             * Use the measured midpoint only when the crossings cover enough
             * vertical space. Otherwise use the preferred doorway-centre
             * height.
             */
            if (acceptedCrossingPoints.size() > 1 &&
                measuredHeightSpan >= minimumMeasuredHeightSpan)
            {
                selectedPassageHeight =
                    0.5 * (minimumMeasuredHeight + maximumMeasuredHeight);
            }

            /*!
             * Prevent an unusually low or high group of skeleton crossings
             * from placing the passage marker near the floor or lintel.
             */
            selectedPassageHeight = std::max(
                minimumPassageCentreHeight,
                std::min(selectedPassageHeight, maximumPassageCentreHeight));

            /*!
             * Determine which side of the ground plane represents the mapped
             * room and wall.
             */
            double aboveGroundSign =
                groundNormal.dot(wallCentroid) + groundEquation(3);

            if (std::abs(aboveGroundSign) < 1e-6)
            {
                aboveGroundSign =
                    groundNormal.dot(passageCentre) + groundEquation(3);
            }

            aboveGroundSign = aboveGroundSign >= 0.0 ? 1.0 : -1.0;

            const double currentSignedHeight =
                groundNormal.dot(passageCentre) + groundEquation(3);

            const double desiredSignedHeight =
                aboveGroundSign * selectedPassageHeight;

            /*!
             * Move only along the ground normal. The averaged horizontal
             * doorway position remains unchanged.
             */
            passageCentre +=
                (desiredSignedHeight - currentSignedHeight) * groundNormal;
        }

        /* Ensure the final centre lies exactly on the wall */
        const double finalPlaneResidual =
            wallNormal.dot(passageCentre) + wallEquation(3);

        passageCentre -= finalPlaneResidual * wallNormal;

        PassageCandidate candidate;

        candidate.wall = wall;

        candidate.crossingPoint = passageCentre;

        candidate.openingRadius = maximumOpeningRadius;

        candidate.edgeWallAlignment =
            edgeAlignmentSum /
            static_cast<double>(acceptedCrossingPoints.size());

        candidate.crossingCount = acceptedCrossingPoints.size();

        passageCandidates.push_back(candidate);

        std::cout << "[PassageDebug] Wall#" << wall->getId()
                  << " produced passage centre " << passageCentre.transpose()
                  << " at height " << selectedPassageHeight << " m from "
                  << acceptedCrossingPoints.size() << " crossings."
                  << std::endl;
    }

    if (passageCandidates.empty())
    {
        std::cout << "[PassageDebug] No valid passage candidates found."
                  << std::endl;

        return;
    }

    /*
     * Process the clearest candidates first.
     */
    std::sort(passageCandidates.begin(),
              passageCandidates.end(),
              [](const PassageCandidate &first, const PassageCandidate &second)
              { return first.openingRadius > second.openingRadius; });

    /* ---------------------------------------------------------------------- *
     * UPDATE EXISTING PASSAGES OR CREATE NEW ONES
     * ---------------------------------------------------------------------- */

    for (const PassageCandidate &candidate : passageCandidates)
    {
        if (candidate.wall == nullptr || candidate.wall->isBad())
        {
            continue;
        }

        Eigen::Vector3d candidateNormal =
            candidate.wall->getGlobalEquation().normal();

        if (!candidateNormal.allFinite() || candidateNormal.norm() < 1e-8)
        {
            continue;
        }

        candidateNormal.normalize();

        ORB_SLAM3::Passage *matchingPassage = nullptr;

        const std::vector<ORB_SLAM3::Passage *> existingPassages =
            mpAtlas->GetAllPassages();

        for (ORB_SLAM3::Passage *existingPassage : existingPassages)
        {
            if (existingPassage == nullptr || !existingPassage->isPassable())
            {
                continue;
            }

            const Eigen::Vector3d existingCentroid =
                existingPassage->getCentroid().cast<double>();

            if (!existingCentroid.allFinite())
            {
                continue;
            }

            Eigen::Vector3d centroidDifference =
                existingCentroid - candidate.crossingPoint;

            /* Ignore vertical displacement during duplicate comparison */
            if (hasValidGroundEquation)
            {
                centroidDifference -=
                    centroidDifference.dot(groundNormal) * groundNormal;
            }

            const double horizontalDistance = centroidDifference.norm();

            if (horizontalDistance > duplicatePassageDistance)
            {
                continue;
            }

            Eigen::Vector3d existingNormal =
                existingPassage->getGlobalEquation().normal();

            if (existingNormal.allFinite() && existingNormal.norm() > 1e-8)
            {
                existingNormal.normalize();

                const double normalAlignment =
                    std::abs(existingNormal.dot(candidateNormal));

                if (normalAlignment < duplicateNormalAlignment)
                {
                    continue;
                }
            }

            matchingPassage = existingPassage;

            break;
        }

        /*!
         * Update an existing passage so a previously created low marker moves
         * to the corrected doorway centre.
         */
        if (matchingPassage != nullptr)
        {
            matchingPassage->setCentroid(candidate.crossingPoint.cast<float>());

            matchingPassage->setGlobalEquation(
                candidate.wall->getGlobalEquation());

            std::cout << "[PassageDebug] Updated Passage#"
                      << matchingPassage->getId() << " to "
                      << candidate.crossingPoint.transpose() << "."
                      << std::endl;

            continue;
        }

        /* Create a new passage */
        GeoSemHelpers::createMapPassage(mpAtlas,
                                        nullptr,
                                        candidate.wall,
                                        true,
                                        candidate.crossingPoint.cast<float>());

        std::cout << "[PassageDebug] Created Passage on Wall#"
                  << candidate.wall->getId() << " at "
                  << candidate.crossingPoint.transpose()
                  << ", crossings=" << candidate.crossingCount
                  << ", opening radius=" << candidate.openingRadius << " m."
                  << std::endl;
    }
}

void SemanticsManager::detectDoorsAndDoorways(ORB_SLAM3::Atlas *pAtlas)
{
    /* Confirm that the Atlas is valid */
    if (pAtlas == nullptr)
    {
        return;
    }

    /* Extract all planes from the current map */
    const std::vector<ORB_SLAM3::Plane *> allPlanes = pAtlas->GetAllPlanes();

    /* Initialise lists of valid wall and door planes */
    std::vector<ORB_SLAM3::Plane *> wallPlanes;
    std::vector<ORB_SLAM3::Plane *> doorPlanes;

    wallPlanes.reserve(allPlanes.size());
    doorPlanes.reserve(allPlanes.size());

    /* Separate mapped planes according to their confirmed semantic type */
    for (ORB_SLAM3::Plane *plane : allPlanes)
    {
        /* Skip invalid mapped planes */
        if (plane == nullptr || plane->isBad())
        {
            continue;
        }

        /* Store confirmed wall planes */
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
        {
            wallPlanes.push_back(plane);
            continue;
        }

        /* Store confirmed door planes */
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::DOOR)
        {
            doorPlanes.push_back(plane);
        }
    }

    /*  Detect blocked passages represented by closed semantic door planes */
    for (ORB_SLAM3::Plane *door : doorPlanes)
    {
        /* Skip invalid door planes */
        if (door == nullptr || door->isBad())
        {
            continue;
        }

        for (ORB_SLAM3::Plane *wall : wallPlanes)
        {
            /* Skip invalid wall planes */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /* Door and wall must be parallel */
            if (!Utils::arePlanesParallel(door, wall))
            {
                continue;
            }

            /* Door and wall must not be opposing surfaces */
            if (Utils::arePlanesFacingEachOther(door, wall))
            {
                continue;
            }

            /* Door must lie close to the supporting wall */
            if (Utils::arePlanesApartEnough(
                    door,
                    wall,
                    sysParams->sem_seg.max_wall_door_distance))
            {
                continue;
            }

            /* Create a blocked passage associated with the supporting wall */
            GeoSemHelpers::createMapPassage(mpAtlas, door, wall, false);
        }
    }

    /*!
     * Detect open passages from connected Voxblox skeleton edges which breach
     * finite mapped wall surfaces.
     *
     * @note        Camera trajectory crossings are deliberately not used.
     */
    detectOpenPassagesFromSkeletonEdges(wallPlanes);

    std::cout << "[PassageDebug] Atlas currently contains "
              << mpAtlas->GetAllPassages().size() << " passages." << std::endl;
}

void SemanticsManager::updatePassages(ORB_SLAM3::Atlas *pAtlas)
{
    // Get the ground plane
    ORB_SLAM3::Plane *groundPlane = pAtlas->GetBiggestGroundPlane();
    if (groundPlane == nullptr)
        return;

    // Get all passages and update their global pose to be consistent with the
    // ground plane
    std::vector<ORB_SLAM3::Passage *> allPassages = pAtlas->GetAllPassages();

    for (const auto &passage : allPassages)
    {
        // Get the passage variant (doorway or undefined)
        ORB_SLAM3::Passage::passageVariant variant = passage->getPassageType();

        // Updating the dimensions of the passage based on the associated door
        // plane
        ORB_SLAM3::Plane *doorPlane = passage->getAssociateDoor();

        // Blocked passages (closed doors) should be aligned with the ground
        // plane normal
        if (!passage->isPassable())
        {
            if (doorPlane == nullptr)
            {
                continue;
            }

            /* Extract width height supple of door */
            std::pair<double, double> widthHeight =
                Utils::computePlaneWidthHeight(doorPlane->getMapClouds());

            /* Extract the measured height and width */
            const double measuredWidth  = widthHeight.first;
            const double measuredHeight = widthHeight.second;

            /* Extract the max width */
            const double maxWidth =
                static_cast<double>(sysParams->sem_seg.max_door_width);

            /* Extract the max height */
            const double maxHeight =
                static_cast<double>(sysParams->sem_seg.max_door_height);

            /* Determine if dimensions are valid */
            const bool validDimensions =
                std::isfinite(measuredWidth) && std::isfinite(measuredHeight) &&
                measuredWidth > 0.0 && measuredHeight > 0.0 &&
                measuredWidth <= maxWidth && measuredHeight <= maxHeight;

            if (!validDimensions)
            {
                std::cout << "[SemanticsManager] Rejecting door plane "
                          << doorPlane->getId() << " for passage "
                          << passage->getId() << ": measured dimensions "
                          << measuredWidth << "x" << measuredHeight
                          << " m exceed limits " << maxWidth << "x" << maxHeight
                          << " m." << std::endl;

                continue;
            }

            /* Set centroid of the door plane */
            passage->setCentroid(doorPlane->getCentroid());

            /* Get the plane global equation */
            passage->setGlobalEquation(doorPlane->getGlobalEquation());

            /* Set width & height of the passage */
            passage->setWidth(measuredWidth);
            passage->setHeight(measuredHeight);
        }
        else
        {
            ORB_SLAM3::Plane passagePlane;
            passagePlane.setGlobalEquation(passage->getGlobalEquation());
            if (!Utils::arePlanesPerpendicular(&passagePlane, groundPlane))
            {
                // Project the passage normal onto the horizontal plane to
                // remove tilt
                const Eigen::Vector3d groundNormal =
                    groundPlane->getGlobalEquation()
                        .coeffs()
                        .head<3>()
                        .normalized();
                Eigen::Vector4d globalEq =
                    passage->getGlobalEquation().coeffs();
                Eigen::Vector3d passageNormal = globalEq.head<3>().normalized();

                Eigen::Vector3d correctedNormal =
                    (passageNormal -
                     passageNormal.dot(groundNormal) * groundNormal)
                        .normalized();
                if (correctedNormal.norm() < 1e-6)
                    continue;
                correctedNormal.normalize();

                // Recompute d so the plane still passes through the centroid
                const Eigen::Vector3d centroid =
                    passage->getCentroid().cast<double>();
                const double d = -correctedNormal.dot(centroid);

                Eigen::Vector4d correctedCoeffs;
                correctedCoeffs.head<3>() = correctedNormal;
                correctedCoeffs(3)        = d;

                passage->setGlobalEquation(g2o::Plane3D(correctedCoeffs));
            }
        }
    }
}

Eigen::Vector3f SemanticsManager::transformPlaneEqToGroundReference(
    const Eigen::Vector4d &planeEq)
{
    /* extract the rotation matrix from the transformation matrix */
    Eigen::Matrix3f rotationMatrix = mPlanePoseMat.block<3, 3>(0, 0);

    /* Compute the inverse transpose of the rotation matrix */
    Eigen::Matrix3f inverseTransposeRotationMatrix =
        rotationMatrix.inverse().transpose();

    /* Transform the coefficients of the plane equation */
    Eigen::Vector3f transformedPlaneCoefficients =
        inverseTransposeRotationMatrix * planeEq.head<3>().cast<float>();

    /* Find the normalized coefficients */
    transformedPlaneCoefficients.normalize();

    return transformedPlaneCoefficients;
}

float SemanticsManager::computeGroundPlaneHeight(Plane *groundPlane)
{
    /* Transform the planeCloud according to the planePose */
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud =
        groundPlane->getMapClouds();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*planeCloud, *transformedCloud, mPlanePoseMat);

    /* get the median height of the plane */
    std::vector<float> yVals;
    for (const auto &point : transformedCloud->points)
    {
        yVals.push_back(point.y);
    }

    size_t numPoint = yVals.size() / 2;

    std::partial_sort(yVals.begin(),
                      yVals.begin() + numPoint,
                      yVals.end(),
                      std::greater<float>());

    return yVals[numPoint - 1];
}

Eigen::Matrix4f SemanticsManager::computePlaneToHorizontal(const Plane *plane)
{
    // initialize the transformation with translation set to a zero vector
    Eigen::Isometry3d planePose;
    planePose.translation() = Eigen::Vector3d(0, 0, 0);

    // normalize the normal vector
    Eigen::Vector3d normal = plane->getGlobalEquation().coeffs().head<3>();

    // get the rotation from the ground plane to the plane with y-facing
    // vertical downwards
    Eigen::Vector3d    verticalAxis = Eigen::Vector3d(0, -1, 0);
    Eigen::Quaterniond q;
    q.setFromTwoVectors(normal, verticalAxis);
    planePose.linear() = q.toRotationMatrix();

    // form homogenous transformation matrix
    Eigen::Matrix4f planePoseMat = planePose.matrix().cast<float>();
    planePoseMat(3, 3)           = 1.0;

    return planePoseMat;
}

void SemanticsManager::detectRoom_FreeSpaceCluster(void)
{
    /*!
     * @brief       The minimum number of skeleton points required to support a
     *              wall.
     */
    constexpr std::size_t minimumSupportedPointCount = 3;

    /*!
     * @brief       Of the skeleton points close to the infinite wall plane,
     *              this ratio must project inside the finite observed wall
     *              boundaries.
     */
    constexpr double minimumFiniteSupportRatio = 0.50;

    /*!
     * @brief       Additional tolerance around the finite wall point-cloud
     *              boundaries.
     */
    constexpr double wallBoundsMargin = 0.75;

    /*!
     * @brief       Enables detailed wall-to-cluster association logging.
     */
    constexpr bool enableDebugOutput = true;

    /* Extract latest skeleton cluster */
    const std::vector<std::vector<Eigen::Vector3d>> clusters =
        getLatestSkeletonCluster();

    /* If cluster is empty then return */
    if (clusters.empty())
    {
        if (enableDebugOutput)
        {
            std::cout << "[SemMgr] No free-space clusters available."
                      << std::endl;
        }

        return;
    }

    /* Extract all planes from the map */
    const std::vector<ORB_SLAM3::Plane *> allPlanes = mpAtlas->GetAllPlanes();

    /* Create a list of all the walls there are in the SGraph */
    std::vector<ORB_SLAM3::Plane *> allWalls;
    allWalls.reserve(allPlanes.size());

    /* For every plane, extract walls */
    for (ORB_SLAM3::Plane *plane : allPlanes)
    {
        /* Skip bad planes */
        if (plane == nullptr || plane->isBad())
        {
            continue;
        }

        /* Append valid wall planes to list */
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
        {
            allWalls.push_back(plane);
        }
    }

    if (enableDebugOutput)
    {
        std::cout << "[SemMgr] Room detection: " << clusters.size()
                  << " clusters, " << allPlanes.size() << " planes, "
                  << allWalls.size() << " usable walls." << std::endl;
    }

    /* If there are no walls, then return */
    if (allWalls.empty())
    {
        return;
    }

    /* Iterate through all the clusters */
    for (std::size_t clusterId = 0; clusterId < clusters.size(); clusterId++)
    {
        /* Extract cluster */
        const std::vector<Eigen::Vector3d> &cluster = clusters[clusterId];

        /* If cluster is empty skip */
        if (cluster.empty())
        {
            continue;
        }

        /* Extract the cluster centroid */
        const Eigen::Vector3d clusterCentroid =
            Utils::computeCentroidFromPoints(cluster);

        /* Initialize a list of planes to track the closest walls */
        std::vector<ORB_SLAM3::Plane *> closestWalls;
        closestWalls.reserve(allWalls.size());

        /* For each wall */
        for (ORB_SLAM3::Plane *wall : allWalls)
        {
            /* Skip wall if it is bad */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /* Extract the point cloud for the wall */
            const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud =
                wall->getMapClouds();

            /* Skip wall if the point cloud is invalid */
            if (wallCloud == nullptr || wallCloud->empty())
            {
                continue;
            }

            /* Extract the centroid of the wall */
            const Eigen::Vector3d wallCentroid =
                wall->getCentroid().cast<double>();

            /* Find the distance from the wall centroid and cluster centroid */
            const double centroidDistance =
                (wallCentroid - clusterCentroid).norm();

            /*!
             * Use centroid distance only as a coarse rejection condition.
             *
             * @note        A long wall may have a centroid far from the room
             *              centre while still forming a valid boundary of the
             *              room.
             */
            const double coarseCentroidDistanceThreshold =
                2.0 * static_cast<double>(
                          sysParams->room_seg
                              .cluster_centroid_wall_centroid_distance_thresh);

            if (centroidDistance >= coarseCentroidDistanceThreshold)
            {
                continue;
            }

            /* Extract plane equation for the wall */
            Eigen::Vector4d wallEquation = wall->getGlobalEquation().coeffs();

            /* Extract the norm of the normal of the wall */
            const double normalNorm = wallEquation.head<3>().norm();

            /* Skip wall if norm is invalud */
            if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
            {
                continue;
            }

            /* Find the normal vector of the wall */
            const Eigen::Vector3d normal = wallEquation.head<3>() / normalNorm;

            /* Find the normalized distance */
            const double normalizedD = wallEquation(3) / normalNorm;

            /* Find an axis U which tangental to the wall plane */
            const Eigen::Vector3d axisU = normal.unitOrthogonal().normalized();

            /* Find orthogonal axis to make handed axis with U and normal */
            const Eigen::Vector3d axisV = normal.cross(axisU).normalized();

            /* Init variables for tracking the size of the wall */
            double minU = std::numeric_limits<double>::max();
            double maxU = std::numeric_limits<double>::lowest();
            double minV = std::numeric_limits<double>::max();
            double maxV = std::numeric_limits<double>::lowest();

            /* Init counter to track number of calid wall points */
            std::size_t validWallPoints = 0;

            /* Iterate through each point in the wall point cloud */
            for (const pcl::PointXYZRGBA &point : wallCloud->points)
            {
                /* If point is invalid, skip */
                if (!pcl::isFinite(point))
                {
                    continue;
                }

                /* Create an eigen vector of the wall point */
                const Eigen::Vector3d wallPoint(static_cast<double>(point.x),
                                                static_cast<double>(point.y),
                                                static_cast<double>(point.z));

                /* Find point relative to the centroid */
                const Eigen::Vector3d relativePoint = wallPoint - wallCentroid;

                /* Find the ponts coordinates in the U axis */
                const double coordinateU = relativePoint.dot(axisU);

                /* Find the ponts coordinates in the V axis */
                const double coordinateV = relativePoint.dot(axisV);

                /* Update boundaries of the wall in U axis */
                minU = std::min(minU, coordinateU);
                maxU = std::max(maxU, coordinateU);

                /* Update boundaries of the wall in V axis */
                minV = std::min(minV, coordinateV);
                maxV = std::max(maxV, coordinateV);

                validWallPoints++;
            }

            /* If there are no valid points, skip wall */
            if (validWallPoints == 0)
            {
                continue;
            }

            /* Number of cluster points close enough to infinite wall plane */
            std::size_t nearPlanePointCount = 0;

            /*!
             * Number of close points whose projections lie inside the finite
             * wall patch.
             */
            std::size_t supportedPointCount = 0;

            /* Init variable to track minimum distance from wall and cluster */
            double minimumPlaneDistance = std::numeric_limits<double>::max();

            /* Iterate through each point in cluster to find point in wall */
            for (const Eigen::Vector3d &clusterPoint : cluster)
            {
                /* Find the signed plane distance */
                const double signedPlaneDistance =
                    normal.dot(clusterPoint) + normalizedD;

                /* Find the absolute value of the plane distance */
                const double planeDistance = std::abs(signedPlaneDistance);

                /* Update if the disatance is smaller than currently tracked */
                minimumPlaneDistance =
                    std::min(minimumPlaneDistance, planeDistance);

                /* If the plane distance is larger than threshold, skip point */
                if (planeDistance >=
                    sysParams->room_seg.cluster_point_wall_distance_thresh)
                {
                    continue;
                }

                /*!
                 * The point is close enough to the infinite plane to
                 * participate in the finite-wall support calculation.
                 */
                nearPlanePointCount++;

                /* Find the projected point on the plane */
                const Eigen::Vector3d projectedPoint =
                    clusterPoint - signedPlaneDistance * normal;

                /* Find relative distance between point and wall centroid */
                const Eigen::Vector3d projectedRelative =
                    projectedPoint - wallCentroid;

                /* Find distance in U axis on wall */
                const double projectedU = projectedRelative.dot(axisU);

                /* Find distance in V axis on wall */
                const double projectedV = projectedRelative.dot(axisV);

                /* Confirm if the wall encapsulates the projected point */
                const bool insideFiniteWall =
                    projectedU >= minU - wallBoundsMargin &&
                    projectedU <= maxU + wallBoundsMargin &&
                    projectedV >= minV - wallBoundsMargin &&
                    projectedV <= maxV + wallBoundsMargin;

                /* If the point is within the wall plane, incriment support */
                if (insideFiniteWall)
                {
                    supportedPointCount++;
                }
            }

            /*!
             * Calculate support relative only to points which are close to the
             * wall plane.
             *
             * @note        Dividing by the complete room cluster unfairly
             *              penalises valid walls as the room becomes more fully
             *              explored.
             */
            const double finiteSupportRatio =
                nearPlanePointCount > 0
                    ? static_cast<double>(supportedPointCount) /
                          static_cast<double>(nearPlanePointCount)
                    : 0.0;

            if (enableDebugOutput)
            {
                std::cout << "[SemMgr] Cluster " << clusterId << ", wall "
                          << wall->getId()
                          << ": centroid distance=" << centroidDistance
                          << " m, minimum plane distance="
                          << minimumPlaneDistance
                          << " m, near-plane points=" << nearPlanePointCount
                          << ", finite support=" << supportedPointCount << "/"
                          << nearPlanePointCount << " ("
                          << finiteSupportRatio * 100.0 << "%)." << std::endl;
            }

            /*!
             * Accept the wall only when there is sufficient absolute support
             * and the majority of nearby points project inside the finite wall
             * patch.
             */
            const bool wallSupportedByCluster =
                supportedPointCount >= minimumSupportedPointCount &&
                finiteSupportRatio >= minimumFiniteSupportRatio;

            if (wallSupportedByCluster)
            {
                closestWalls.push_back(wall);
            }
        }

        /* Organise closest walls in order of ids */
        std::sort(closestWalls.begin(),
                  closestWalls.end(),
                  [](ORB_SLAM3::Plane *first, ORB_SLAM3::Plane *second)
                  { return first->getId() < second->getId(); });

        /* Remove duplicate walls using IDs rather than pointers */
        closestWalls.erase(
            std::unique(closestWalls.begin(),
                        closestWalls.end(),
                        [](ORB_SLAM3::Plane *first, ORB_SLAM3::Plane *second)
                        { return first->getId() == second->getId(); }),
            closestWalls.end());

        if (enableDebugOutput)
        {
            std::cout << "[SemMgr] Cluster " << clusterId << " has "
                      << closestWalls.size() << " candidate walls."
                      << std::endl;
        }

        /* If there are no closest walls then skip to next cluster */
        if (closestWalls.empty())
        {
            continue;
        }

        /* Try to update an existing room first */
        ORB_SLAM3::Room *room = associateRooms(clusterCentroid, closestWalls);

        /*!
         * If no existing room describes this free-space cluster.
         * Create one.
         *
         * Do not remove walls that are globally registered.
         * Walls may be shared between adjacent rooms.
         */
        if (room == nullptr)
        {
            room = GeoSemHelpers::createBlankRoomCandidate(mpAtlas,
                                                           clusterCentroid);

            if (room == nullptr)
            {
                std::cerr << "[SemMgr] Failed to create room "
                             "candidate for cluster "
                          << clusterId << "." << std::endl;

                continue;
            }

            mpAtlas->AddCandidateMapRoom(room);

            std::cout << "[SemMgr] Created room candidate SE#" << room->getId()
                      << " for cluster " << clusterId << "." << std::endl;
        }
        else if (enableDebugOutput)
        {
            std::cout << "[SemMgr] Cluster " << clusterId
                      << " associated with existing SE#" << room->getId() << "."
                      << std::endl;
        }

        /* The skeleton/free-space centroid is the semantic room centre */
        room->setCentroid(clusterCentroid);

        /* Find the walls of the room */
        std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

        for (ORB_SLAM3::Plane *wall : closestWalls)
        {
            /* Skip invalid walls */
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            /* Check to see if closest wall is in any of the current rooms */
            const bool alreadyInRoom =
                std::any_of(roomWalls.begin(),
                            roomWalls.end(),
                            [wall](ORB_SLAM3::Plane *existingWall) {
                                return existingWall != nullptr &&
                                       existingWall->getId() == wall->getId();
                            });

            /* If the wall is already in a room, skip to next slosest wall */
            if (alreadyInRoom)
            {
                continue;
            }

            /*  Add the relationship to the Room */
            room->setWalls(wall);
            roomWalls.push_back(wall);

            /* Atlas registration is not exclusive ownership */
            if (mpAtlas->GetRoomWallPlaneById(wall->getId()) == nullptr)
            {
                mpAtlas->AddRoomWallPlane(wall);
            }

            if (enableDebugOutput)
            {
                std::cout << "[SemMgr] Added wall " << wall->getId()
                          << " to SE#" << room->getId() << "." << std::endl;
            }
        }

        /* Find all the walls in a room */
        roomWalls = room->getWalls();

        /*!
         * Consolidate provisional single-wall structural elements whose wall
         * has now been absorbed into the cluster-backed room.
         *
         * @note        This is deliberately more restrictive than the old
         *              centroid-only reAssociateRooms() implementation.
         */
        consolidateProvisionalRooms(room);

        /* Remove any invalid walls from the room */
        roomWalls.erase(std::remove_if(roomWalls.begin(),
                                       roomWalls.end(),
                                       [](ORB_SLAM3::Plane *wall) {
                                           return wall == nullptr ||
                                                  wall->isBad();
                                       }),
                        roomWalls.end());

        if (enableDebugOutput)
        {
            std::cout << "[SemMgr] SE#" << room->getId() << " currently has "
                      << roomWalls.size() << " valid walls." << std::endl;
        }

        /*!
         * Confirm the room from its free-space cluster.
         *
         * @note        A room is defined by connected free space, not by having
         *              a particular arrangement or number of walls. The
         *              associated walls describe the room boundary but do not
         *              define whether the free-space region is a room.
         */
        const bool validFreeSpaceCluster =
            cluster.size() >=
            static_cast<std::size_t>(sysParams->room_seg.min_cluster_vertices);

        /*!
         * Require at least one associated wall before inserting the free-space
         * cluster into the structural hierarchy.
         */
        const bool hasBoundaryEvidence = !roomWalls.empty();

        /* Confirm the cluster-backed structual element as a room */
        if (validFreeSpaceCluster && hasBoundaryEvidence &&
            room->getRoomVariant() == ORB_SLAM3::Room::roomVariant::UNDEFINED)
        {
            /* Set the room variant to room */
            room->setRoomVariant(ORB_SLAM3::Room::roomVariant::ROOM);

            /* Set the name of the room */
            room->setName("Room#" + std::to_string(room->getId()));

            std::cout << "[SemMgr] Structural Elemetn #" << room->getId()
                      << " classified as a Room from free-space cluster "
                      << clusterId << "." << std::endl;
        }
    }
}

void SemanticsManager::detectRoom_GNN(void)
{
    // [TODO] Needs to be implemented
}

void SemanticsManager::getUpdatedFloors(void)
{
    /* The current implementation supports one floor */
    if (mpAtlas->GetAllFloors().empty())
    {
        GeoSemHelpers::createMapFloor(mpAtlas);
    }

    /* Extract all the rooms from the map */
    std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    /* Remove all invalid rooms */
    allRooms.erase(std::remove_if(allRooms.begin(),
                                  allRooms.end(),
                                  [](ORB_SLAM3::Room *room)
                                  { return room == nullptr || room->isBad(); }),
                   allRooms.end());

    /* If no rooms are valid, return */
    if (allRooms.empty())
    {
        return;
    }

    /* Create list of centoirds for each room */
    std::vector<Eigen::Vector3d> roomCentroids;
    roomCentroids.reserve(allRooms.size());

    /* Extract centroids from each room */
    for (ORB_SLAM3::Room *room : allRooms)
    {
        roomCentroids.push_back(room->getCentroid());
    }

    /* Find the floor centroid from the room centroids */
    const Eigen::Vector3d floorCentroid =
        Utils::computeCentroidFromPoints(roomCentroids);

    /* Extract all floors from map */
    const std::vector<ORB_SLAM3::Floor *> floors = mpAtlas->GetAllFloors();

    /* Iterate through all floors and set them to all rooms */
    for (ORB_SLAM3::Floor *floor : floors)
    {
        /* Skip invalid floors */
        if (floor == nullptr)
        {
            continue;
        }

        floor->setRooms(allRooms);
        floor->setCentroid(floorCentroid);
    }
}

ORB_SLAM3::Room *SemanticsManager::associateRooms(
    const Eigen::Vector3d                  clusterCentroid,
    const std::vector<ORB_SLAM3::Plane *> &wallList)
{
    /* Extract parameter on centre distance threshold of room */
    const double centerDistanceThreshold =
        sysParams->room_seg.center_distance_thresh;

    /* Find shared wall distance threshold, based on larger parameter */
    const double sharedWallDistanceThreshold =
        std::max(centerDistanceThreshold,
                 static_cast<double>(
                     sysParams->room_seg
                         .cluster_centroid_wall_centroid_distance_thresh));

    constexpr double sideEpsilon = 0.20;

    /* Init list variables of nearest room and best shared room */
    ORB_SLAM3::Room *bestSharedRoom = nullptr;
    ORB_SLAM3::Room *nearestRoom    = nullptr;

    /* Init a counter to track the number of best same side matches */
    std::size_t bestSameSideMatches = 0;

    /* Init variables to find the best shared distance and nearest distance */
    double bestSharedDistance = std::numeric_limits<double>::max();
    double nearestDistance    = std::numeric_limits<double>::max();

    /* Get a list of all rooms within map */
    const std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    /* Iterate through rooms */
    for (ORB_SLAM3::Room *room : allRooms)
    {
        /* Skip room if invalid */
        if (room == nullptr || room->isBad())
        {
            continue;
        }

        /* Extract room centroid */
        const Eigen::Vector3d roomCentroid = room->getCentroid();

        /* Find the distance from the cluster center to the room center */
        const double centroidDistance = (clusterCentroid - roomCentroid).norm();

        /* Extract the walls from the room */
        const std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

        /* Init a list to track the room wall ids */
        std::unordered_set<int> roomWallIds;
        roomWallIds.reserve(roomWalls.size());

        /* Iterate through room walls to extract ids of room */
        for (ORB_SLAM3::Plane *roomWall : roomWalls)
        {
            /* Skip invalid walls */
            if (roomWall != nullptr && !roomWall->isBad())
            {
                roomWallIds.insert(roomWall->getId());
            }
        }

        /* Init counters for walls on same and oposite sides of room */
        std::size_t sameSideMatches     = 0;
        std::size_t oppositeSideMatches = 0;

        /* Init a flag to see if the rooms share any walls */
        bool sharesAnyWall = false;

        /* Iterate through walls in room and see if they share walls */
        for (ORB_SLAM3::Plane *candidateWall : wallList)
        {
            /* Skip invalid walls */
            if (candidateWall == nullptr || candidateWall->isBad())
            {
                continue;
            }

            /* If wall is not within room, skip */
            if (roomWallIds.count(candidateWall->getId()) == 0)
            {
                continue;
            }

            /* If wall is not skipped, then shares a wall */
            sharesAnyWall = true;

            /* Extract plane equation */
            Eigen::Vector4d equation =
                candidateWall->getGlobalEquation().coeffs();

            /* Find plane normal magnitude */
            const double normalNorm = equation.head<3>().norm();

            /* If magnitude is invalud, skip wall */
            if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
            {
                continue;
            }

            /* Find unit vector of plane norm */
            equation /= normalNorm;

            /* Caldaulte the side of the cluster */
            const double clusterSide =
                equation.head<3>().dot(clusterCentroid) + equation(3);

            /* Caldaulte the side of the room */
            const double roomSide =
                equation.head<3>().dot(roomCentroid) + equation(3);

            /*!
             * A provisional room may initially have its centroid
             * directly on its first wall. Treat this as compatible.
             */
            const bool centroidNearWall =
                std::abs(clusterSide) <= sideEpsilon ||
                std::abs(roomSide) <= sideEpsilon;

            const bool sameSide = clusterSide * roomSide > 0.0;

            if (centroidNearWall || sameSide)
            {
                sameSideMatches++;
            }
            else
            {
                oppositeSideMatches++;
            }
        }

        /*!
         * Shared-wall matching is preferred, but only when the cluster and room
         * are on the same side.
         */
        if (sameSideMatches > 0 &&
            centroidDistance <= sharedWallDistanceThreshold)
        {
            if (sameSideMatches > bestSameSideMatches ||
                (sameSideMatches == bestSameSideMatches &&
                 centroidDistance < bestSharedDistance))
            {
                bestSameSideMatches = sameSideMatches;

                bestSharedDistance = centroidDistance;

                bestSharedRoom = room;
            }
        }

        /*!
         * A shared wall on the opposite side is evidence of an adjacent room.
         * Do not use centroid fallback in that case.
         */
        if (sharesAnyWall && sameSideMatches == 0 && oppositeSideMatches > 0)
        {
            continue;
        }

        /*!
         * Centroid fallback is only used for provisional structural elements.
         *
         * @note        A confirmed room must not absorb a different free-space
         *              cluster solely because the cluster centroid is nearby. A
         *              confirmed room should be updated using shared wall
         *              evidence.
         */
        if (!sharesAnyWall &&
            room->getRoomVariant() == ORB_SLAM3::Room::roomVariant::UNDEFINED &&
            centroidDistance <= centerDistanceThreshold &&
            centroidDistance < nearestDistance)
        {
            nearestDistance = centroidDistance;

            nearestRoom = room;
        }
    }

    ORB_SLAM3::Room *result =
        bestSharedRoom != nullptr ? bestSharedRoom : nearestRoom;

    if (result != nullptr)
    {
        std::cout << "[SemMgr] Associated cluster with SE#" << result->getId()
                  << ": same-side shared walls=" << bestSameSideMatches
                  << ", centroid distance="
                  << (clusterCentroid - result->getCentroid()).norm() << " m."
                  << std::endl;
    }

    return result;
}

void SemanticsManager::associateAllWallsToRooms(void)
{
    /*!
     * A wall must be observed from several keyframes before it is inserted into
     * the higher-level semantic hierarchy.
     */
    constexpr std::size_t minimumWallObservationCount = 3;

    /* Extract all planes from the current map */
    const std::vector<ORB_SLAM3::Plane *> allPlanes = mpAtlas->GetAllPlanes();

    /* Extract all current rooms and provisional structural elements */
    std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    /* Helper which confirms that a room already contains a wall */
    const auto roomContainsWall = [](ORB_SLAM3::Room  *room,
                                     ORB_SLAM3::Plane *wall) -> bool
    {
        /* Return false if either object is invalid */
        if (room == nullptr || wall == nullptr)
        {
            return false;
        }

        /* Extract the walls currently assigned to the room */
        const std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

        /* Check whether the requested wall is already present */
        return std::any_of(roomWalls.begin(),
                           roomWalls.end(),
                           [wall](ORB_SLAM3::Plane *existingWall) {
                               return existingWall != nullptr &&
                                      existingWall->getId() == wall->getId();
                           });
    };

    /* Iterate through every mapped plane */
    for (ORB_SLAM3::Plane *wall : allPlanes)
    {
        /* Skip invalid planes */
        if (wall == nullptr || wall->isBad())
        {
            continue;
        }

        /* Only confirmed wall planes enter the room hierarchy */
        if (wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
        {
            continue;
        }

        /*!
         * Do not create a provisional structural element form a short-lived
         * plane hypothesis.
         */
        if (wall->getObservationCount() < minimumWallObservationCount)
        {
            continue;
        }

        /* Init flag which confirms whether the wall already has a parent */
        bool wallHasRoom = false;

        /* Search every valid room for the wall */
        for (ORB_SLAM3::Room *room : allRooms)
        {
            /* Skip invalid rooms */
            if (room == nullptr || room->isBad())
            {
                continue;
            }

            /* If the room contains the wall, the hierarchy is complete */
            if (roomContainsWall(room, wall))
            {
                wallHasRoom = true;
                break;
            }
        }

        /* Skip walls which already belong to a room */
        if (wallHasRoom)
        {
            continue;
        }

        /* Extract the centroid of the orphan wall */
        const Eigen::Vector3d wallCentroid = wall->getCentroid().cast<double>();

        /*!
         * The free-space detector did not assign this wall to a room.
         * Create a provisional structural element containing this wall.
         *
         * @note        Do not attach an orphan wall to the nearest room using
         *              centroid distance alone. Adjacent rooms may be close
         *              while still being separated by the same wall.
         */
        ORB_SLAM3::Room *provisionalRoom =
            GeoSemHelpers::createBlankRoomCandidate(mpAtlas, wallCentroid);

        /* Confirm the provisional room was created */
        if (provisionalRoom == nullptr)
        {
            std::cerr << "[SemMgr] Failed to create provisional SE for wall "
                      << wall->getId() << "." << std::endl;

            continue;
        }

        /* Add the orphan wall to the provisional structural element */
        provisionalRoom->setWalls(wall);

        /* Add the provisional room to the current map */
        mpAtlas->AddCandidateMapRoom(provisionalRoom);

        /* Update the local room list for the remaining walls */
        allRooms.push_back(provisionalRoom);

        /*!
         * Register the wall in the Atlas-wide room-wall collection.
         *
         * @note        This registry is not exclusive ownership. The same
         *              wall may later belong to two rooms on opposite sides.
         */
        if (mpAtlas->GetRoomWallPlaneById(wall->getId()) == nullptr)
        {
            mpAtlas->AddRoomWallPlane(wall);
        }

        std::cout << "[SemMgr] Wall " << wall->getId()
                  << " had no room. Created provisional SE#"
                  << provisionalRoom->getId() << "." << std::endl;
    }
}

void SemanticsManager::consolidateProvisionalRooms(
    ORB_SLAM3::Room *selectedRoom)
{
    /* Confirm the selected room is valid */
    if (selectedRoom == nullptr || selectedRoom->isBad())
    {
        return;
    }

    /* Extract all walls assigned to the cluster-backed room */
    const std::vector<ORB_SLAM3::Plane *> selectedWalls =
        selectedRoom->getWalls();

    /* Create a set containing the selected wall IDs */
    std::unordered_set<int> selectedWallIds;
    selectedWallIds.reserve(selectedWalls.size());

    /* Insert every valid selected wall ID into the set */
    for (ORB_SLAM3::Plane *wall : selectedWalls)
    {
        if (wall != nullptr && !wall->isBad())
        {
            selectedWallIds.insert(wall->getId());
        }
    }

    /* A room without walls cannot absorb another structural element */
    if (selectedWallIds.empty())
    {
        return;
    }

    /* Extract all rooms and provisional structural elements */
    const std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    /* Iterate through every possible redundant structural element */
    for (ORB_SLAM3::Room *candidateRoom : allRooms)
    {
        /* Skip invalid rooms and the selected room itself */
        if (candidateRoom == nullptr || candidateRoom == selectedRoom ||
            candidateRoom->isBad())
        {
            continue;
        }

        /*!
         * Only automatically absorb undefined structural elements.
         * Classified rooms and corridors must never be merged automatically.
         */
        if (candidateRoom->getRoomVariant() !=
            ORB_SLAM3::Room::roomVariant::UNDEFINED)
        {
            continue;
        }

        /* Extract the candidate room walls */
        const std::vector<ORB_SLAM3::Plane *> candidateWalls =
            candidateRoom->getWalls();

        /*!
         * Orphan-wall fallback creates one provisional SE per wall. Restrict
         * automatic consolidation to those single-wall provisional elements.
         * This prevents an emerging room on the opposite side of a shared wall
         * from being removed before it has enough evidence for classification.
         */
        std::vector<ORB_SLAM3::Plane *> validCandidateWalls;
        validCandidateWalls.reserve(candidateWalls.size());

        for (ORB_SLAM3::Plane *candidateWall : candidateWalls)
        {
            if (candidateWall != nullptr && !candidateWall->isBad())
            {
                validCandidateWalls.push_back(candidateWall);
            }
        }

        /* Only single-wall provisional elements are safe to absorb */
        if (validCandidateWalls.size() != 1)
        {
            continue;
        }

        /* Extract the single wall represented by the provisional element */
        ORB_SLAM3::Plane *candidateWall = validCandidateWalls.front();

        /* The selected room must already contain the candidate wall */
        if (selectedWallIds.count(candidateWall->getId()) == 0)
        {
            continue;
        }

        /*!
         * Confirm the candidate centroid is still close to its wall plane.
         * This identifies a wall-centred orphan SE rather than a free-space
         * cluster which may represent a genuine room on the opposite side.
         */
        Eigen::Vector4d wallEquation =
            candidateWall->getGlobalEquation().coeffs();

        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }

        wallEquation /= normalNorm;

        const double candidatePlaneDistance =
            std::abs(wallEquation.head<3>().dot(candidateRoom->getCentroid()) +
                     wallEquation(3));

        constexpr double provisionalWallDistanceThreshold = 0.25;

        if (candidatePlaneDistance > provisionalWallDistanceThreshold)
        {
            continue;
        }

        /* Preserve passage relationships before invalidating the candidate */
        const std::vector<ORB_SLAM3::Passage *> candidatePassages =
            candidateRoom->getPassages();

        for (ORB_SLAM3::Passage *passage : candidatePassages)
        {
            /* Skip invalid passages */
            if (passage == nullptr)
            {
                continue;
            }

            /* Check whether the selected room already contains the passage */
            const std::vector<ORB_SLAM3::Passage *> selectedPassages =
                selectedRoom->getPassages();

            const bool alreadyPresent = std::any_of(
                selectedPassages.begin(),
                selectedPassages.end(),
                [passage](ORB_SLAM3::Passage *existingPassage)
                {
                    return existingPassage != nullptr &&
                           existingPassage->getId() == passage->getId();
                });

            /* Copy the passage relationship if required */
            if (!alreadyPresent)
            {
                selectedRoom->setDoorways(passage);
            }
        }

        /* Mark the redundant provisional structural element as invalid */
        candidateRoom->setBad();

        std::cout << "[SemMgr] Consolidated provisional SE#"
                  << candidateRoom->getId() << " into SE#"
                  << selectedRoom->getId() << "." << std::endl;
    }
}

void SemanticsManager::associatePassagesToRooms(void)
{
    /* Extract all rooms from the current map */
    const std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    /* Extract all passages from the current map */
    const std::vector<ORB_SLAM3::Passage *> allPassages =
        mpAtlas->GetAllPassages();

    constexpr double sideEpsilon = 0.20;

    /* Iterate through every passage */
    for (ORB_SLAM3::Passage *passage : allPassages)
    {
        /* Skip invalid passages */
        if (passage == nullptr)
        {
            continue;
        }

        /* Extract the wall or walls supporting the passage */
        const std::vector<ORB_SLAM3::Plane *> supportingWalls =
            passage->getAssociateWalls();

        /* A passage without a supporting wall cannot connect rooms */
        if (supportingWalls.empty())
        {
            continue;
        }

        /* Extract and normalize the passage plane equation */
        Eigen::Vector4d passageEquation = passage->getGlobalEquation().coeffs();

        const double normalNorm = passageEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }

        passageEquation /= normalNorm;

        /* Extract the passage centroid in double precision */
        const Eigen::Vector3d passageCentroid =
            passage->getCentroid().cast<double>();

        /* Track the closest room found on each side of the passage */
        ORB_SLAM3::Room *negativeSideRoom = nullptr;
        ORB_SLAM3::Room *positiveSideRoom = nullptr;

        double negativeRoomDistance = std::numeric_limits<double>::max();
        double positiveRoomDistance = std::numeric_limits<double>::max();

        /* Iterate through all valid rooms */
        for (ORB_SLAM3::Room *room : allRooms)
        {
            /* Skip invalid rooms */
            if (room == nullptr || room->isBad())
            {
                continue;
            }

            /* Extract the walls assigned to the room */
            const std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

            /* Confirm the room contains a wall supporting the passage */
            const bool sharesSupportingWall = std::any_of(
                supportingWalls.begin(),
                supportingWalls.end(),
                [&](ORB_SLAM3::Plane *supportingWall)
                {
                    if (supportingWall == nullptr)
                    {
                        return false;
                    }

                    return std::any_of(roomWalls.begin(),
                                       roomWalls.end(),
                                       [&](ORB_SLAM3::Plane *roomWall)
                                       {
                                           return roomWall != nullptr &&
                                                  roomWall->getId() ==
                                                      supportingWall->getId();
                                       });
                });

            /* Skip rooms which do not contain the supporting wall */
            if (!sharesSupportingWall)
            {
                continue;
            }

            /* Extract the room centroid */
            const Eigen::Vector3d roomCentroid = room->getCentroid();

            /* Determine which side of the passage plane contains the room */
            const double side = passageEquation.head<3>().dot(roomCentroid) +
                                passageEquation(3);

            /*!
             * A wall-centred provisional SE does not yet provide enough
             * evidence to form a room-to-passage connection.
             */
            if (std::abs(side) <= sideEpsilon)
            {
                continue;
            }

            /* Find the distance from the room to the passage */
            const double roomDistance = (roomCentroid - passageCentroid).norm();

            /* Keep the closest room on the negative side */
            if (side < 0.0 && roomDistance < negativeRoomDistance)
            {
                negativeRoomDistance = roomDistance;
                negativeSideRoom     = room;
            }

            /* Keep the closest room on the positive side */
            if (side > 0.0 && roomDistance < positiveRoomDistance)
            {
                positiveRoomDistance = roomDistance;
                positiveSideRoom     = room;
            }
        }

        /* Helper which adds a passage to a room without duplicates */
        const auto addPassageToRoom = [passage](ORB_SLAM3::Room *room)
        {
            /* Skip invalid rooms */
            if (room == nullptr)
            {
                return;
            }

            /* Extract the passages already assigned to the room */
            const std::vector<ORB_SLAM3::Passage *> roomPassages =
                room->getPassages();

            /* Check whether the relationship already exists */
            const bool alreadyAssociated = std::any_of(
                roomPassages.begin(),
                roomPassages.end(),
                [passage](ORB_SLAM3::Passage *existingPassage)
                {
                    return existingPassage != nullptr &&
                           existingPassage->getId() == passage->getId();
                });

            /* Add the relationship if required */
            if (!alreadyAssociated)
            {
                room->setDoorways(passage);

                std::cout << "[SemMgr] Associated Passage#" << passage->getId()
                          << " with Room#" << room->getId() << "." << std::endl;
            }
        };

        /* Add at most one room from each side of the passage */
        addPassageToRoom(negativeSideRoom);
        addPassageToRoom(positiveSideRoom);
    }
}

void SemanticsManager::reAssociateRooms(void)
{
    /*!
     * Re-run the targeted provisional-room consolidation pass.
     *
     * @note        This function does not merge confirmed rooms. It only allows
     *              a confirmed room or corridor to absorb redundant,
     *              single-wall provisional structural elements.
     */
    const std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    for (ORB_SLAM3::Room *room : allRooms)
    {
        /* Skip invalid structural elements */
        if (room == nullptr || room->isBad())
        {
            continue;
        }

        /*!
         * Only confirmed rooms and corridors may absorb provisional
         * structural elements.
         */
        const bool isConfirmedRoom =
            room->getRoomVariant() == ORB_SLAM3::Room::roomVariant::ROOM ||
            room->getRoomVariant() == ORB_SLAM3::Room::roomVariant::CORRIDOR;

        if (!isConfirmedRoom)
        {
            continue;
        }

        /* Require more than one valid wall before allowing consolidation */
        std::size_t validWallCount = 0;

        const std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

        for (ORB_SLAM3::Plane *wall : roomWalls)
        {
            if (wall != nullptr && !wall->isBad())
            {
                validWallCount++;
            }
        }

        if (validWallCount < 2)
        {
            continue;
        }

        /*!
         * Consolidate only redundant single-wall provisional structural
         * elements whose wall already belongs to this confirmed room.
         */
        consolidateProvisionalRooms(room);
    }
}

} // namespace ORB_SLAM3