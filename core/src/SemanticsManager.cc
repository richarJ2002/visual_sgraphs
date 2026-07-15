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
    mpAtlas = pAtlas;

    // Get the system parameters
    sysParams = SystemParams::GetParams();
}

void SemanticsManager::Run()
{
    while (true)
    {
        /* Get the start time */
        auto start = std::chrono::high_resolution_clock::now();

        /* Init the main ground plane variable */
        Plane *mainGroundPlane = mpAtlas->GetBiggestGroundPlane();

        /*!
         * Update the ground plane.
         *
         * @note:   It might have been updated when semantic segmentation did
         *          not detect any plane.
         */
        if (mainGroundPlane != nullptr)
        {
            /*!
             * Re-compute the transofmraiton from ground to horizontal.
             *
             * @note:   Re-compute the transformation from ground to horizontal
             *          - maybe global eq. changed.
             */
            mPlanePoseMat = computePlaneToHorizontal(mainGroundPlane);

            /* Filter all planes with semantics */
            filterGroundPlanes(mainGroundPlane);
            filterWallPlanes();
        }

        /*!
         * Re-associate semantic planes if they get close to each other after
         * optimization
         */
        if (sysParams->sem_seg.reassociate.enabled)
        {
            Utils::reAssociateSemanticPlanes(mpAtlas);
        }

        /* Check for possible room candidates */
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

        /* Make sure every wall is assocaited with a room */
        assocaiteAlLWallsToRooms();

        /* Re-associate rooms based on walls and clusters */
        // reAssociateRooms();

        /* Detect passages between rooms */
        if (sysParams->sem_seg.enable_passage_detection)
        {
            detectDoorsAndDoorways(mpAtlas);
            updatePassages(mpAtlas);
            assocaitePassagesToRooms();
        }

        /* Check detected floors */
        getUpdatedFloors();

        /* wait until its intervalTime to run the next iteration */
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if (elapsed.count() < runInterval)
        {
            std::this_thread::sleep_for(std::chrono::seconds(runInterval) -
                                        elapsed);
        }
    }
}

std::vector<std::vector<Eigen::Vector3d>>
    SemanticsManager::getLatestSkeletonCluster()
{
    unique_lock<std::mutex> lock(mMutexNewRooms);
    // Get the latest skeleton cluster from Atlas
    return mpAtlas->GetSkeletoClusterPoints();
}

std::vector<ORB_SLAM3::Room *> SemanticsManager::getLatestGNNRoomCandidates()
{
    // [TODO]
}

void SemanticsManager::filterWallPlanes()
{
    for (const auto &plane : mpAtlas->GetAllPlanes())
    {
        if (plane->getExpectedPlaneType() ==
            ORB_SLAM3::Plane::planeVariant::WALL)
        {
            // wall validation based on the mPlanePoseMat
            // only works if the ground plane is set, needs the correction
            // matrix: mPlanePoseMat
            Eigen::Vector3f transformedPlaneCoefficients =
                transformPlaneEqToGroundReference(
                    plane->getGlobalEquation().coeffs());

            // if the transformed plane is vertical based on absolute value,
            // then assign semantic, otherwise ignore threshold should be
            // leniently set (ideally with correct ground plane reference, this
            // value should be close to 0.00)
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

    // get the median height of the plane to compute the threshold
    float threshY = computeGroundPlaneHeight(groundPlane) -
                    sysParams->sem_seg.max_step_elevation;

    // go through all ground planes to check validity
    int groundPlaneId = groundPlane->getId();
    for (const auto &plane : mpAtlas->GetAllPlanes())
    {
        if (plane->getExpectedPlaneType() !=
                ORB_SLAM3::Plane::planeVariant::GROUND ||
            plane->getId() == groundPlaneId)
        {
            continue;
        }

        // if the plane is above the threshold (inverted y), then reset the
        // plane semantics
        if (computeGroundPlaneHeight(plane) < threshY)
        {
            plane->resetPlaneSemantics();
            continue;
        }

        // filter here based on orientation of the plane (needs to be
        // horizontal)
        Eigen::Vector3f transformedPlaneCoefficients =
            transformPlaneEqToGroundReference(
                plane->getGlobalEquation().coeffs());

        // if the transformed plane is horizontal based on absolute value, then
        // assign semantic, otherwise ignore threshold should be leniently set
        // (ideally with correct ground plane reference, this value should be
        // close to 0.00)
        if (abs(transformedPlaneCoefficients(0)) >
            sysParams->sem_seg.max_tilt_ground)
            plane->resetPlaneSemantics();
    }
}

void SemanticsManager::detectDoorsAndDoorways(ORB_SLAM3::Atlas *pAtlas)
{
    // Variables
    int windowSize = sysParams->sem_seg.passage_kf_window;

    // Get all planes and filter only DOOR and WALL variants
    std::vector<ORB_SLAM3::Plane *> allPlanes = pAtlas->GetAllPlanes();
    std::vector<ORB_SLAM3::Plane *> wallPlanes;
    std::vector<ORB_SLAM3::Plane *> doorPlanes;
    for (const auto &plane : allPlanes)
    {
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL)
        {
            wallPlanes.push_back(plane);
        }
        else if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::DOOR)
        {
            doorPlanes.push_back(plane);
        }
    }

    std::cout << "[PassageDebug] walls=" << wallPlanes.size()
              << ", doors=" << doorPlanes.size()
              << ", existing passages=" << pAtlas->GetAllPassages().size()
              << std::endl;

    // Procedure#1 - Detecting closed doors (blocked passages)
    // Detect doors with the same normal as walls and close to walls
    for (const auto &door : doorPlanes)
    {
        for (const auto &wall : wallPlanes)
        {
            // Skip if the door and wall are not parallel
            if (!Utils::arePlanesParallel(door, wall))
                continue;

            // Skip if the door and wall are facing each other
            if (Utils::arePlanesFacingEachOther(door, wall))
                continue;

            // Skip if the door and wall are not close to each other
            if (Utils::arePlanesApartEnough(
                    door,
                    wall,
                    sysParams->sem_seg.max_wall_door_distance))
                continue;

            // Otherwise, it is a valid closed door to be connected to the wall
            GeoSemHelpers::createMapPassage(mpAtlas, door, wall, false);
        }
    }

    // Procedure #2: detect open passages from trajectory crossings
    std::vector<ORB_SLAM3::KeyFrame *> allKFs = pAtlas->GetAllKeyFrames();

    allKFs.erase(std::remove_if(allKFs.begin(),
                                allKFs.end(),
                                [](ORB_SLAM3::KeyFrame *kf)
                                { return kf == nullptr || kf->isBad(); }),
                 allKFs.end());

    if (allKFs.size() < 2)
    {
        std::cout << "[PassageDebug] Not enough valid keyframes: "
                  << allKFs.size() << std::endl;

        return;
    }

    std::sort(allKFs.begin(),
              allKFs.end(),
              [](const ORB_SLAM3::KeyFrame *a, const ORB_SLAM3::KeyFrame *b)
              { return a->mnId < b->mnId; });

    constexpr double crossingEpsilon = 0.05;
    constexpr double minimumBaseline = 0.10;

    for (ORB_SLAM3::Plane *wall : wallPlanes)
    {
        if (wall == nullptr || wall->isBad())
        {
            continue;
        }

        Eigen::Vector4d wallEq = wall->getGlobalEquation().coeffs();

        const double normalNorm = wallEq.head<3>().norm();

        if (normalNorm < 1e-8)
        {
            std::cout << "[PassageDebug] Wall " << wall->getId()
                      << " has an invalid equation." << std::endl;

            continue;
        }

        wallEq /= normalNorm;

        const Eigen::Vector3d wallNormal = wallEq.head<3>();

        bool passageDetected = false;

        double minimumAbsoluteDistance = std::numeric_limits<double>::max();

        for (std::size_t i = 0; i + 1 < allKFs.size(); ++i)
        {
            ORB_SLAM3::KeyFrame *kfA = allKFs[i];
            ORB_SLAM3::KeyFrame *kfB = allKFs[i + 1];

            const Eigen::Vector3d posA =
                kfA->GetPoseInverse().translation().cast<double>();

            const Eigen::Vector3d posB =
                kfB->GetPoseInverse().translation().cast<double>();

            const double baseline = (posB - posA).norm();

            if (baseline < minimumBaseline)
            {
                continue;
            }

            const double dA = wallNormal.dot(posA) + wallEq(3);

            const double dB = wallNormal.dot(posB) + wallEq(3);

            minimumAbsoluteDistance =
                std::min(minimumAbsoluteDistance,
                         std::min(std::abs(dA), std::abs(dB)));

            /*!
             * A real crossing requires the consecutive keyframes to be on
             * opposite sides of the wall plane.
             */
            const bool signChange =
                (dA < 0.0 && dB > 0.0) || (dA > 0.0 && dB < 0.0);

            /*!
             * Reject tiny sign changes caused by pose or plane-estimation
             * noise. This is the distance travelled in the wall-normal
             * direction.
             */
            const double normalTravel = std::abs(dB - dA);

            constexpr double minimumNormalTravel = 0.10;

            if (!signChange || normalTravel < minimumNormalTravel)
            {
                continue;
            }

            if (std::min(std::abs(dA), std::abs(dB)) >
                sysParams->sem_seg.max_kf_passage_distance)
            {
                continue;
            }

            const double denominator = dA - dB;

            Eigen::Vector3d crossingPoint;

            if (std::abs(denominator) > 1e-8)
            {
                double t = dA / denominator;
                t        = std::clamp(t, 0.0, 1.0);

                crossingPoint = posA + t * (posB - posA);
            }
            else
            {
                crossingPoint = 0.5 * (posA + posB);
            }

            std::cout << "[PassageDebug] Trajectory crossed wall "
                      << wall->getId() << " between KF " << kfA->mnId
                      << " and KF " << kfB->mnId << ". dA=" << dA
                      << ", dB=" << dB
                      << ", crossing=" << crossingPoint.transpose()
                      << std::endl;

            GeoSemHelpers::createMapPassage(mpAtlas,
                                            nullptr,
                                            wall,
                                            true,
                                            crossingPoint.cast<float>());

            passageDetected = true;
            break;
        }

        if (!passageDetected)
        {
            std::cout << "[PassageDebug] No trajectory crossing for wall "
                      << wall->getId() << ". Closest keyframe distance="
                      << minimumAbsoluteDistance << " m." << std::endl;
        }
    }

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
    // extract the rotation matrix from the transformation matrix
    Eigen::Matrix3f rotationMatrix = mPlanePoseMat.block<3, 3>(0, 0);

    // Compute the inverse transpose of the rotation matrix
    Eigen::Matrix3f inverseTransposeRotationMatrix =
        rotationMatrix.inverse().transpose();

    // Transform the coefficients of the plane equation
    Eigen::Vector3f transformedPlaneCoefficients =
        inverseTransposeRotationMatrix * planeEq.head<3>().cast<float>();
    transformedPlaneCoefficients.normalize();

    return transformedPlaneCoefficients;
}

float SemanticsManager::computeGroundPlaneHeight(Plane *groundPlane)
{
    // transform the planeCloud according to the planePose
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud =
        groundPlane->getMapClouds();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*planeCloud, *transformedCloud, mPlanePoseMat);

    // get the median height of the plane
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

void SemanticsManager::detectRoom_FreeSpaceCluster()
{
    /*!
     * Original value was effectively 0.30.
     *
     * If the logs repeatedly show "0 candidate walls", try reducing this
     * to 0.10 because skeleton points normally occupy the room interior
     * rather than lying directly beside the walls.
     */
    constexpr double minClosePointRatio = 0.30;

    /* Set this to false after room detection is working correctly */
    constexpr bool enableDebugOutput = true;

    /* Obtain the latest free-space/skeleton clusters */
    const std::vector<std::vector<Eigen::Vector3d>> clusters =
        getLatestSkeletonCluster();

    if (clusters.empty())
    {
        if (enableDebugOutput)
        {
            std::cout << "[SemMgr] No free-space clusters available."
                      << std::endl;
        }

        return;
    }

    /* Initialize list of wall planes*/
    std::vector<ORB_SLAM3::Plane *> allWalls;

    /* Collect valid wall planes */
    const std::vector<ORB_SLAM3::Plane *> allPlanes = mpAtlas->GetAllPlanes();

    allWalls.reserve(allPlanes.size());

    for (ORB_SLAM3::Plane *plane : allPlanes)
    {
        if (plane == nullptr || plane->isBad())
        {
            continue;
        }

        /*!
         * getPlaneType() may still be UNDEFINED while semantic votes are
         * accumulating. Accept getExpectedPlaneType() as well so that likely
         * wall planes can participate in room detection.
         */
        const bool isWall =
            plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::WALL;

        if (isWall)
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

    if (allWalls.empty())
    {
        return;
    }

    /* Process every free-space cluster */
    for (std::size_t clusterId = 0; clusterId < clusters.size(); ++clusterId)
    {
        const std::vector<Eigen::Vector3d> &cluster = clusters[clusterId];

        if (cluster.empty())
        {
            if (enableDebugOutput)
            {
                std::cout << "[SemMgr] Cluster " << clusterId << " is empty."
                          << std::endl;
            }

            continue;
        }

        const Eigen::Vector3d clusterCentroid =
            Utils::computeCentroidFromPoints(cluster);

        std::vector<ORB_SLAM3::Plane *> closestWalls;
        closestWalls.reserve(allWalls.size());

        /* Find walls spatially related to this free-space cluster */
        for (ORB_SLAM3::Plane *wall : allWalls)
        {
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr wallCloud =
                wall->getMapClouds();

            /*
             * A finite wall test requires the actual wall point cloud.
             */
            if (wallCloud == nullptr || wallCloud->empty())
            {
                if (enableDebugOutput)
                {
                    std::cout << "[SemMgr] Skipping wall " << wall->getId()
                              << ": empty wall cloud." << std::endl;
                }

                continue;
            }

            const Eigen::Vector3d wallCentroid =
                wall->getCentroid().cast<double>();

            const double centroidDistance =
                (wallCentroid - clusterCentroid).norm();

            /*
             * First coarse check.
             */
            if (centroidDistance >=
                sysParams->room_seg
                    .cluster_centroid_wall_centroid_distance_thresh)
            {
                continue;
            }

            /*
             * Normalize the wall equation:
             *
             *     n.x + d = 0
             */
            Eigen::Vector4d wallEquation = wall->getGlobalEquation().coeffs();

            const double normalNorm = wallEquation.head<3>().norm();

            if (normalNorm < 1e-8)
            {
                continue;
            }

            const Eigen::Vector3d normal = wallEquation.head<3>() / normalNorm;

            const double normalizedD = wallEquation(3) / normalNorm;

            /*
             * Construct two perpendicular axes lying inside the wall plane.
             */
            const Eigen::Vector3d axisU = normal.unitOrthogonal().normalized();

            const Eigen::Vector3d axisV = normal.cross(axisU).normalized();

            /*
             * Calculate the finite 2-D bounds of the observed wall cloud.
             */
            double minU = std::numeric_limits<double>::max();
            double maxU = std::numeric_limits<double>::lowest();
            double minV = std::numeric_limits<double>::max();
            double maxV = std::numeric_limits<double>::lowest();

            std::size_t validWallPoints = 0;

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

                const double coordinateU = relativePoint.dot(axisU);

                const double coordinateV = relativePoint.dot(axisV);

                minU = std::min(minU, coordinateU);
                maxU = std::max(maxU, coordinateU);
                minV = std::min(minV, coordinateV);
                maxV = std::max(maxV, coordinateV);

                ++validWallPoints;
            }

            if (validWallPoints == 0)
            {
                continue;
            }

            /*
             * Permit some tolerance because wall point clouds are incomplete
             * and noisy.
             */
            constexpr double wallBoundsMargin = 0.75;

            std::size_t supportedPointCount = 0;

            double minimumPlaneDistance = std::numeric_limits<double>::max();

            for (const Eigen::Vector3d &clusterPoint : cluster)
            {
                /*
                 * Signed perpendicular distance to the infinite plane.
                 */
                const double signedPlaneDistance =
                    normal.dot(clusterPoint) + normalizedD;

                const double planeDistance = std::abs(signedPlaneDistance);

                minimumPlaneDistance =
                    std::min(minimumPlaneDistance, planeDistance);

                /*
                 * The point must first be reasonably close to the plane.
                 */
                if (planeDistance >=
                    sysParams->room_seg.cluster_point_wall_distance_thresh)
                {
                    continue;
                }

                /*
                 * Project the cluster point onto the wall plane.
                 */
                const Eigen::Vector3d projectedPoint =
                    clusterPoint - signedPlaneDistance * normal;

                const Eigen::Vector3d projectedRelative =
                    projectedPoint - wallCentroid;

                const double projectedU = projectedRelative.dot(axisU);

                const double projectedV = projectedRelative.dot(axisV);

                /*
                 * Crucial finite-wall check:
                 * the projection must lie inside, or near, the observed wall
                 * patch.
                 */
                const bool insideFiniteWall =
                    projectedU >= minU - wallBoundsMargin &&
                    projectedU <= maxU + wallBoundsMargin &&
                    projectedV >= minV - wallBoundsMargin &&
                    projectedV <= maxV + wallBoundsMargin;

                if (insideFiniteWall)
                {
                    ++supportedPointCount;
                }
            }

            const double supportRatio =
                static_cast<double>(supportedPointCount) /
                static_cast<double>(cluster.size());

            if (enableDebugOutput)
            {
                std::cout << "[SemMgr] Cluster " << clusterId << ", wall "
                          << wall->getId()
                          << ": centroid distance=" << centroidDistance
                          << " m, minimum plane distance="
                          << minimumPlaneDistance
                          << " m, finite support=" << supportedPointCount << "/"
                          << cluster.size() << " (" << supportRatio * 100.0
                          << "%)." << std::endl;
            }

            if (supportRatio >= minClosePointRatio)
            {
                closestWalls.push_back(wall);
            }
        }

        /* Remove duplicate plane pointers */
        std::sort(closestWalls.begin(), closestWalls.end());

        closestWalls.erase(
            std::unique(closestWalls.begin(), closestWalls.end()),
            closestWalls.end());

        if (enableDebugOutput)
        {
            std::cout << "[SemMgr] Cluster " << clusterId << " has "
                      << closestWalls.size() << " candidate walls."
                      << std::endl;
        }

        if (closestWalls.empty())
        {
            continue;
        }

        /*!
         * First try to associate with an existing room.
         *
         * This must occur before removing already-assigned walls, because
         * wall overlap is how associateRooms() recognizes an existing room.
         */
        ORB_SLAM3::Room *room = associateRooms(clusterCentroid, closestWalls);

        /*!
         * If no room matches, only unassigned walls may be used to create
         * a new room candidate.
         */
        if (room == nullptr)
        {
            /*
             * No existing room matched this free-space cluster.
             * Print the current Atlas state before creating a room.
             */
            const std::vector<ORB_SLAM3::Room *> allRooms =
                mpAtlas->GetAllRooms();

            std::cout << "[SemMgr] No valid room matched cluster " << clusterId
                      << "." << std::endl;

            std::cout << "[SemMgr] GetAllRooms returned " << allRooms.size()
                      << " rooms." << std::endl;

            for (ORB_SLAM3::Room *mapRoom : allRooms)
            {
                if (mapRoom == nullptr)
                {
                    std::cout << "  null room" << std::endl;
                    continue;
                }

                std::cout << "  Room SE#" << mapRoom->getId()
                          << ": bad=" << mapRoom->isBad() << ", variant="
                          << static_cast<int>(mapRoom->getRoomVariant())
                          << ", walls=" << mapRoom->getWalls().size()
                          << ", centroid=" << mapRoom->getCentroid().transpose()
                          << std::endl;
            }

            /*
             * Find candidate walls that are not already registered
             * as belonging to a room.
             */
            std::vector<ORB_SLAM3::Plane *> unassignedWalls;
            unassignedWalls.reserve(closestWalls.size());

            for (ORB_SLAM3::Plane *wall : closestWalls)
            {
                if (wall == nullptr)
                {
                    continue;
                }

                const auto *registeredWall =
                    mpAtlas->GetRoomWallPlaneById(wall->getId());

                const bool isRegistered = registeredWall != nullptr;

                std::cout << "  Wall " << wall->getId()
                          << ": registered=" << isRegistered << std::endl;

                if (!isRegistered)
                {
                    unassignedWalls.push_back(wall);
                }
            }

            /*
             * Check the vector before moving from it.
             */
            if (unassignedWalls.empty())
            {
                std::cout << "[SemMgr] Cluster " << clusterId
                          << " has no unassigned walls for a new room."
                          << std::endl;

                std::cout << "[SemMgr] Candidate walls are already registered, "
                          << "but no valid room containing them was found."
                          << std::endl;

                continue;
            }

            std::cout << "[SemMgr] Cluster " << clusterId << " has "
                      << unassignedWalls.size() << " unassigned walls."
                      << std::endl;

            /*
             * Only move the walls after checking whether the vector
             * is empty.
             */
            closestWalls = std::move(unassignedWalls);

            /*
             * Create exactly one room candidate.
             */
            room = GeoSemHelpers::createBlankRoomCandidate(mpAtlas,
                                                           clusterCentroid);

            if (room == nullptr)
            {
                std::cerr << "[SemMgr] Failed to create room candidate for "
                          << "cluster " << clusterId << "." << std::endl;

                continue;
            }

            /*
             * Register the candidate with the Atlas.
             */
            mpAtlas->AddCandidateMapRoom(room);

            std::cout << "[SemMgr] New room candidate created: SE#"
                      << room->getId() << " for cluster " << clusterId << "."
                      << std::endl;
        }
        else if (enableDebugOutput)
        {
            std::cout << "[SemMgr] Cluster " << clusterId
                      << " associated with existing room SE#" << room->getId()
                      << "." << std::endl;
        }

        /* Obtain the room's current walls */
        std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

        /* Add every candidate wall that is not already in this room */
        for (ORB_SLAM3::Plane *wall : closestWalls)
        {
            if (wall == nullptr || wall->isBad())
            {
                continue;
            }

            const bool alreadyInCurrentRoom =
                std::any_of(roomWalls.begin(),
                            roomWalls.end(),
                            [&](ORB_SLAM3::Plane *existingWall) {
                                return existingWall != nullptr &&
                                       existingWall->getId() == wall->getId();
                            });

            if (alreadyInCurrentRoom)
            {
                continue;
            }

            /*!
             * Add the wall to this room regardless of whether it is already
             * present in the Atlas-wide room-wall collection.
             */
            room->setWalls(wall);
            roomWalls.push_back(wall);

            /*!
             * Add it to the Atlas-wide collection only if it is not already
             * there.
             */
            if (mpAtlas->GetRoomWallPlaneById(wall->getId()) == nullptr)
            {
                mpAtlas->AddRoomWallPlane(wall);
            }

            if (enableDebugOutput)
            {
                std::cout << "[SemMgr] Added wall " << wall->getId()
                          << " to room SE#" << room->getId() << "."
                          << std::endl;
            }
        }

        /*!
         * Critical fix:
         *
         * Refresh roomWalls after calling setWalls(). The original code used
         * the stale copy obtained before the walls were added.
         */
        roomWalls = room->getWalls();

        /* Remove invalid walls before layout classification */
        roomWalls.erase(std::remove_if(roomWalls.begin(),
                                       roomWalls.end(),
                                       [](ORB_SLAM3::Plane *wall) {
                                           return wall == nullptr ||
                                                  wall->isBad();
                                       }),
                        roomWalls.end());

        if (enableDebugOutput)
        {
            std::cout << "[SemMgr] Room SE#" << room->getId()
                      << " currently has " << roomWalls.size()
                      << " valid walls." << std::endl;
        }

        /*!
         * At least two walls are needed before searching for a pair of
         * facing walls.
         */
        if (roomWalls.size() < 2)
        {
            continue;
        }

        const std::vector<std::pair<ORB_SLAM3::Plane *, ORB_SLAM3::Plane *>>
            facingWalls = Utils::getFacingPlanes(roomWalls);

        if (enableDebugOutput)
        {
            std::cout << "[SemMgr] Room SE#" << room->getId() << " has "
                      << facingWalls.size() << " facing wall pairs."
                      << std::endl;
        }

        /*!
         * Free-space room detection classifies the structural element as a
         * room when at least one pair of facing walls is available.
         */
        if (room->getRoomVariant() == ORB_SLAM3::Room::roomVariant::UNDEFINED &&
            !facingWalls.empty())
        {
            room->setRoomVariant(ORB_SLAM3::Room::roomVariant::ROOM);

            room->setName("Room#" + std::to_string(room->getId()));

            std::cout << "[SemMgr] Structural Element #" << room->getId()
                      << " classified as a Room." << std::endl;
        }
    }
}

void SemanticsManager::detectRoom_GNN()
{
    // [TODO] Needs to be implemented
}

void SemanticsManager::getUpdatedFloors()
{
    // [TODO] The current version supports singe floor only.
    if (mpAtlas->GetAllFloors().size() < 1)
    {
        // Create a new floor object
        GeoSemHelpers::createMapFloor(mpAtlas);
    }
    else
    {
        /* Update the existing floor object to cotain all rooms */
        std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

        /* If the room is bad, remove it from the list */
        allRooms.erase(std::remove_if(allRooms.begin(),
                                      allRooms.end(),
                                      [](ORB_SLAM3::Room *room)
                                      { return room->isBad(); }),
                       allRooms.end());

        /* If there is no rooms, return */
        if (allRooms.empty())
        {
            return;
        }

        /* Initialize vector to contain room centroids */
        std::vector<Eigen::Vector3d> roomCentroids;

        /* Reserve memory for room ventroid vector */
        roomCentroids.reserve(allRooms.size());

        /* Iterate through every room in rooms array */
        for (ORB_SLAM3::Room *room : allRooms)
        {
            /* If room is valid, push back the centroids, otherwise skip */
            if (room != nullptr)
            {
                roomCentroids.push_back(room->getCentroid());
            }
        }

        /* If no valid centroids, then return */
        if (roomCentroids.empty())
        {
            return;
        }

        /* Get all the floors available */
        for (ORB_SLAM3::Floor *floor : mpAtlas->GetAllFloors())
        {
            /* If floor is invalid skip */
            if (floor == nullptr)
            {
                continue;
            }

            /* Set the floor to the valid rooms */
            floor->setRooms(allRooms);

            /* Set the centroid of the floor based on room centroids */
            floor->setCentroid(Utils::computeCentroidFromPoints(roomCentroids));
        }
    }
}

ORB_SLAM3::Room *SemanticsManager::associateRooms(
    const Eigen::Vector3d                  clusterCentroid,
    const std::vector<ORB_SLAM3::Plane *> &wallList)
{
    /* Extract the distance threshold between room centers */
    const double distanceThreshold = sysParams->room_seg.center_distance_thresh;

    /* Init pointers to best overlap room candidate */
    ORB_SLAM3::Room *bestOverlapRoom = nullptr;

    /* Init pointers to nearest room */
    ORB_SLAM3::Room *nearestRoom = nullptr;

    /* Init a variable which counts the number of rooms within the threshold */
    std::size_t bestOverlapCount = 0;

    /* Init a variable which tracks the best overlap distance found */
    double bestOverlapDistance_m = std::numeric_limits<double>::max();

    /* Init a variable which tracks the nearest rooms */
    double nearestDistance = std::numeric_limits<double>::max();

    /* Extract the list of rooms */
    const std::vector<ORB_SLAM3::Room *> allRooms = mpAtlas->GetAllRooms();

    /* Iterate through all the rooms available to find matching candidates */
    for (ORB_SLAM3::Room *mapRoom : allRooms)
    {
        /* Critical: never associate a cluster with an invalid room */
        if (mapRoom == nullptr || mapRoom->isBad())
        {
            continue;
        }

        /* Find the distance from the centroid to the room center */
        const double centroidDistance =
            (clusterCentroid - mapRoom->getCentroid()).norm();

        /* If the distance is greater than the threshold, skip to next room */
        if (centroidDistance >= distanceThreshold)
        {
            continue;
        }

        /*!
         * Keep the closest valid room as a fallback even when plane IDs
         * have changed and there is no wall overlap.
         */
        if (centroidDistance < nearestDistance)
        {
            nearestDistance = centroidDistance;
            nearestRoom     = mapRoom;
        }

        /* Extract existing walls associated with the room */
        const std::vector<ORB_SLAM3::Plane *> existingWalls =
            mapRoom->getWalls();

        /* Init a list of plane ids for the existing walls */
        std::unordered_set<int> existingWallIds;

        /* Allocate memory to the list based on the number of existing walls */
        existingWallIds.reserve(existingWalls.size());

        /* For each existing wall, insert the id */
        for (ORB_SLAM3::Plane *wall : existingWalls)
        {
            if (wall != nullptr && !wall->isBad())
            {
                existingWallIds.insert(wall->getId());
            }
        }

        /* Init a counter which tracks the number of matches */
        std::size_t matches = 0;

        /* Iterate through current wall list and count the number of matches */
        for (ORB_SLAM3::Plane *wall : wallList)
        {
            if (wall != nullptr && !wall->isBad() &&
                existingWallIds.count(wall->getId()) > 0)
            {
                matches++;
            }
        }

        /* If the room candidate is a better match, update best candidate */
        if (matches > bestOverlapCount ||
            (matches == bestOverlapCount && matches > 0 &&
             centroidDistance < bestOverlapDistance_m))
        {
            bestOverlapCount      = matches;
            bestOverlapDistance_m = centroidDistance;
            bestOverlapRoom       = mapRoom;
        }
    }

    /* Create a best match candidate */
    ORB_SLAM3::Room *bestMatch =
        bestOverlapRoom != nullptr ? bestOverlapRoom : nearestRoom;

    /* Output results */
    if (bestMatch != nullptr)
    {
        std::cout << "[SemMgr] Associated cluster with room SE#"
                  << bestMatch->getId() << ": shared walls=" << bestOverlapCount
                  << ", centroid distance="
                  << (clusterCentroid - bestMatch->getCentroid()).norm()
                  << " m." << std::endl;
    }

    return bestMatch;
}

void SemanticsManager::associateAllWallsToRooms(void)
{
    /* Extract rooms */
    std::vector<ORB_SLAM3::Room *> rooms = mpAtlas->GetAllRooms();

    /* Extract all the planes */
    const std::vector<ORB_SLAM3::Plane *> planes = mpAtlas->GetAllPlanes();

    /* Iterate through all the planes to associate to rooms */
    for (ORB_SLAM3::Plane *wall : planes)
    {
        /* Skip wall is the plane is invalid */
        if (wall == nullptr || wall->isBad())
        {
            continue;
        }

        /* Skip planes which are not walls */
        if (wall->getPlaneType() != ORB_SLAM3::Plane::planeVariant::WALL)
        {
            continue;
        }

        /* Check to see that the planes already belong to at least one room */
        bool alreadyAssigned = false;

        /* Iterate through rooms to check if current wall is assigned to room */
        for (ORB_SLAM3::Room *room : rooms)
        {
            /* If the room is bad, skip */
            if (room == nullptr || room->getWalls())
            {
                continue;
            }

            /* Extract the room walls */
            const std::vector<Plane *> roomWalls = room->getWalls();

            alreadyAssigned =
                std::any_of(roomWalls.begin(),
                            roomWalls.end(),
                            [&](ORB_SLAM3::Plane *roomWall) {
                                return roomWall != nullptr &&
                                       roomWall->getId() == wall->getId();
                            });

            if (alreadyAssigned)
            {
                break;
            }
        }
    }
};

void SemanticsManager::assocaitePassagesToRooms(void)
{
    /* Initialize a list of all the rooms */
    const std::vector<ORB_SLAM3::Room *> rooms = mpAtlas->GetAllRooms();

    /* Initialize a list of all the passages */
    const std::vector<ORB_SLAM3::Passage *> passages =
        mpAtlas->GetAllPassages();

    /* Iterate over all the passages to associate with rooms */
    for (ORB_SLAM3::Passage *passage : passages)
    {
        /* If passage is invalid, skip */
        if (passage == nullptr)
        {
            continue;
        }

        /* Find the assoicated walls with the pasasge */
        const std::vector<ORB_SLAM3::Plane *> passageWalls =
            passage->getAssociateWalls();

        /* Iterate through all the rooms to find where the passage is */
        for (ORB_SLAM3::Room *room : rooms)
        {
            /* If room is invalid, skip */
            if (room == nullptr || room->isBad())
            {
                continue;
            }

            /* Extract the walls from the room */
            const std::vector<ORB_SLAM3::Plane *> roomWalls = room->getWalls();

            /* Initialize a flag to indiacte if room shares wall with passage */
            bool sharesWall = false;

            /* For every wall assoicated with a passage, compare to room */
            for (ORB_SLAM3::Plane *passageWall : passageWalls)
            {
                /* If passage wall is invalid skip */
                if (passageWall == nullptr)
                {
                    continue;
                }

                /*!
                 * If any of the room wall id matches with the wall id of the
                 * passage set flag to true.
                 */
                sharesWall = std::any_of(roomWalls.begin(),
                                         roomWalls.end(),
                                         [&](ORB_SLAM3::Plane *roomWall) {
                                             return roomWall != nullptr &&
                                                    roomWall->getId() ==
                                                        passageWall->getId();
                                         });

                /* If wall is shared between passage and wall, break loop */
                if (sharesWall)
                {
                    break;
                }
            }

            /* If no shared wall found, skip to next room */
            if (!sharesWall)
            {
                continue;
            }

            const std::vector<ORB_SLAM3::Passage *> roomPassages =
                room->getPassages();

            const bool alreadyAssociated =
                std::any_of(roomPassages.begin(),
                            roomPassages.end(),
                            [&](ORB_SLAM3::Passage *existing) {
                                return existing != nullptr &&
                                       existing->getId() == passage->getId();
                            });

            if (!alreadyAssociated)
            {
                room->setDoorways(passage);

                std::cout << "[SemMgr] Associated Passage#" << passage->getId()
                          << " with Room#" << room->getId() << "." << std::endl;
            }
        }
    }
}

void SemanticsManager::reAssociateRooms()
{
    // Variables
    double distanceThresh = sysParams->room_seg.center_distance_thresh;
    auto   allRooms       = mpAtlas->GetAllRooms();

    // Loop over all rooms to find duplicates
    for (size_t i = 0; i < allRooms.size(); ++i)
    {
        ORB_SLAM3::Room *room1 = allRooms[i];

        if (!room1 || room1->isBad())
        {
            cout << "ROOM1:" << i << " IS BAD";
            continue;
        }

        for (size_t j = i + 1; j < allRooms.size(); ++j)
        {
            ORB_SLAM3::Room *room2 = allRooms[j];
            if (!room2 || room2->isBad())
            {
                cout << "ROOM2:" << i << " IS BAD";
                continue;
            }

            double distance =
                (room1->getCentroid() - room2->getCentroid()).norm();
            if (distance < distanceThresh)
            {
                // Take all walls from room2 that are not already in room1
                for (auto *wall : room2->getWalls())
                {
                    bool exists = false;
                    for (auto *w : room1->getWalls())
                    {
                        if (wall->getId() == w->getId())
                        {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists)
                        room1->setWalls(wall);
                }

                // Merge room2 into room1
                room2->setBad();
                room1->setRoomVariant(ORB_SLAM3::Room::ROOM);
                room1->setName("Room#" + std::to_string(room1->getId()));

                std::cout << "[SemMgr] Merging Room #" << room2->getId()
                          << " into Room #" << room1->getId()
                          << " due to proximity." << std::endl;
            }
        }
    }
}
} // namespace ORB_SLAM3