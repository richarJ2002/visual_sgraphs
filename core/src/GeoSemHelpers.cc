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

#include "GeoSemHelpers.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace ORB_SLAM3
{
ORB_SLAM3::Plane *GeoSemHelpers::createMapPlane(
    Atlas                                        *mpAtlas,
    ORB_SLAM3::KeyFrame                          *pKF,
    const g2o::Plane3D                            estimatedPlane,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
    ORB_SLAM3::Plane::planeVariant                semanticType,
    double                                        confidence)
{
    ORB_SLAM3::Plane *newMapPlane = new ORB_SLAM3::Plane();
    newMapPlane->setColor();
    newMapPlane->setLocalEquation(estimatedPlane);
    newMapPlane->SetMap(mpAtlas->GetCurrentMap());
    newMapPlane->setId(mpAtlas->GetAllPlanes().size());
    newMapPlane->refKeyFrame = pKF;

    // The observation of the plane
    ORB_SLAM3::Plane::Observation obs;

    // the observation of the plane equation
    obs.localPlane = estimatedPlane;

    // the observation of the plane point cloud (measurement)
    Eigen::Matrix4d Gij;
    Gij.setZero();
    if (SystemParams::GetParams()->optimization.plane_point.enabled)
    {
        for (auto &point : planeCloud->points)
        {
            Eigen::Vector4d pointVec;
            pointVec << point.x, point.y, point.z, 1;
            Gij += pointVec * pointVec.transpose();
        }
    }
    obs.Gij = Gij;

    // the semantic class of the observation
    obs.semanticType = semanticType;

    // the aggregated confidence of the plane
    obs.confidence = confidence;
    newMapPlane->addObservation(pKF, obs);

    // Set the plane type to undefined, as it is not known yet
    newMapPlane->setPlaneType(ORB_SLAM3::Plane::planeVariant::UNDEFINED);

    // Set the global equation of the plane
    g2o::Plane3D globalEquation =
        Utils::applyPoseToPlane(pKF->GetPoseInverse().matrix().cast<double>(),
                                estimatedPlane);
    newMapPlane->setGlobalEquation(globalEquation);

    // transform the plane cloud to the global frame
    pcl::transformPointCloud(*planeCloud,
                             *planeCloud,
                             pKF->GetPoseInverse().matrix().cast<float>());

    // Fill the plane with the pointcloud
    if (!planeCloud->points.empty())
        newMapPlane->setMapClouds(planeCloud);

    if (SystemParams::GetParams()->optimization.plane_map_point.enabled)
    {
        for (const auto &mapPoint : pKF->GetMapPoints())
            if (newMapPlane->isPointinPlaneCloud(
                    mapPoint->GetWorldPos().cast<double>()))
                newMapPlane->setMapPoints(mapPoint);
    }

    pKF->AddMapPlane(newMapPlane);
    mpAtlas->AddMapPlane(newMapPlane);

    return newMapPlane;
}

void GeoSemHelpers::updateMapPlane(
    Atlas                                  *mpAtlas,
    ORB_SLAM3::KeyFrame                    *pKF,
    const g2o::Plane3D                      estimatedPlane,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
    int                                     planeId,
    ORB_SLAM3::Plane::planeVariant          semanticType,
    double                                  confidence)
{
    // Find the matched plane among all planes of the map
    Plane *currentPlane = mpAtlas->GetPlaneById(planeId);

    // The observation of the plane
    ORB_SLAM3::Plane::Observation obs;

    // the observation of the plane equation
    obs.localPlane = estimatedPlane;

    // the observation of the plane point cloud (measurement)
    Eigen::Matrix4d Gij;
    Gij.setZero();
    if (SystemParams::GetParams()->optimization.plane_point.enabled)
    {
        for (auto &point : planeCloud->points)
        {
            Eigen::Vector4d pointVec;
            pointVec << point.x, point.y, point.z, 1;
            Gij += pointVec * pointVec.transpose() *
                   (static_cast<int>(point.a) / 255.0);
        }
    }
    obs.Gij = Gij;

    // the semantic class of the observation
    obs.semanticType = semanticType;

    // the aggregated confidence of the plane
    obs.confidence = confidence;
    currentPlane->addObservation(pKF, obs);

    // Add the plane to the list of planes in the current KeyFrame
    pKF->AddMapPlane(currentPlane);

    // transform the plane cloud to the global frame
    pcl::transformPointCloud(*planeCloud,
                             *planeCloud,
                             pKF->GetPoseInverse().matrix().cast<float>());

    // Update the pointcloud of the plane
    if (!planeCloud->points.empty())
        currentPlane->setMapClouds(planeCloud);

    if (SystemParams::GetParams()->optimization.plane_map_point.enabled)
    {
        for (const auto &mapPoint : pKF->GetMapPoints())
            if (currentPlane->isPointinPlaneCloud(
                    mapPoint->GetWorldPos().cast<double>()))
                currentPlane->setMapPoints(mapPoint);
    }
}

std::pair<bool, std::string> GeoSemHelpers::checkIfMarkerIsDoorway(
    const int                     &markerId,
    std::vector<ORB_SLAM3::Room *> envRooms)
{
    bool        isDoorway = true;
    std::string name      = "";
    // Loop over all markers attached to doorways
    for (const auto &roomPtr : envRooms)
    {
        if (roomPtr->getMetaMarkerId() == markerId)
        {
            isDoorway = false;
            name      = roomPtr->getName();
            break; // No need to continue searching if found
        }
    }
    // Returning
    return std::make_pair(isDoorway, name);
}

void GeoSemHelpers::markerSemanticAnalysis(
    Atlas                         *mpAtlas,
    ORB_SLAM3::KeyFrame           *pKF,
    std::vector<ORB_SLAM3::Room *> envRooms)
{
    // Get the markers from the current KeyFrame
    std::vector<Marker *> mvpMapMarkers = pKF->getCurrentFrameMarkers();

    for (Marker *mCurrentMarker : mvpMapMarkers)
    {
        // Variables
        ORB_SLAM3::Marker *currentMapMarker;

        // Check the type of the marker
        std::pair<bool, std::string> result =
            checkIfMarkerIsDoorway(mCurrentMarker->getId(), envRooms);
        bool        markerIsDoorway = result.first;
        std::string doorwayName     = result.second;

        // Change the marker type
        mCurrentMarker->setMarkerType(
            markerIsDoorway ? ORB_SLAM3::Marker::markerVariant::ON_DOOR
                            : ORB_SLAM3::Marker::markerVariant::ON_ROOM_CENTER);

        // If the marker is not in the map, add it
        if (!mCurrentMarker->isMarkerInGMap())
        {
            mCurrentMarker->setMap(mpAtlas->GetCurrentMap());
            mCurrentMarker->setGlobalPose(pKF->GetPoseInverse() *
                                          mCurrentMarker->getLocalPose());
            mCurrentMarker->setMarkerInGMap(true);

            // Creating a new marker in the map
            currentMapMarker = createMapMarker(mpAtlas, pKF, mCurrentMarker);
        }
        // Else, add the observation to the existing marker
        else
            for (auto mappedMarker : mpAtlas->GetAllMarkers())
                if (mappedMarker->getId() == mCurrentMarker->getId())
                {
                    currentMapMarker = mappedMarker;
                    currentMapMarker->addObservation(
                        pKF,
                        mCurrentMarker->getLocalPose());
                }
    }
}

ORB_SLAM3::Marker *
    GeoSemHelpers::createMapMarker(Atlas                   *mpAtlas,
                                   ORB_SLAM3::KeyFrame     *pKF,
                                   const ORB_SLAM3::Marker *visitedMarker)
{
    ORB_SLAM3::Marker *newMapMarker = new ORB_SLAM3::Marker();

    newMapMarker->setId(visitedMarker->getId());
    newMapMarker->setMap(mpAtlas->GetCurrentMap());
    newMapMarker->setOpId(visitedMarker->getOpId());
    newMapMarker->setTime(visitedMarker->getTime());
    newMapMarker->setLocalPose(visitedMarker->getLocalPose());
    newMapMarker->setGlobalPose(visitedMarker->getGlobalPose());
    newMapMarker->setMarkerType(visitedMarker->getMarkerType());
    newMapMarker->setMarkerInGMap(visitedMarker->isMarkerInGMap());
    newMapMarker->addObservation(pKF, visitedMarker->getLocalPose());

    pKF->AddMapMarker(newMapMarker);
    mpAtlas->AddMapMarker(newMapMarker);

    return newMapMarker;
}

void GeoSemHelpers::createMapPassage(ORB_SLAM3::Atlas *mpAtlas,
                                     ORB_SLAM3::Plane *doorPlane,
                                     ORB_SLAM3::Plane *wallPlane,
                                     bool              isOpenPassage,
                                     Eigen::Vector3f   passageCentroid)
{
    /* ---------------------------------------------------------------------- *
     * VALIDATE REQUIRED INPUTS
     * ---------------------------------------------------------------------- */

    if (mpAtlas == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: Atlas is null."
                  << std::endl;
        return;
    }

    /*!
     * Every passage must be associated with a wall.
     *
     * Closed door:
     *     doorPlane != nullptr
     *     wallPlane != nullptr
     *
     * Open passage:
     *     doorPlane == nullptr
     *     wallPlane != nullptr
     */
    if (wallPlane == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane is null"
                  << std::endl;
        return;
    }

    if (wallPlane->isBad())
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane"
                  << wallPlane->getId() << " is bad." << std::endl;
        return;
    }

    /* Extract all passages */
    const std::vector<ORB_SLAM3::Passage *> allPassages =
        mpAtlas->GetAllPassages();

    /* ---------------------------------------------------------------------- *
     * DETERMINE THE PASSAGE GEOMETRY
     * ---------------------------------------------------------------------- */

    /* Extract the max door height and width */
    double width  = SystemParams::GetParams()->sem_seg.max_door_width;
    double height = SystemParams::GetParams()->sem_seg.max_door_height;

    /* Initialize variables to define the passage */
    Eigen::Vector3f centroid;
    g2o::Plane3D    passageEquation;

    if (doorPlane != nullptr)
    {
        /* Confirm the door plane is not bad */
        if (doorPlane->isBad())
        {
            std::cerr << '[GeoSemHelper] Cannot create passage: door plane '
                      << doorPlane->getId() << " id bad." << std::endl;
            return;
        }

        /* Extract centroid and plane equation */
        centroid        = doorPlane->getCentroid();
        passageEquation = doorPlane->getGlobalEquation();

        /* Extract point cloud of door */
        const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr doorCloud =
            doorPlane->getMapClouds();

        /* Use measured door dimensions when a valid point cloud if available */
        if (doorCloud != nullptr && !doorCloud->empty())
        {
            /* Compute the dimensions of the door */
            const std::pair<double, double> measuredDimensions =
                Utils::computePlaneWidthHeight(doorCloud);

            /* Extract the dimensions of the door from the tuple */
            const double measuredWidth  = measuredDimensions.first;
            const double measuredHeight = measuredDimensions.second;

            /* Clip the width dimension of the door */
            if (std::isfinite(measuredWidth) && measuredWidth > 0.0)
            {
                width = std::min(
                    measuredWidth,
                    static_cast<double>(
                        SystemParams::GetParams()->sem_seg.max_door_width));
            }

            /* Clip the height dimension of the door */
            if (std::isfinite(measuredHeight) && measuredHeight > 0.0)
            {
                height = std::min(
                    measuredHeight,
                    static_cast<double>(
                        SystemParams::GetParams()->sem_seg.max_door_height));
            }
        }
    }
    else
    {
        /*!
         * No door exists. This is an open passage detected from the camera
         * trajectory crossing a wall plane.
         */
        centroid = passageCentroid;

        /* Extract the plane coefficients of the wall */
        Eigen::Vector4d wallEquation = wallPlane->getGlobalEquation().coeffs();

        /* Extract the magnitude of the norm from the coefficients */
        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            std::cerr << "[GeoSemHelper] Cannot create open passage: wall "
                      << wallPlane->getId() << " has an invalid plane equation."
                      << std::endl;
            return;
        }

        /* Normalize the norm vector */
        const Eigen::Vector3d normalizedNormal =
            wallEquation.head<3>() / normalNorm;

        /* Create parrallel plane, passing through detected wall centroid */
        Eigen::Vector4d correctionEquation;
        correctionEquation.head<3>() = normalizedNormal;
        correctionEquation(3) = -normalizedNormal.dot(centroid.cast<double>());

        /* Create an equation for the passage plane */
        passageEquation = g2o::Plane3D(correctionEquation);
    }

    /* Confirm the centroid of the plane is valid */
    if (!centroid.allFinite())
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: invalid centroid."
                  << std::endl;
        return;
    }

    /* ---------------------------------------------------------------------- *
     * CHECK FOR AN EXISTING DUPLICATE BEFORE ALLOCATING ANYTHING
     * ---------------------------------------------------------------------- */

    /* Extract the duplicate distacne threshold */
    const double duplicateDistanceThreshold =
        SystemParams::GetParams()->sem_seg.passage_centroid_distance_thresh;

    /* Extract parameters from passage equation */
    Eigen::Vector4d candidateEquation = passageEquation.coeffs();

    /* Extract the normal norm from the candidate equation */
    const double candidateNormalNorm = candidateEquation.head<3>().norm();

    /* Confirm norm is valid */
    if (candidateNormalNorm < 1e-8)
    {
        std::cerr << "[GeoSemHelper] Cannot create pasasge: invalid candidate "
                     "plane equation"
                  << std::endl;
        return;
    }

    /* Extract the unit norm of the plane */
    const Eigen::Vector3d candidateNormal =
        candidateEquation.head<3>() / candidateNormalNorm;

    /* Iterate through existing passages and see if any passage matches */
    for (ORB_SLAM3::Passage *existingPassage : allPassages)
    {
        /* Confirm that existing passage is valid*/
        if (existingPassage == nullptr)
        {
            continue;
        }

        /* Do not merge an open passage with a blocked passage */
        if (existingPassage->isPassable() != isOpenPassage)
        {
            continue;
        }

        /* Find distance from candidate passage to existing passage centroid */
        const double centroidDistance =
            (centroid - existingPassage->getCentroid()).norm();

        /* If distance is greater than threshold, skip */
        if (centroidDistance >= duplicateDistanceThreshold)
        {
            continue;
        }

        /* Extract the equation of the existing passage */
        Eigen::Vector4d existingEquation =
            existingPassage->getGlobalEquation().coeffs();

        /* Find the normal norm of the existing plane */
        const double existingNormalNorm = existingEquation.head<3>().norm();

        /* Confirm that the norm is valid, otherwise skip */
        if (existingNormalNorm < 1e-8)
        {
            continue;
        }

        /* Extract the unit vector of the norm */
        const Eigen::Vector3d existingNormal =
            existingEquation.head<3>() / existingNormalNorm;

        /*!
         * Find the absolute dot product handles opposite representations of the
         * same plane normal.
         */
        const double normalAlignment =
            std::abs(candidateNormal.dot(existingNormal));

        /* Define a constant expression for the parrallel normal threshold */
        constexpr double parallelNormalThreshold = 0.95;

        /*!
         * If normal alignment is outside threshold skip.
         *
         * @note        normalAlignment = cos(theta),
         *              normalAlignment = 1 @ theta = 0
         */
        if (normalAlignment < parallelNormalThreshold)
        {
            continue;
        }

        std::cout << "[GeoSemHelper] Skipping duplicate passage near Passage#"
                  << existingPassage->getId()
                  << ": centroid distance=" << centroidDistance
                  << " m, normal alignment=" << normalAlignment << "."
                  << std::endl;

        return;
    }

    /* ---------------------------------------------------------------------- *
     * NO PASSAGE MATCH FOUND. GENERATING NEW PASSAGE
     * ---------------------------------------------------------------------- */

    /* Init variable to set the passage id */
    int passageId = 0;

    /* Extract the passages to find the id of the passage */
    for (ORB_SLAM3::Passage *existingPassage : allPassages)
    {
        /* Check that pasasge is valid */
        if (existingPassage != nullptr)
        {
            /* Set passage id to largest id + 1 of the existing passage */
            passageId = std::max(passageId, existingPassage->getId() + 1);
        }
    }

    /* Initialize passage object */
    ORB_SLAM3::Passage *newMapPassage = new ORB_SLAM3::Passage();

    /* Fill passage object */
    newMapPassage->setId(passageId);
    newMapPassage->setMap(mpAtlas->GetCurrentMap());

    newMapPassage->setCentroid(centroid);
    newMapPassage->setGlobalEquation(passageEquation);

    newMapPassage->setWidth(width);
    newMapPassage->setHeight(height);
    newMapPassage->setPassable(isOpenPassage);

    newMapPassage->addAssociateWall(wallPlane);

    /*!
     * Both a detected closed door and a trajectory-detected open
     * passage represent a doorway.
     */
    newMapPassage->setPassageType(ORB_SLAM3::Passage::passageVariant::DOORWAY);

    if (doorPlane != nullptr)
    {
        newMapPassage->setAssociateDoor(doorPlane);
    }

    /* -------------------------------------------------------------- *
     * Insert into the Atlas
     * -------------------------------------------------------------- */

    std::ostringstream infoStream;

    infoStream << (isOpenPassage ? "open" : "blocked") << ", " << std::fixed
               << std::setprecision(2) << width << "x" << height << "m";

    std::cout << "[GeoSemHelper] Creating Passage#" << passageId
              << " associated with wall " << wallPlane->getId();

    if (doorPlane != nullptr)
    {
        std::cout << " and door plane " << doorPlane->getId();
    }

    std::cout << " (" << infoStream.str()
              << "), centroid=" << centroid.transpose() << "." << std::endl;

    mpAtlas->AddMapPassage(newMapPassage);

    std::cout << "[GeoSemHelper] Atlas now contains "
              << mpAtlas->GetAllPassages().size() << " passages." << std::endl;
}

ORB_SLAM3::Room *
    GeoSemHelpers::createBlankRoomCandidate(ORB_SLAM3::Atlas *mpAtlas,
                                            Eigen::Vector3d   centroid)
{
    /* Confirm that the mpAtlas is valid */
    if (mpAtlas == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create room: Atlas is null."
                  << std::endl;

        return nullptr;
    }

    /* Extract the room id */
    const int roomId = static_cast<int>(mpAtlas->GetAllRooms().size());

    /* Create new room object */
    ORB_SLAM3::Room *newRoom = new ORB_SLAM3::Room();

    /*!
     * Constructor diagnostic.
     *
     * At this stage only inspect the bad-state. The ID and room
     * variant have not yet been explicitly assigned.
     */
    std::cout << "[RoomDebug] Immediately after Room construction:" << " bad="
              << static_cast<int>(newRoom->isBad()) << std::endl;

    /*!
     * Initialize the room entity.
     */
    newRoom->setId(roomId);
    newRoom->setCentroid(centroid);
    newRoom->setMap(mpAtlas->GetCurrentMap());
    newRoom->setName("SE#" + std::to_string(roomId));
    newRoom->setRoomVariant(ORB_SLAM3::Room::roomVariant::UNDEFINED);

    /*
     * Diagnostic after explicit initialization.
     */
    std::cout << "[RoomDebug] Initialized room:" << " id=" << newRoom->getId()
              << " bad=" << static_cast<int>(newRoom->isBad())
              << " variant=" << static_cast<int>(newRoom->getRoomVariant())
              << " centroid=" << newRoom->getCentroid().transpose()
              << std::endl;

    /*!
     * Do not add the room to the Atlas here.
     *
     * SemanticsManager already calls:
     *
     *     mpAtlas->AddCandidateMapRoom(room);
     *
     * Adding it here as well would risk duplicate insertion.
     */
    return newRoom;
}

void GeoSemHelpers::associateGroundPlaneToRoom(Atlas           *mpAtlas,
                                               ORB_SLAM3::Room *givenRoom)
{
    std::vector<ORB_SLAM3::Plane *> allWalls = givenRoom->getWalls();
    ORB_SLAM3::Plane               *associatedGroundPlane = nullptr;
    size_t                          maxInliers            = 0;

    // get the ground planes from the Atlas
    std::vector<ORB_SLAM3::Plane *> groundPlanes;
    for (const auto &plane : mpAtlas->GetAllPlanes())
        if (plane->getPlaneType() == ORB_SLAM3::Plane::planeVariant::GROUND)
            groundPlanes.push_back(plane);

    if (groundPlanes.empty())
        // no ground planes in the Atlas
        return;
    else
    {
        // check which ground plane has the most points within the walls
        for (const auto &plane : groundPlanes)
        {
            // count inliers of the plane
            size_t inliers = countGroundPlanePointsWithinWalls(allWalls, plane);

            // update the associated ground plane if the current plane has more
            // inliers
            if (inliers > maxInliers)
            {
                maxInliers            = inliers;
                associatedGroundPlane = plane;
            }
        }

        if (associatedGroundPlane != nullptr)
            givenRoom->setGroundPlane(associatedGroundPlane);
        else
            // set the biggest ground plane as the ground plane of the room
            // [TODO] - logic for when ground plane is not found within the
            // walls
            givenRoom->setGroundPlane(mpAtlas->GetBiggestGroundPlane());
    }
}

size_t GeoSemHelpers::countGroundPlanePointsWithinWalls(
    std::vector<ORB_SLAM3::Plane *> &roomWalls,
    ORB_SLAM3::Plane                *groundPlane)
{
    // [TODO] - verify the correctness of this function
    // the point cloud of the ground plane
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr groundCloud =
        groundPlane->getMapClouds();

    // the number of points within the walls
    size_t count = 0;

    // store the wall equations
    std::vector<Eigen::Vector4d> wallEquations;
    for (const auto &wall : roomWalls)
        wallEquations.push_back(wall->getGlobalEquation().coeffs());

    // for each point in the ground plane, check if it is within the walls
    for (const auto &point : groundCloud->points)
    {
        bool isWithinWalls = true;
        for (const auto &wallEquation : wallEquations)
        {
            // convert the point to Eigen vector
            Eigen::Vector3d pointVec =
                Eigen::Vector3d(point.x, point.y, point.z);

            // substitute the point into the wall equation to get the signed
            // distance
            float signedDistance =
                wallEquation.head<3>().dot(pointVec) + wallEquation(3);

            // if the point is outside the wall, break the loop
            if (signedDistance < 0)
            {
                isWithinWalls = false;
                break;
            }
        }

        // if the point is within the walls, increment the count
        if (isWithinWalls)
            count++;
    }
    return count;
}

void GeoSemHelpers::createMapFloor(ORB_SLAM3::Atlas *mpAtlas)
{
    // Create a new floor object
    Eigen::Vector3d   centroid    = Eigen::Vector3d::Zero();
    ORB_SLAM3::Floor *newMapFloor = new ORB_SLAM3::Floor();

    // Variables
    int floorId = mpAtlas->GetAllFloors().size();

    // Fill the floor entity
    newMapFloor->setOpId(-1);
    newMapFloor->setOpIdG(-1);
    newMapFloor->setId(floorId);
    newMapFloor->setCentroid(centroid);
    newMapFloor->setMap(mpAtlas->GetCurrentMap());
    newMapFloor->setName("Floor#" + std::to_string(floorId));

    // Add the floor to the map
    mpAtlas->AddMapFloor(newMapFloor);

    std::cout << "[GeoSemHelper] Creating Floor#" << newMapFloor->getId()
              << " ..." << std::endl;
}
} // namespace ORB_SLAM3