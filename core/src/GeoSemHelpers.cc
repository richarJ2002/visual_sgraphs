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

/*!
 * @file         GeoSemHelpers.cc
 *
 * @brief        Implements GeoSemHelpers declared in GeoSemHelpers.h.
 */

#include "GeoSemHelpers.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <Eigen/Eigenvalues>

namespace vs_graphs
{
namespace core
{

bool GeoSemHelpers::refitMappedPlaneFromCloud(vs_graphs::core::geometric::Plane *plane)
{
    /* Confirm the mapped plane is valid */
    if (plane == nullptr || plane->isBad())
    {
        return false;
    }

    /* Claim one immutable generation; fitting never observes concurrent growth.
     */
    const std::optional<geometric::Plane::GeometrySnapshot> geometrySnapshot =
        plane->beginMapCloudRefit();

    /* Require sufficient points for a stable covariance estimate */
    if (!geometrySnapshot.has_value() ||
        geometrySnapshot->supportCloud == nullptr ||
        geometrySnapshot->supportCloud->size() < 20)
    {
        return false;
    }

    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud =
        geometrySnapshot->supportCloud;

    /* Calculate the centroid from all valid cloud points */
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    /* Init a counter to count the number of valid point clouds */
    std::size_t validPointCount = 0;

    for (const pcl::PointXYZRGBA &point : cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        centroid += Eigen::Vector3d(static_cast<double>(point.x),
                                    static_cast<double>(point.y),
                                    static_cast<double>(point.z));

        validPointCount++;
    }

    /* Return when too few valid points remain */
    if (validPointCount < 20)
    {
        return false;
    }

    centroid /= static_cast<double>(validPointCount);

    /* Calculate the covariance matrix of the mapped plane cloud */
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

    for (const pcl::PointXYZRGBA &point : cloud->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        const Eigen::Vector3d pointVector(static_cast<double>(point.x),
                                          static_cast<double>(point.y),
                                          static_cast<double>(point.z));

        const Eigen::Vector3d difference = pointVector - centroid;

        covariance += difference * difference.transpose();
    }

    covariance /= static_cast<double>(validPointCount);

    /*!
     * The eigenvector belonging to the smallest eigenvalue is the normal of
     * the best-fitting plane.
     */
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(
        covariance);

    if (eigenSolver.info() != Eigen::Success)
    {
        return false;
    }

    Eigen::Vector3d fittedNormal = eigenSolver.eigenvectors().col(0);

    if (!fittedNormal.allFinite() || fittedNormal.norm() < 1e-8)
    {
        return false;
    }

    fittedNormal.normalize();

    /*!
     * Preserve the previous normal direction to prevent the plane equation
     * from changing sign between updates.
     */
    Eigen::Vector4d previousEquation = geometrySnapshot->equation_World;

    const double previousNormalNorm = previousEquation.head<3>().norm();

    if (std::isfinite(previousNormalNorm) && previousNormalNorm > 1e-8)
    {
        const Eigen::Vector3d previousNormal =
            previousEquation.head<3>() / previousNormalNorm;

        if (fittedNormal.dot(previousNormal) < 0.0)
        {
            fittedNormal *= -1.0;
        }
    }

    /* Construct the fitted global plane equation */
    Eigen::Vector4d fittedEquation;

    fittedEquation.head<3>() = fittedNormal;

    fittedEquation(3) = -fittedNormal.dot(centroid);

    /* Publish the complete fitted geometry and recompute finite bounds once. */
    return plane->completeMapCloudRefit(geometrySnapshot->cloudGeneration,
                                        centroid,
                                        g2o::Plane3D(fittedEquation),
                                        validPointCount);
}

vs_graphs::core::geometric::Plane *GeoSemHelpers::createMapPlane(
    Atlas                                        *p_atlas_inout,
    vs_graphs::core::KeyFrame                          *pKF,
    const g2o::Plane3D                            estimatedPlane,
    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
    vs_graphs::core::geometric::Plane::PlaneVariant                semanticType,
    double                                        confidence)
{
    vs_graphs::core::Map *p_currentMap = p_atlas_inout->GetCurrentMap();

    if (p_currentMap == nullptr)
    {
        return nullptr;
    }

    vs_graphs::core::geometric::Plane *newMapPlane = new vs_graphs::core::geometric::Plane();
    newMapPlane->setColor();
    newMapPlane->setLocalEquation(estimatedPlane);
    newMapPlane->setMap(p_currentMap);
    newMapPlane->setId(p_currentMap->reservePlaneId());
    newMapPlane->refKeyFrame = pKF;

    /* Stamp which face of the physical surface this is, from the camera that
     * observed it. Only the side turned toward a camera can ever be seen, so
     * this position permanently identifies the face -- and therefore which
     * room it bounds -- without any later re-derivation from observation
     * history (see Plane::observationOrigin_World_m). */
    if (pKF != nullptr)
    {
        const Eigen::Vector3d observationOrigin_World_m =
            pKF->GetCameraCenter().cast<double>();

        if (observationOrigin_World_m.allFinite())
        {
            newMapPlane->setObservationOrigin_World(observationOrigin_World_m);
        }
    }

    /* ---------------------------------------------------------------------- *
     * CONSTRUCT POINT PLANE CONSTRAINT MATRIX
     * ---------------------------------------------------------------------- */

    /* Init variable which is used to construct plane constraint matrix  */
    Eigen::Matrix4d pointPlaneConstraintMatrix;

    /* Clear variable */
    pointPlaneConstraintMatrix.setZero();

    /* If plane optimization enabled */
    if (types::SystemParams::getParams()->optimization.planePoint.enabled)
    {
        /* Iterate through points in point cloud */
        for (auto &point : planeCloud->points)
        {
            /* Create the homogeneous coordinate point vector object */
            Eigen::Vector4d pointVec;

            /* Load the point into the vector */
            pointVec << point.x, point.y, point.z, 1;

            /* Accumulate the square matrix from poitns */
            pointPlaneConstraintMatrix += pointVec * pointVec.transpose();
        }
    }

    /* ---------------------------------------------------------------------- *
     * UPDATE OBSERVATION OF PLANE
     * ---------------------------------------------------------------------- */

    /* Init observation struct to store information about the plane */
    vs_graphs::core::geometric::Plane::Observation obs;

    /* Store the result of the plane constaint matrix */
    obs.pointPlaneConstraintMatrix = pointPlaneConstraintMatrix;

    /* Store the observes semantic type of the plane */
    obs.semanticType = semanticType;

    /* Store the aggregatede confidence of the plane */
    obs.confidence = confidence;

    /* Store the equation of the plane with respect to the camera */
    obs.localPlane = estimatedPlane;

    /* ---------------------------------------------------------------------- *
     * UPDATE OBSERVATIONS OF NEW PLANE
     * ---------------------------------------------------------------------- */

    /* Add observation and keyframe to plane */
    newMapPlane->addObservation(pKF, obs);

    /* Set the plane type */
    newMapPlane->setPlaneType(semanticType);

    /* Get the global equation of the plane */
    g2o::Plane3D globalEquation_World =
        Utils::applyPoseToPlane(pKF->GetPoseInverse().matrix().cast<double>(),
                                estimatedPlane);

    /* Set the global equation of the plane in the map world plane */
    newMapPlane->setGlobalEquation(globalEquation_World);

    /* Transform the plane cloud to the global frame */
    pcl::transformPointCloud(*planeCloud,
                             *planeCloud,
                             pKF->GetPoseInverse().matrix().cast<float>());

    /* Fill the plane with the pointcloud */
    if (!planeCloud->points.empty())
    {
        /* Add the point clouds to the new map plane */
        newMapPlane->replaceMapClouds(planeCloud);
        refitMappedPlaneFromCloud(newMapPlane);
    }

    /* ---------------------------------------------------------------------- *
     * ASSOCIATE ORB MAP POINTS WITH THE PLANE
     * ---------------------------------------------------------------------- */

    /*!
     * Associate sparse ORB map landmarks whose world positions are supported by
     * the observed finite plane cloud. These associations may later be used to
     * construct map-point-to-plane constraints during graph optimisation.
     */
    if (types::SystemParams::getParams()->optimization.planeMapPoint.enabled)
    {
        /* Iterate through the orb points (expressed in global frame) */
        for (const auto &mapPoint : pKF->GetMapPoints())
        {
            /* If the orb feature is within the plane, set as map point */
            if (newMapPlane->isPointinPlaneCloud(
                    mapPoint->GetWorldPos().cast<double>()))
            {
                newMapPlane->setMapPoints(mapPoint);
            }
        }
    }

    /* Add the plane to the keyframe */
    pKF->AddMapPlane(newMapPlane);

    /* Add the palne to the current map */
    p_atlas_inout->AddMapPlane(newMapPlane);

    return newMapPlane;
}

void GeoSemHelpers::updateMapPlane(
    Atlas                                  *p_atlas_in,
    vs_graphs::core::KeyFrame                    *pKF,
    const g2o::Plane3D                      estimatedPlane,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr planeCloud,
    int                                     planeId,
    vs_graphs::core::geometric::Plane::PlaneVariant          semanticType,
    double                                  confidence)
{
    // Find the matched plane among all planes of the map
    vs_graphs::core::geometric::Plane *currentPlane = p_atlas_in->GetPlaneById(planeId);

    // The observation of the plane
    vs_graphs::core::geometric::Plane::Observation obs;

    // the observation of the plane equation
    obs.localPlane = estimatedPlane;

    // the observation of the plane point cloud (measurement)
    Eigen::Matrix4d pointPlaneConstraintMatrix;
    pointPlaneConstraintMatrix.setZero();
    if (types::SystemParams::getParams()->optimization.planePoint.enabled)
    {
        for (auto &point : planeCloud->points)
        {
            Eigen::Vector4d pointVec;
            pointVec << point.x, point.y, point.z, 1;
            pointPlaneConstraintMatrix += pointVec * pointVec.transpose() *
                                          (static_cast<int>(point.a) / 255.0);
        }
    }
    obs.pointPlaneConstraintMatrix = pointPlaneConstraintMatrix;

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

    /* Update the point cloud of the mapped plane */
    if (!planeCloud->empty())
    {
        currentPlane->setMapClouds(planeCloud);

        /*!
         * Refit the mapped global equation from the complete accumulated point
         * cloud.
         *
         * @note        Without refitting, the point cloud and centroid change
         *              but the original plane equation becomes stale.
         */
        refitMappedPlaneFromCloud(currentPlane);
    }

    if (types::SystemParams::getParams()->optimization.planeMapPoint.enabled)
    {
        for (const auto &mapPoint : pKF->GetMapPoints())
            if (currentPlane->isPointinPlaneCloud(
                    mapPoint->GetWorldPos().cast<double>()))
                currentPlane->setMapPoints(mapPoint);
    }
}

std::pair<bool, std::string> GeoSemHelpers::checkIfMarkerIsDoorway(
    const int                     &markerId,
    std::vector<vs_graphs::core::semantic::Room *> envRooms)
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
    Atlas                         *p_atlas_inout,
    vs_graphs::core::KeyFrame           *pKF,
    std::vector<vs_graphs::core::semantic::Room *> envRooms)
{
    // Get the markers from the current KeyFrame
    std::vector<semantic::Marker *> mvpMapMarkers = pKF->getCurrentFrameMarkers();

    for (semantic::Marker *mCurrentMarker : mvpMapMarkers)
    {
        // Variables
        vs_graphs::core::semantic::Marker *currentMapMarker;

        // Check the type of the marker
        std::pair<bool, std::string> result =
            checkIfMarkerIsDoorway(mCurrentMarker->getId(), envRooms);
        bool        markerIsDoorway = result.first;
        std::string doorwayName     = result.second;

        // Change the marker type
        mCurrentMarker->setMarkerType(
            markerIsDoorway
                ? vs_graphs::core::semantic::Marker::MarkerVariant::ON_DOOR
                : vs_graphs::core::semantic::Marker::MarkerVariant::
                      ON_ROOM_CENTER);

        // If the marker is not in the map, add it
        if (!mCurrentMarker->isMarkerInGMap())
        {
            mCurrentMarker->setMap(p_atlas_inout->GetCurrentMap());
            mCurrentMarker->setGlobalPose(pKF->GetPoseInverse() *
                                          mCurrentMarker->getLocalPose());
            mCurrentMarker->setMarkerInGMap(true);

            // Creating a new marker in the map
            currentMapMarker = createMapMarker(p_atlas_inout, pKF, mCurrentMarker);
        }
        // Else, add the observation to the existing marker
        else
            for (auto mappedMarker : p_atlas_inout->GetAllMarkers())
                if (mappedMarker->getId() == mCurrentMarker->getId())
                {
                    currentMapMarker = mappedMarker;
                    currentMapMarker->addObservation(
                        pKF,
                        mCurrentMarker->getLocalPose());
                }
    }
}

vs_graphs::core::semantic::Marker *
    GeoSemHelpers::createMapMarker(Atlas                   *p_atlas_inout,
                                   vs_graphs::core::KeyFrame     *pKF,
                                   const vs_graphs::core::semantic::Marker *visitedMarker)
{
    vs_graphs::core::semantic::Marker *newMapMarker = new vs_graphs::core::semantic::Marker();

    newMapMarker->setId(visitedMarker->getId());
    newMapMarker->setMap(p_atlas_inout->GetCurrentMap());
    newMapMarker->setOpId(visitedMarker->getOpId());
    newMapMarker->setTime(visitedMarker->getTime());
    newMapMarker->setLocalPose(visitedMarker->getLocalPose());
    newMapMarker->setGlobalPose(visitedMarker->getGlobalPose());
    newMapMarker->setMarkerType(visitedMarker->getMarkerType());
    newMapMarker->setMarkerInGMap(visitedMarker->isMarkerInGMap());
    newMapMarker->addObservation(pKF, visitedMarker->getLocalPose());

    pKF->AddMapMarker(newMapMarker);
    p_atlas_inout->AddMapMarker(newMapMarker);

    return newMapMarker;
}

void GeoSemHelpers::createMapPassage(vs_graphs::core::Atlas *p_atlas_inout,
                                     vs_graphs::core::geometric::Plane *p_doorPlane_in,
                                     vs_graphs::core::geometric::Plane *p_wallPlane_in,
                                     bool              isOpenPassage_in,
                                     Eigen::Vector3d passageCentroid_World_m_in)
{
    /* ---------------------------------------------------------------------- *
     * VALIDATE REQUIRED INPUTS
     * ---------------------------------------------------------------------- */

    if (p_atlas_inout == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: Atlas is null."
                  << std::endl;
        return;
    }

    /*!
     * Every passage must be associated with a wall.
     *
     * Closed door:
     *     p_doorPlane_in != nullptr
     *     p_wallPlane_in != nullptr
     *
     * Open passage:
     *     p_doorPlane_in == nullptr
     *     p_wallPlane_in != nullptr
     */
    if (p_wallPlane_in == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane is null"
                  << std::endl;
        return;
    }

    if (p_wallPlane_in->isBad())
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane"
                  << p_wallPlane_in->getId() << " is bad." << std::endl;
        return;
    }

    /*!
     * Passage creation requires the supporting wall to have sufficient
     * observations. This prevents spurious passages on isolated wall segments
     * that have no evidence.
     */
    bool         wallHasConfirmedRoom = false;
    const size_t minObs =
        types::SystemParams::getParams()->roomSeg.minimumWallObservationCount;
    if (p_wallPlane_in->getObservationCount() >= minObs)
    {
        wallHasConfirmedRoom = true;
    }

    if (!wallHasConfirmedRoom)
    {
        std::cerr << "[GeoSemHelper] Cannot create passage: wall plane "
                  << p_wallPlane_in->getId()
                  << " has insufficient observations ("
                  << p_wallPlane_in->getObservationCount() << " < " << minObs
                  << ")." << std::endl;
        return;
    }

    /* Extract all passages */
    const std::vector<vs_graphs::core::semantic::Passage *> allPassages =
        p_atlas_inout->GetAllPassages();

    /* ---------------------------------------------------------------------- *
     * DETERMINE THE PASSAGE GEOMETRY
     * ---------------------------------------------------------------------- */

    /* Extract the max door height and width */
    double width  = types::SystemParams::getParams()->semSeg.maxDoorWidth;
    double height = types::SystemParams::getParams()->semSeg.maxDoorHeight;

    /* Initialize variables to define the passage */
    Eigen::Vector3d centroid;
    g2o::Plane3D    passageEquation;

    if (p_doorPlane_in != nullptr)
    {
        /* Confirm the door plane is not bad */
        if (p_doorPlane_in->isBad())
        {
            std::cerr << "[GeoSemHelper] Cannot create passage: door plane "
                      << p_doorPlane_in->getId() << " is bad." << std::endl;
            return;
        }

        /* Extract centroid and plane equation */
        centroid        = p_doorPlane_in->getCentroid();
        passageEquation = p_doorPlane_in->getGlobalEquation();

        /* Extract point cloud of door */
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr doorCloud =
            p_doorPlane_in->getGeometrySnapshot().supportCloud;

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
                        types::SystemParams::getParams()->semSeg.maxDoorWidth));
            }

            /* Clip the height dimension of the door */
            if (std::isfinite(measuredHeight) && measuredHeight > 0.0)
            {
                height = std::min(
                    measuredHeight,
                    static_cast<double>(
                        types::SystemParams::getParams()->semSeg.maxDoorHeight));
            }
        }
    }
    else
    {
        /*!
         * No door exists. This is an open passage detected from the camera
         * trajectory crossing a wall plane.
         */
        centroid = passageCentroid_World_m_in;

        /* Extract the plane coefficients of the wall */
        Eigen::Vector4d wallEquation =
            p_wallPlane_in->getGlobalEquation().coeffs();

        /* Extract the magnitude of the norm from the coefficients */
        const double normalNorm = wallEquation.head<3>().norm();

        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            std::cerr << "[GeoSemHelper] Cannot create open passage: wall "
                      << p_wallPlane_in->getId()
                      << " has an invalid plane equation." << std::endl;
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
        types::SystemParams::getParams()->semSeg.passageCentroidDistanceThresh;

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
    for (vs_graphs::core::semantic::Passage *p_existingPassage : allPassages)
    {
        /* Confirm that existing passage is valid*/
        if (p_existingPassage == nullptr)
        {
            continue;
        }

        /* Find the distance from centroid to passage */
        Eigen::Vector3d centroidDistanceVector =
            (centroid - p_existingPassage->getCentroid());

        /* Find distance from candidate passage to existing passage centroid */
        const double centroidDistance = centroidDistanceVector.norm();

        /* If distance is greater than threshold, skip */
        if (centroidDistance >= duplicateDistanceThreshold)
        {
            continue;
        }

        /* Extract the equation of the existing passage */
        Eigen::Vector4d existingEquation =
            p_existingPassage->getGlobalEquation().coeffs();

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

        /*
         * Parallel openings on unrelated nearby walls are not duplicates.
         * Permit the two observed faces of one physical wall, but require the
         * passage centroids to remain close to the opposite passage plane.
         */
        constexpr double      maximumSupportingPlaneSeparation_m = 0.30;
        const Eigen::Vector4d normalizedCandidateEquation =
            candidateEquation / candidateNormalNorm;
        const Eigen::Vector4d normalizedExistingEquation =
            existingEquation / existingNormalNorm;
        const double candidatePlaneResidual_m =
            std::abs(normalizedCandidateEquation.head<3>().dot(
                         p_existingPassage->getCentroid()) +
                     normalizedCandidateEquation(3));
        const double existingPlaneResidual_m =
            std::abs(normalizedExistingEquation.head<3>().dot(centroid) +
                     normalizedExistingEquation(3));

        if (candidatePlaneResidual_m > maximumSupportingPlaneSeparation_m ||
            existingPlaneResidual_m > maximumSupportingPlaneSeparation_m)
        {
            continue;
        }

        /*
         * Passage state is evidence, not identity. Confirmed connected free
         * space promotes a prior passage hypothesis to traversable; later door
         * observations never downgrade that stronger evidence.
         *
         * A passage is expected to have TWO observed wall faces that offset a
         * physical wall (the near face and the opposite face). Geometry must
         * never be re-anchored onto the face that happens to have been seen
         * last, or the passage plane flips between the two faces from cycle to
         * cycle and every side-dependent decision (far-side holds, prospective
         * placement) flips with it. Therefore:
         *   - When this wall is already an associated supporting face, refresh
         *     centroid/equation normally (same-face refinement).
         *   - When this is the FIRST time we see the *opposite* face of the
         *     same physical wall (not yet among the supporting faces), only add
         *     it to the passage. The passage plane stays anchored to the face
         *     that framed the passage; a downstream mid-plane pass pairs the
         *     two faces and recomputes a stable aperture plane.
         */
        const std::vector<vs_graphs::core::geometric::Plane *> existingSupportingWalls =
            p_existingPassage->getAssociateWalls();
        const bool isKnownSupportingFace =
            std::find(existingSupportingWalls.begin(),
                      existingSupportingWalls.end(),
                      p_wallPlane_in) != existingSupportingWalls.end();

        if (isOpenPassage_in)
        {
            p_existingPassage->setPassable(true);
            p_existingPassage->setCentroid(centroid);

            if (isKnownSupportingFace)
            {
                p_existingPassage->setGlobalEquation(passageEquation);
            }
        }

        p_existingPassage->addAssociateWall(p_wallPlane_in);

        if (p_doorPlane_in != nullptr &&
            p_existingPassage->getAssociateDoor() == nullptr)
        {
            p_existingPassage->setAssociateDoor(p_doorPlane_in);
        }

        std::cout << "[GeoSemHelper] Updated existing semantic::Passage#"
                  << p_existingPassage->getId()
                  << ": centroid distance=" << centroidDistance
                  << " m, normal alignment=" << normalAlignment << ", state="
                  << (p_existingPassage->isPassable() ? "open" : "blocked")
                  << "." << std::endl;

        return;
    }

    /* ---------------------------------------------------------------------- *
     * NO PASSAGE MATCH FOUND. GENERATING NEW PASSAGE
     * ---------------------------------------------------------------------- */

    /* Passage identities belong to the mission, not to an active SLAM map. */
    const int passageId = p_atlas_inout->reservePassageIdentity();

    /* Initialize passage object */
    vs_graphs::core::semantic::Passage *p_newMapPassage = new vs_graphs::core::semantic::Passage();

    /* Fill passage object */
    p_newMapPassage->setId(passageId);
    p_newMapPassage->setMap(p_atlas_inout->GetCurrentMap());

    p_newMapPassage->setCentroid(centroid);
    p_newMapPassage->setGlobalEquation(passageEquation);

    p_newMapPassage->setWidth(width);
    p_newMapPassage->setHeight(height);
    p_newMapPassage->setPassable(isOpenPassage_in);

    p_newMapPassage->addAssociateWall(p_wallPlane_in);

    /*!
     * Both a detected closed door and a trajectory-detected open
     * passage represent a doorway.
     */
    p_newMapPassage->setPassageType(
        vs_graphs::core::semantic::Passage::PassageVariant::DOORWAY);

    if (p_doorPlane_in != nullptr)
    {
        p_newMapPassage->setAssociateDoor(p_doorPlane_in);
    }

    /* -------------------------------------------------------------- *
     * Insert into the Atlas
     * -------------------------------------------------------------- */

    std::ostringstream infoStream;

    infoStream << (isOpenPassage_in ? "open" : "blocked") << ", " << std::fixed
               << std::setprecision(2) << width << "x" << height << "m";

    std::cout << "[GeoSemHelper] Creating semantic::Passage#" << passageId
              << " associated with wall " << p_wallPlane_in->getId();

    if (p_doorPlane_in != nullptr)
    {
        std::cout << " and door plane " << p_doorPlane_in->getId();
    }

    std::cout << " (" << infoStream.str()
              << "), centroid=" << centroid.transpose() << "." << std::endl;

    p_atlas_inout->AddMapPassage(p_newMapPassage);

    std::cout << "[GeoSemHelper] Atlas now contains "
              << p_atlas_inout->GetAllPassages().size() << " passages."
              << std::endl;
}

vs_graphs::core::semantic::Room *
    GeoSemHelpers::createBlankRoomCandidate(vs_graphs::core::Atlas  *p_atlas_inout,
                                            Eigen::Vector3d    centroid,
                                            std::optional<int> stableRoomId_in)
{
    /* Confirm that the p_atlas_inout is valid */
    if (p_atlas_inout == nullptr)
    {
        std::cerr << "[GeoSemHelper] Cannot create room: Atlas is null."
                  << std::endl;

        return nullptr;
    }

    /* Extract the existing rooms from the map */
    const std::vector<vs_graphs::core::semantic::Room *> existingRooms = p_atlas_inout->GetAllRooms();

    /*!
     * Hard invariant, enforced at this single room-creation choke point
     * (this is the only call site in the codebase that ever constructs a
     * new vs_graphs::core::semantic::Room): a map may hold at most one more room than it
     * has PASSABLE passages. A map's first room is either the mission's cold
     * bootstrap or the topology-only recovery proxy restored after tracking
     * loss; it needs no active-map passage yet -- that is the "+1". Every new
     * semantic room after that must be the confirmed or prospective far side
     * of a genuine passage.
     *
     * Deliberately counts isPassable() passages only, not every registered
     * Passage object: a Passage can also be created "blocked" purely from a
     * classified door plane sitting near a wall (detectDoorsAndDoorways(),
     * GeoSemHelpers.cc's createMapPassage() called with isOpenPassage_in =
     * false) -- no free-space evidence at all. Counting that toward the
     * budget would let a semantic door classification alone unlock a new
     * room the same way real passage evidence does, which is exactly the
     * loophole this gate exists to close. A blocked passage earns its
     * budget slot only once it is actually observed passable (Connected
     * ESDF free space through the wall -- see createMapPassage()'s
     * isOpenPassage_in = true path, reached only from
     * updatePassages()'s skeleton-crossing candidates), matching this
     * project's rule: a room may only be created from a genuine
     * free-space-skeleton-crosses-wall observation, never a toggled
     * passable/blocked state alone.
     */
    const std::vector<vs_graphs::core::semantic::Passage *> currentMapPassages =
        p_atlas_inout->GetAllPassages();
    const std::size_t passablePassageCount =
        std::count_if(currentMapPassages.begin(),
                      currentMapPassages.end(),
                      [](vs_graphs::core::semantic::Passage *p_passage)
                      {
                          return p_passage != nullptr && !p_passage->isBad() &&
                                 p_passage->isPassable();
                      });

    /* Recovery (explicit stable ID) restores an already-discovered identity
     * after a tracking-loss reset; it is not new discovery and must not be
     * blocked by the passage budget. The budget gates discovery only. */
    if (!stableRoomId_in.has_value() &&
        existingRooms.size() > passablePassageCount)
    {
        std::cerr << "[GeoSemHelper] Refusing to create a new room: "
                  << existingRooms.size() << " room(s) already exist against "
                  << passablePassageCount
                  << " passable passage(s) in this map -- room count may "
                     "never exceed passable-passage count + 1."
                  << std::endl;

        return nullptr;
    }

    const int roomId = stableRoomId_in.has_value()
                           ? *stableRoomId_in
                           : p_atlas_inout->reserveRoomIdentity();
    p_atlas_inout->observeRoomIdentity(roomId);

    /* Create new room */
    vs_graphs::core::semantic::Room *newRoom = new vs_graphs::core::semantic::Room();

    /*!
     * Fill the parameters of room. The caller is responsible for inserting it
     * with:
     *      p_atlas_inout->AddCandidateMapRoom(newRoom);
     */

    newRoom->setId(roomId);
    newRoom->setCentroid(centroid);
    newRoom->setMap(p_atlas_inout->GetCurrentMap());

    newRoom->setName("SE#" + std::to_string(roomId));

    newRoom->setRoomVariant(vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED);

    std::cout << "[GeoSemHelper] Created provisional SE#" << newRoom->getId()
              << " at " << newRoom->getCentroid().transpose() << "."
              << std::endl;

    return newRoom;
}

void GeoSemHelpers::associateGroundPlaneToRoom(Atlas           *p_atlas_in,
                                               vs_graphs::core::semantic::Room *givenRoom)
{
    std::vector<vs_graphs::core::geometric::Plane *> allWalls = givenRoom->getWalls();
    vs_graphs::core::geometric::Plane               *associatedGroundPlane = nullptr;
    size_t                          maxInliers            = 0;

    // get the ground planes from the Atlas
    std::vector<vs_graphs::core::geometric::Plane *> groundPlanes;
    for (const auto &plane : p_atlas_in->GetAllPlanes())
        if (plane->getPlaneType() == vs_graphs::core::geometric::Plane::PlaneVariant::GROUND)
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
            givenRoom->setGroundPlane(p_atlas_in->GetBiggestGroundPlane());
    }
}

size_t GeoSemHelpers::countGroundPlanePointsWithinWalls(
    std::vector<vs_graphs::core::geometric::Plane *> &roomWalls,
    vs_graphs::core::geometric::Plane                *groundPlane)
{
    // [TODO] - verify the correctness of this function
    // the point cloud of the ground plane
    const geometric::Plane::GeometrySnapshot groundGeometry =
        groundPlane->getGeometrySnapshot();
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr groundCloud =
        groundGeometry.supportCloud;

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

void GeoSemHelpers::createMapFloor(vs_graphs::core::Atlas  *p_atlas_inout,
                                   std::optional<int> stableFloorId_in)
{
    vs_graphs::core::Map *p_currentMap = p_atlas_inout->GetCurrentMap();

    if (p_currentMap == nullptr)
    {
        return;
    }

    // Create a new floor object
    Eigen::Vector3d   centroid    = Eigen::Vector3d::Zero();
    vs_graphs::core::semantic::Floor *newMapFloor = new vs_graphs::core::semantic::Floor();

    // Variables
    const int floorId = stableFloorId_in.has_value()
                            ? *stableFloorId_in
                            : p_atlas_inout->reserveFloorIdentity();
    p_atlas_inout->observeFloorIdentity(floorId);

    // Fill the floor entity
    newMapFloor->setOpId(-1);
    newMapFloor->setOpIdG(-1);
    newMapFloor->setId(floorId);
    newMapFloor->setCentroid(centroid);
    newMapFloor->setMap(p_currentMap);
    newMapFloor->setName("semantic::Floor#" + std::to_string(floorId));

    // Add the floor to the map
    p_atlas_inout->AddMapFloor(newMapFloor);

    std::cout << "[GeoSemHelper] Creating semantic::Floor#" << newMapFloor->getId()
              << " ..." << std::endl;
}
} // namespace core
} // namespace vs_graphs
