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

#include "Geometric/Plane.h"
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <limits>
#include <pcl/octree/octree_search.h>

namespace ORB_SLAM3
{
Plane::Plane(void)
{
    id    = -1;
    opId  = -1;
    opIdG = -1;

    mbBad     = false;
    planeType = Plane::planeVariant::UNDEFINED;

    centroid.setZero();

    mpMap = nullptr;

    refKeyFrame     = nullptr;
    mnBAGlobalForKF = 0;

    planeCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    octree = boost::make_shared<
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA>>(
        SystemParams::GetParams()->refine_map_points.octree.resolution);

    minPlaneU = std::numeric_limits<double>::max();
    maxPlaneU = std::numeric_limits<double>::lowest();
    minPlaneV = std::numeric_limits<double>::max();
    maxPlaneV = std::numeric_limits<double>::lowest();
}
Plane::~Plane() {}

/* -------------------------------------------------------------------------- *
 * UTILITY METHODS
 * -------------------------------------------------------------------------- */

int Plane::getId() const
{
    return id;
}

void Plane::setId(int value)
{
    id = value;
}

int Plane::getOpId() const
{
    return opId;
}

void Plane::setOpId(int value)
{
    opId = value;
}

int Plane::getOpIdG() const
{
    return opIdG;
}

void Plane::setOpIdG(int value)
{
    opIdG = value;
}

void Plane::setBad(void)
{
    unique_lock<mutex> lock(mMutexType);
    mbBad = true;
}

bool Plane::isBad(void)
{
    unique_lock<mutex> lock(mMutexType);
    return mbBad;
}

std::vector<uint8_t> Plane::getColor() const
{
    return color;
}

void Plane::setColor(void)
{
    if (color.size() == 0)
    {
        color.push_back(rand() % 256);
        color.push_back(rand() % 256);
        color.push_back(rand() % 256);
    }
}

std::set<MapPoint *> Plane::getMapPoints(void)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mapPoints;
}

void Plane::setMapPoints(MapPoint *value)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mapPoints.insert(value);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Plane::getMapClouds(void)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return planeCloud;
}

Plane::GeometrySnapshot Plane::getGeometrySnapshot(void) const
{
    std::scoped_lock lock(mMutexPos, mMutexFeatures);
    GeometrySnapshot snapshot;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudCopy(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (planeCloud != nullptr)
    {
        *cloudCopy = *planeCloud;
    }

    snapshot.supportCloud              = cloudCopy;
    snapshot.equation_World            = globalEquation.coeffs();
    snapshot.centroid_World_m          = centroid;
    snapshot.minPlaneU_m               = minPlaneU;
    snapshot.maxPlaneU_m               = maxPlaneU;
    snapshot.minPlaneV_m               = minPlaneV;
    snapshot.maxPlaneV_m               = maxPlaneV;
    snapshot.finiteSupportCount        = lastSuccessfulRefitFinitePointCount;
    snapshot.observationCount          = observationCount;
    snapshot.cloudGeneration           = cloudGeneration;
    snapshot.successfulRefitGeneration = successfulRefitGeneration;
    return snapshot;
}

Plane::ObservationSideSnapshot Plane::getObservationSideSnapshot(
    const Eigen::Vector4d &normalizedEquation_World_in) const
{
    ObservationSideSnapshot snapshot;
    if (!normalizedEquation_World_in.allFinite() ||
        std::abs(normalizedEquation_World_in.head<3>().norm() - 1.0) > 1e-3)
    {
        return snapshot;
    }

    constexpr double minimumReliableSideDistance_m = 0.10;
    constexpr double minimumSignConsensusRatio = 0.75;
    std::vector<double> signedDistances_m;

    for (const auto &[p_keyFrame, observation] : getObservations())
    {
        static_cast<void>(observation);
        if (p_keyFrame == nullptr || p_keyFrame->isBad())
        {
            continue;
        }

        const Eigen::Vector3d cameraCenter_World_m =
            p_keyFrame->GetCameraCenter().cast<double>();
        const double signedDistance_m =
            normalizedEquation_World_in.head<3>().dot(cameraCenter_World_m) +
            normalizedEquation_World_in(3);
        if (cameraCenter_World_m.allFinite() &&
            std::isfinite(signedDistance_m) &&
            std::abs(signedDistance_m) >= minimumReliableSideDistance_m)
        {
            signedDistances_m.push_back(signedDistance_m);
        }
    }

    snapshot.evidenceCount = signedDistances_m.size();
    if (signedDistances_m.empty())
    {
        return snapshot;
    }

    const std::size_t positiveCount = static_cast<std::size_t>(std::count_if(
        signedDistances_m.begin(),
        signedDistances_m.end(),
        [](const double distance_m) { return distance_m > 0.0; }));
    const std::size_t negativeCount = signedDistances_m.size() - positiveCount;
    const std::size_t consensusCount = std::max(positiveCount, negativeCount);
    snapshot.consensusRatio = static_cast<double>(consensusCount) /
                              static_cast<double>(signedDistances_m.size());
    if (snapshot.consensusRatio < minimumSignConsensusRatio)
    {
        snapshot.face = ObservationSideSnapshot::Face::AMBIGUOUS;
        return snapshot;
    }

    const bool positiveConsensus = positiveCount >= negativeCount;
    snapshot.face = positiveConsensus ? ObservationSideSnapshot::Face::POSITIVE
                                      : ObservationSideSnapshot::Face::NEGATIVE;
    signedDistances_m.erase(
        std::remove_if(signedDistances_m.begin(),
                       signedDistances_m.end(),
                       [positiveConsensus](const double distance_m)
                       { return (distance_m > 0.0) != positiveConsensus; }),
        signedDistances_m.end());
    const std::size_t medianIndex = signedDistances_m.size() / 2U;
    std::nth_element(signedDistances_m.begin(),
                     signedDistances_m.begin() + medianIndex,
                     signedDistances_m.end());
    snapshot.medianSignedDistance_m = signedDistances_m[medianIndex];
    return snapshot;
}

/* -------------------------------------------------------------------------- *
 * FUNCTINAL METHODS
 * -------------------------------------------------------------------------- */

void Plane::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    std::scoped_lock lock(mMutexPos, mMutexType, mMutexFeatures);

    /* ---------------------------------------------------------------------- *
     * Transform centroid
     * ---------------------------------------------------------------------- */

    centroid = transform_oldWorldToNewWorld_in.map(centroid);

    /* ---------------------------------------------------------------------- *
     * Transform plane point cloud
     * ---------------------------------------------------------------------- */

    for (auto &point : planeCloud->points)
    {
        Eigen::Vector3d pointVector(point.x, point.y, point.z);

        pointVector = transform_oldWorldToNewWorld_in.map(pointVector);

        point.x = pointVector.x();
        point.y = pointVector.y();
        point.z = pointVector.z();
    }

    /* ---------------------------------------------------------------------- *
     * Transform plane equations
     * ---------------------------------------------------------------------- */

    globalEquation =
        transformPlaneEquation(globalEquation, transform_oldWorldToNewWorld_in);

    /* ---------------------------------------------------------------------- *
     * Rebuild spatial index
     * ---------------------------------------------------------------------- */

    octree->deleteTree();
    octree->setInputCloud(planeCloud);
    octree->addPointsFromInputCloud();

    /* Recompute the finite bounds in the transformed frame. */
    updatePlaneBoundsWithoutLock();
    ++cloudGeneration;
    successfulRefitGeneration = cloudGeneration;
}

void Plane::alignGeometryToEquation(
    const g2o::Plane3D &targetEquation_NewWorld_in)
{
    std::scoped_lock lock(mMutexPos, mMutexType, mMutexFeatures);

    Eigen::Vector4d currentCoefficients = globalEquation.coeffs();
    Eigen::Vector4d targetCoefficients  = targetEquation_NewWorld_in.coeffs();

    const double currentNormalNorm = currentCoefficients.head<3>().norm();
    const double targetNormalNorm  = targetCoefficients.head<3>().norm();

    if (!currentCoefficients.allFinite() || !targetCoefficients.allFinite() ||
        currentNormalNorm < 1e-12 || targetNormalNorm < 1e-12)
    {
        return;
    }

    currentCoefficients /= currentNormalNorm;
    targetCoefficients /= targetNormalNorm;

    /* Plane equations are sign-equivalent; use the smaller rotation. */
    if (currentCoefficients.head<3>().dot(targetCoefficients.head<3>()) < 0.0)
    {
        targetCoefficients = -targetCoefficients;
    }

    const Eigen::Vector3d currentNormal = currentCoefficients.head<3>();
    const Eigen::Vector3d targetNormal  = targetCoefficients.head<3>();

    const double normalDotProduct =
        std::clamp(currentNormal.dot(targetNormal), -1.0, 1.0);

    Eigen::Quaterniond rotation_oldPlaneToOptimizedPlane;

    if (normalDotProduct > 1.0 - 1e-12)
    {
        rotation_oldPlaneToOptimizedPlane = Eigen::Quaterniond::Identity();
    }
    else if (normalDotProduct < -1.0 + 1e-12)
    {
        const Eigen::Vector3d referenceAxis = std::abs(currentNormal.x()) < 0.9
                                                  ? Eigen::Vector3d::UnitX()
                                                  : Eigen::Vector3d::UnitY();
        const Eigen::Vector3d rotationAxis =
            currentNormal.cross(referenceAxis).normalized();

        rotation_oldPlaneToOptimizedPlane = Eigen::Quaterniond(
            Eigen::AngleAxisd(std::acos(-1.0), rotationAxis));
    }
    else
    {
        const Eigen::Vector3d rotationAxis = currentNormal.cross(targetNormal);

        rotation_oldPlaneToOptimizedPlane =
            Eigen::Quaterniond(1.0 + normalDotProduct,
                               rotationAxis.x(),
                               rotationAxis.y(),
                               rotationAxis.z())
                .normalized();
    }

    if (!rotation_oldPlaneToOptimizedPlane.coeffs().allFinite())
    {
        return;
    }

    /*!
     * Rotate about a point on the current plane, then translate only along
     * the optimized normal until the transformed support lies on the target
     * equation. This avoids an arbitrary rotation about the world origin.
     */
    const double centroidSignedDistance_m =
        currentNormal.dot(centroid) + currentCoefficients(3);

    const Eigen::Vector3d anchor_OldWorld_m =
        centroid - centroidSignedDistance_m * currentNormal;

    const Eigen::Matrix3d rotationMatrix =
        rotation_oldPlaneToOptimizedPlane.toRotationMatrix();

    const Eigen::Vector3d rotationTranslation_m =
        anchor_OldWorld_m - rotationMatrix * anchor_OldWorld_m;

    const double distanceAfterRotation_m =
        currentCoefficients(3) - targetNormal.dot(rotationTranslation_m);

    const Eigen::Vector3d normalTranslation_m =
        (distanceAfterRotation_m - targetCoefficients(3)) * targetNormal;

    const Eigen::Vector3d translation_oldPlaneToOptimizedPlane_m =
        rotationTranslation_m + normalTranslation_m;

    centroid =
        rotationMatrix * centroid + translation_oldPlaneToOptimizedPlane_m;

    for (pcl::PointXYZRGBA &point : planeCloud->points)
    {
        Eigen::Vector3d point_NewWorld_m(point.x, point.y, point.z);
        point_NewWorld_m = rotationMatrix * point_NewWorld_m +
                           translation_oldPlaneToOptimizedPlane_m;

        point.x = point_NewWorld_m.x();
        point.y = point_NewWorld_m.y();
        point.z = point_NewWorld_m.z();
    }

    globalEquation = g2o::Plane3D(targetCoefficients);

    octree->deleteTree();
    octree->setInputCloud(planeCloud);
    octree->addPointsFromInputCloud();
    updatePlaneBoundsWithoutLock();
    ++cloudGeneration;
    successfulRefitGeneration = cloudGeneration;
}

g2o::Plane3D Plane::transformPlaneEquation(
    const g2o::Plane3D &plane_in,
    const g2o::Sim3    &transform_oldWorldToNewWorld_in)
{
    /*!
     * Transform a plane equation:
     *
     *     n^T x + d = 0
     *
     * under a Sim3 transformation:
     *
     *     x' = s R x + t
     *
     * The transformed plane is:
     *
     *     n'^T x' + d' = 0
     *
     */

    const Eigen::Matrix3d rotation = transform_oldWorldToNewWorld_in.rotation()
                                         .toRotationMatrix()
                                         .cast<double>();

    const Eigen::Vector3d translation =
        transform_oldWorldToNewWorld_in.translation().cast<double>();

    const double scale = transform_oldWorldToNewWorld_in.scale();

    /* Original plane parameters */
    const Eigen::Vector3d normal = plane_in.normal().cast<double>();

    /* g2o::Plane3D::distance() returns -d; use the equation coefficient. */
    const double equationOffset = plane_in.coeffs()(3);

    /*!
     * Transform the normal. For a similarity transform, the scale does not
     * affect the direction of the normal.
     */
    const Eigen::Vector3d transformedNormal = rotation * normal;

    /*!
     * Transform the plane offset.
     *
     * x' = sRx + t
     *
     * therefore:
     *
     * d' = s*d - n'^T*t
     */
    const double transformedEquationOffset =
        scale * equationOffset - transformedNormal.dot(translation);

    g2o::Plane3D transformedPlane(Eigen::Vector4d(transformedNormal.x(),
                                                  transformedNormal.y(),
                                                  transformedNormal.z(),
                                                  transformedEquationOffset));

    return transformedPlane;
}

void Plane::setMapClouds(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloud_in)
{
    if (!p_planeCloud_in || p_planeCloud_in->empty())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mMutexFeatures);

    /* Add the new points to the plane cloud */
    for (const auto &point : p_planeCloud_in->points)
    {
        planeCloud->points.push_back(point);
    }

    /* Direct writes to the points container do not update PCL's organization
     * metadata. Mapped planes accumulate observations over time, so retaining
     * the first observation's width eventually makes every later copy or
     * transform report an invalid width/size combination. */
    planeCloud->width  = planeCloud->size();
    planeCloud->height = 1;

    /* Update the octree */
    octree->deleteTree();
    octree->setInputCloud(planeCloud);
    octree->addPointsFromInputCloud();
    ++cloudGeneration;
}

void Plane::replaceMapClouds(
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_planeCloud_in)
{
    std::scoped_lock lock(mMutexPos, mMutexType, mMutexFeatures);
    planeCloud->clear();
    lastSuccessfulRefitFinitePointCount = 0;
    ++cloudGeneration;

    if (!p_planeCloud_in || p_planeCloud_in->empty())
    {
        centroid.setZero();
        mbBad = true;
        octree->deleteTree();
        return;
    }

    planeCloud->assign(p_planeCloud_in->begin(), p_planeCloud_in->end());
    planeCloud->header              = p_planeCloud_in->header;
    planeCloud->is_dense            = p_planeCloud_in->is_dense;
    planeCloud->sensor_origin_      = p_planeCloud_in->sensor_origin_;
    planeCloud->sensor_orientation_ = p_planeCloud_in->sensor_orientation_;

    /* Update the octree */
    octree->deleteTree();
    octree->setInputCloud(planeCloud);
    octree->addPointsFromInputCloud();
}

std::optional<Plane::GeometrySnapshot> Plane::beginMapCloudRefit(void)
{
    std::scoped_lock lock(mMutexPos, mMutexFeatures);

    if (planeCloud == nullptr || planeCloud->empty() ||
        cloudGeneration <= lastRefitAttemptGeneration)
    {
        return std::nullopt;
    }

    lastRefitAttemptGeneration = cloudGeneration;
    GeometrySnapshot snapshot;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudCopy(
        new pcl::PointCloud<pcl::PointXYZRGBA>(*planeCloud));
    snapshot.supportCloud              = cloudCopy;
    snapshot.equation_World            = globalEquation.coeffs();
    snapshot.centroid_World_m          = centroid;
    snapshot.minPlaneU_m               = minPlaneU;
    snapshot.maxPlaneU_m               = maxPlaneU;
    snapshot.minPlaneV_m               = minPlaneV;
    snapshot.maxPlaneV_m               = maxPlaneV;
    snapshot.finiteSupportCount        = lastSuccessfulRefitFinitePointCount;
    snapshot.observationCount          = observationCount;
    snapshot.cloudGeneration           = cloudGeneration;
    snapshot.successfulRefitGeneration = successfulRefitGeneration;
    return snapshot;
}

bool Plane::completeMapCloudRefit(
    const std::uint64_t sourceCloudGeneration_in,
    const Eigen::Vector3d &centroid_World_m_in,
    const g2o::Plane3D    &equation_World_in,
    const std::size_t      finitePointCount_in)
{
    std::scoped_lock lock(mMutexPos, mMutexType, mMutexFeatures);

    if (sourceCloudGeneration_in != cloudGeneration)
    {
        return false;
    }

    centroid                              = centroid_World_m_in;
    globalEquation                        = equation_World_in;
    lastSuccessfulRefitFinitePointCount = finitePointCount_in;
    successfulRefitGeneration             = sourceCloudGeneration_in;

    updatePlaneBoundsWithoutLock();
    return true;
}

bool Plane::isPointinPlaneCloud(const Eigen::Vector3d &point)
{
    unique_lock<mutex> lock(mMutexFeatures);
    pcl::PointXYZRGBA  pointPCL;
    pointPCL.x = point(0);
    pointPCL.y = point(1);
    pointPCL.z = point(2);

    SystemParams      *sysParams = SystemParams::GetParams();
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (octree->radiusSearch(
            pointPCL,
            sysParams->refine_map_points.octree.search_radius,
            pointIdxRadiusSearch,
            pointRadiusSquaredDistance,
            sysParams->refine_map_points.octree.min_neighbors) ==
        sysParams->refine_map_points.octree.min_neighbors)
    {
        return true;
    }

    return false;
}

Plane::planeVariant Plane::getPlaneType(void)
{
    unique_lock<mutex> lock(mMutexType);
    return planeType;
}

Plane::planeVariant Plane::getExpectedPlaneType(void)
{
    unique_lock<mutex> lock(mMutexType);

    // get the maximum vote
    double       maxVotes = 0;
    planeVariant maxType  = planeVariant::UNDEFINED;
    for (const auto &vote : semanticVotes)
    {
        if (vote.second > maxVotes)
        {
            maxVotes = vote.second;
            maxType  = vote.first;
        }
    }
    return maxType;
}

void Plane::castWeightedVote(Plane::planeVariant semanticType,
                             double              voteWeight)
{
    unique_lock<mutex> lock(mMutexType);

    if (semanticType == planeVariant::UNDEFINED)
        return;

    // check if semantic type is already in the semanticVotes map
    if (semanticVotes.find(semanticType) == semanticVotes.end())
        semanticVotes[semanticType] = voteWeight;
    else
        semanticVotes[semanticType] += voteWeight;

    // update based on new vote rankings
    // find the semantic type with the maximum votes
    double       maxVotes = 0;
    planeVariant maxType  = planeVariant::UNDEFINED;
    for (const auto &vote : semanticVotes)
    {
        if (vote.second > maxVotes)
        {
            maxVotes = vote.second;
            maxType  = vote.first;
        }
    }

    // set the plane type if votes above a certain threshold
    if (maxVotes >= SystemParams::GetParams()->sem_seg.min_votes)
        planeType = maxType;
    else
        planeType = planeVariant::UNDEFINED;
}

void Plane::setPlaneType(planeVariant newType)
{
    unique_lock<mutex> lock(mMutexType);
    planeType = newType;
}

void Plane::resetPlaneSemantics(void)
{
    unique_lock<mutex> lock(mMutexType);

    semanticVotes.clear();
    planeType = planeVariant::UNDEFINED;
}

g2o::Plane3D Plane::getLocalEquation(void) const
{
    unique_lock<mutex> lock(mMutexPos);
    return localEquation;
}

void Plane::setLocalEquation(const g2o::Plane3D &value)
{
    unique_lock<mutex> lock(mMutexPos);
    localEquation = value;
}

g2o::Plane3D Plane::getGlobalEquation(void) const
{
    unique_lock<mutex> lock(mMutexPos);
    return globalEquation;
}

void Plane::setGlobalEquation(const g2o::Plane3D &value)
{
    unique_lock<mutex> lock(mMutexPos);
    globalEquation = value;
}

Eigen::Vector3d Plane::getCentroid(void) const
{
    unique_lock<mutex> lock(mMutexPos);
    return centroid;
}

void Plane::updateSizeOfPlane(void)
{
    std::scoped_lock lock(mMutexPos, mMutexType, mMutexFeatures);
    updatePlaneBoundsWithoutLock();
}

void Plane::updatePlaneBoundsWithoutLock(void)
{

    /* Reset the plane size */
    minPlaneU = std::numeric_limits<double>::max();
    maxPlaneU = std::numeric_limits<double>::lowest();
    minPlaneV = std::numeric_limits<double>::max();
    maxPlaneV = std::numeric_limits<double>::lowest();

    /* Extract the coefficient of the wall */
    Eigen::Vector4d wallEquation = globalEquation.coeffs();

    /* Extract the magnitude of the norm from the coefficients */
    const double normalMagnitude = wallEquation.head<3>().norm();

    if (!std::isfinite(normalMagnitude) || normalMagnitude < 1e-8)
    {
        std::cerr << "[GeoSemHelper] Cannot create open passage: wall " << id
                  << " has an invalid plane equation." << std::endl;
        return;
    }

    /* Normalize the norm vector */
    Eigen::Vector3d normalVector = wallEquation.head<3>() / normalMagnitude;

    for (Eigen::Index component = 0; component < normalVector.size(); ++component)
    {
        if (std::abs(normalVector(component)) <= 1e-12)
        {
            continue;
        }
        if (normalVector(component) < 0.0)
        {
            normalVector = -normalVector;
        }
        break;
    }

    /* Ensure the plane normal is valid */
    if (!normalVector.allFinite() ||
        normalVector.squaredNorm() < std::numeric_limits<double>::epsilon())
    {
        return;
    }

    /* Normalize the plane normal in the world frame */
    const Eigen::Vector3d planeNormal = normalVector.normalized();

    /* Match finiteWallExtentsAreCompatible(): deterministic X/Y reference. */
    const Eigen::Vector3d referenceAxis = std::abs(planeNormal.x()) <= 0.90
                                               ? Eigen::Vector3d::UnitX()
                                               : Eigen::Vector3d::UnitY();

    /* Construct orthonormal axes lying inside the plane */
    const Eigen::Vector3d axisU = planeNormal.cross(referenceAxis).normalized();
    const Eigen::Vector3d axisV = planeNormal.cross(axisU).normalized();

    /* Init flag to indicate that a valid point was found */
    bool foundValidPoint = false;

    /* Skip if cloud is invalid */
    if (!planeCloud || planeCloud->empty())
    {
        return;
    }

    /* Iterate through associated map points */
    for (const pcl::PointXYZRGBA &point : planeCloud->points)
    {
        /* Skip points with invalid coordinates */
        if (!pcl::isFinite(point))
        {
            continue;
        }

        /* Convert PCL to Eigen */
        const Eigen::Vector3d point_World(static_cast<double>(point.x),
                                          static_cast<double>(point.y),
                                          static_cast<double>(point.z));

        /* Emit actual canonical world projections, not centroid-relative extents. */
        const double pointU_Plane = point_World.dot(axisU);
        const double pointV_Plane = point_World.dot(axisV);

        /* Update finite plane bounds */
        minPlaneU = std::min(minPlaneU, pointU_Plane);
        maxPlaneU = std::max(maxPlaneU, pointU_Plane);
        minPlaneV = std::min(minPlaneV, pointV_Plane);
        maxPlaneV = std::max(maxPlaneV, pointV_Plane);

        /* Set flag to indicate that a valid point is linked with the plane */
        foundValidPoint = true;
    }

    /* Reset to zero when no valid supporting points exist */
    if (!foundValidPoint)
    {
        mbBad = true;
    }
}

void Plane::setCentroid(const Eigen::Vector3d &value)
{
    unique_lock<mutex> lock(mMutexPos);
    centroid = value;
}

std::map<KeyFrame *, Plane::Observation> Plane::getObservations(void) const
{
    unique_lock<mutex> lock(mMutexFeatures);
    return observations;
}

std::size_t Plane::getObservationCount(void) const
{
    unique_lock<mutex> lock(mMutexFeatures);
    return observationCount;
}

void Plane::addObservation(KeyFrame          *p_keyFrame_in,
                           const Observation &observation_in)
{
    /* Confirm the keyframe is valid */
    if (p_keyFrame_in == nullptr || p_keyFrame_in->isBad())
    {
        return;
    }

    /* Lock the plane observation data */
    unique_lock<mutex> lock(mMutexFeatures);

    /*!
     * Insert the observation only when the keyframe has not previously
     * observed this plane.
     */
    const auto insertionResult =
        observations.insert({p_keyFrame_in, observation_in});

    /* Increment the observation count after a successful insertion */
    if (insertionResult.second)
    {
        observationCount++;

        if (refKeyFrame == nullptr)
        {
            refKeyFrame = p_keyFrame_in;
        }
    }
}

void Plane::mergeObservation(KeyFrame          *p_keyFrame_in,
                             const Observation &observation_in)
{
    if (p_keyFrame_in == nullptr || p_keyFrame_in->isBad())
    {
        return;
    }

    std::scoped_lock lock(mMutexFeatures, mMutexType);
    auto evidenceFromObservation = [](const Observation &observation)
    {
        std::map<planeVariant, double> evidence =
            observation.semanticEvidence;
        if (evidence.empty() &&
            observation.semanticType != planeVariant::UNDEFINED &&
            std::isfinite(observation.confidence))
        {
            evidence[observation.semanticType] += observation.confidence;
        }
        return evidence;
    };

    const auto existingIterator = observations.find(p_keyFrame_in);
    if (existingIterator == observations.end())
    {
        Observation mergedObservation = observation_in;
        mergedObservation.semanticEvidence =
            evidenceFromObservation(observation_in);
        observations.emplace(p_keyFrame_in, std::move(mergedObservation));
        ++observationCount;
        if (refKeyFrame == nullptr)
        {
            refKeyFrame = p_keyFrame_in;
        }
    }
    else
    {
        Observation &retainedObservation = existingIterator->second;
        const double retainedConfidence = retainedObservation.confidence;
        retainedObservation.pointPlaneConstraintMatrix +=
            observation_in.pointPlaneConstraintMatrix;

        std::map<planeVariant, double> combinedEvidence =
            evidenceFromObservation(retainedObservation);
        for (const auto &[semanticType, weight] :
             evidenceFromObservation(observation_in))
        {
            combinedEvidence[semanticType] += weight;
        }
        retainedObservation.semanticEvidence = std::move(combinedEvidence);
        retainedObservation.confidence += observation_in.confidence;

        if (observation_in.confidence > retainedConfidence)
        {
            retainedObservation.localPlane = observation_in.localPlane;
            retainedObservation.semanticType = observation_in.semanticType;
        }
    }

    rebuildSemanticVotesWithoutLock();
}

void Plane::rebuildSemanticVotesWithoutLock(void)
{
    semanticVotes.clear();
    for (const auto &[p_keyFrame, observation] : observations)
    {
        static_cast<void>(p_keyFrame);
        if (!observation.semanticEvidence.empty())
        {
            for (const auto &[semanticType, weight] :
                 observation.semanticEvidence)
            {
                if (semanticType != planeVariant::UNDEFINED &&
                    std::isfinite(weight))
                {
                    semanticVotes[semanticType] += weight;
                }
            }
        }
        else if (observation.semanticType != planeVariant::UNDEFINED &&
                 std::isfinite(observation.confidence))
        {
            semanticVotes[observation.semanticType] += observation.confidence;
        }
    }

    double maxVotes = 0.0;
    planeVariant maxType = planeVariant::UNDEFINED;
    for (const auto &[semanticType, votes] : semanticVotes)
    {
        if (votes > maxVotes)
        {
            maxVotes = votes;
            maxType = semanticType;
        }
    }
    planeType = maxVotes >= SystemParams::GetParams()->sem_seg.min_votes
                    ? maxType
                    : planeVariant::UNDEFINED;
}

void Plane::eraseObservation(KeyFrame *p_keyFrame_in)
{
    /* Confirm the keyframe is valid */
    if (p_keyFrame_in == nullptr)
    {
        return;
    }

    {
        /* Lock observations and semantics for one consistent vote rebuild. */
        std::scoped_lock lock(mMutexFeatures, mMutexType);

        const auto observationIterator = observations.find(p_keyFrame_in);

        /* Return when the keyframe has no observation */
        if (observationIterator == observations.end())
        {
            return;
        }

        /* Remove the observation */
        observations.erase(observationIterator);

        /* Decrement the observation count safely */
        if (observationCount > 0)
        {
            observationCount--;
        }

        if (refKeyFrame == p_keyFrame_in)
        {
            refKeyFrame = nullptr;

            for (const auto &[p_candidateKeyFrame, candidateObservation] :
                 observations)
            {
                (void)candidateObservation;

                if (p_candidateKeyFrame == nullptr)
                {
                    continue;
                }

                if (refKeyFrame == nullptr ||
                    p_candidateKeyFrame->mnId < refKeyFrame->mnId)
                {
                    refKeyFrame = p_candidateKeyFrame;
                }
            }
        }

        rebuildSemanticVotesWithoutLock();
    }
}

Map *Plane::GetMap(void)
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void Plane::SetMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}
} // namespace ORB_SLAM3
