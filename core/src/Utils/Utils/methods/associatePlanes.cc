/*!
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
 * @file            associatePlanes.cc
 *
 * @brief           Implements Utils::associatePlanes(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/private_functions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::associatePlanes(
    const vector<geometric::Plane *>            &mappedPlanes_in,
    g2o::Plane3D                                 observedPlane_in,
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_observedCloud_in,
    const Eigen::Matrix4d                       &keyframePose_in,
    const geometric::Plane::PlaneVariant         observedPlaneType_in,
    const float                                  threshold_in,
    int                                         &matchedPlaneId_out,
    const float                           maximumFiniteCloudDistance_m_in,
    const std::optional<Eigen::Vector3d> &observationOrigin_World_m_in)
{
    /* Return no association when no mapped planes are available */
    if (mappedPlanes_in.empty())
    {
        matchedPlaneId_out = -1;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* Confirm the observed plane point cloud is valid */
    if (p_observedCloud_in == nullptr || p_observedCloud_in->empty())
    {
        matchedPlaneId_out = -1;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* Extract the system parameters */
    types::SystemParams *p_sysParams = nullptr;
    if (types::SystemParams::getParams(p_sysParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }

    /* Extract and normalize the observed plane equation */
    Eigen::Vector4d givenEquation   = observedPlane_in.coeffs();
    const double    givenNormalNorm = givenEquation.head<3>().norm();

    /* Confirm norm is valid */
    if (!std::isfinite(givenNormalNorm) || givenNormalNorm < 1e-8)
    {
        matchedPlaneId_out = -1;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* Find the unit vector */
    givenEquation /= givenNormalNorm;

    /* Calculate the centroid of the observed global point cloud */
    Eigen::Vector3d givenCentroid = Eigen::Vector3d::Zero();

    std::size_t validGivenPointCount = 0;

    for (const pcl::PointXYZRGBA &point : p_observedCloud_in->points)
    {
        if (!pcl::isFinite(point))
        {
            continue;
        }

        givenCentroid += Eigen::Vector3d(static_cast<double>(point.x),
                                         static_cast<double>(point.y),
                                         static_cast<double>(point.z));

        validGivenPointCount++;
    }

    /* Return when the point cloud has no valid points */
    if (validGivenPointCount == 0)
    {
        matchedPlaneId_out = -1;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    givenCentroid /= static_cast<double>(validGivenPointCount);

    /*!
     * Association thresholds.
     *
     * @note        The ominus threshold_in is used here as the maximum angular
     *              difference in radians.
     */
    const double maximumAngularDifference =
        std::max(0.01, static_cast<double>(threshold_in));

    const double maximumPlaneDistance = std::max(
        0.01,
        static_cast<double>(p_sysParams->seg.planeAssociation.distanceThresh));

    const double maximumCentroidDistance = std::max(
        0.10,
        static_cast<double>(p_sysParams->seg.planeAssociation.centroidThresh));

    const bool useWallExtension =
        observedPlaneType_in == geometric::Plane::PlaneVariant::WALL &&
        p_sysParams->semSeg.reassociate.wallExtension.enabled;

    const double configuredFiniteCloudDistance_m =
        maximumFiniteCloudDistance_m_in > 0.0F
            ? static_cast<double>(maximumFiniteCloudDistance_m_in)
        : useWallExtension
            ? static_cast<double>(p_sysParams->semSeg.reassociate.wallExtension
                                      .maximumInPlaneGap_m)
            : static_cast<double>(p_sysParams->seg.planeAssociation
                                      .clusterSeparation.tolerance);

    const double maximumFiniteCloudDistance =
        std::max(0.05, configuredFiniteCloudDistance_m);

    /*!
     * Minimum fraction of sampled observation points which must be close to
     * the mapped finite plane cloud when the centroids are far apart.
     */
    constexpr double minimumFiniteOverlapRatio = 0.10;

    /*!
     * Limit the number of nearest-neighbour searches for each candidate.
     */
    constexpr std::size_t maximumSampleCount = 300;

    /* Track the best mapped-plane candidate */
    int bestPlaneId = -1;

    double bestAssociationScore = std::numeric_limits<double>::max();

    /* Iterate through every mapped plane */
    for (geometric::Plane *p_mappedPlane : mappedPlanes_in)
    {
        /* Skip invalid mapped planes */
        if (p_mappedPlane == nullptr || p_mappedPlane->isBad())
        {
            continue;
        }

        /* Extract the mapped plane point cloud */
        const geometric::Plane::GeometrySnapshot mappedGeometry =
            p_mappedPlane->getGeometrySnapshot();
        const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_mappedCloud =
            mappedGeometry.supportCloud;

        /* Skip mapped planes without finite geometry */
        if (p_mappedCloud == nullptr || p_mappedCloud->empty())
        {
            continue;
        }

        /*
         * Check semantic compatibility.
         *
         * A mapped UNDEFINED plane is allowed to match a semantically labelled
         * observation so that it can accumulate enough votes for confirmation.
         */
        const geometric::Plane::PlaneVariant mappedPlaneType =
            p_mappedPlane->getExpectedPlaneType();

        const bool semanticTypesCompatible =
            observedPlaneType_in == geometric::Plane::PlaneVariant::UNDEFINED ||
            mappedPlaneType == geometric::Plane::PlaneVariant::UNDEFINED ||
            mappedPlaneType == observedPlaneType_in;

        if (!semanticTypesCompatible)
        {
            continue;
        }

        /*!
         * Transform the mapped equation into the frame used by the supplied
         * observation.
         *
         * @note        SemanticSegmentation now supplies both planes in the
         *              global frame, therefore keyframePose_in is normally
         * identity.
         */
        g2o::Plane3D mappedPlaneInGivenFrame{};
        if (Utils::applyPoseToPlane(keyframePose_in,
                                    g2o::Plane3D(mappedGeometry.equation_World),
                                    mappedPlaneInGivenFrame) !=
            UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            // applyPoseToPlane cannot fail; continue as before.
        }

        Eigen::Vector4d mappedEquation = mappedPlaneInGivenFrame.coeffs();

        const double mappedNormalNorm = mappedEquation.head<3>().norm();

        if (!std::isfinite(mappedNormalNorm) || mappedNormalNorm < 1e-8)
        {
            continue;
        }

        mappedEquation /= mappedNormalNorm;

        /*!
         * Ensure both equations use the same normal direction before comparing
         * their distance coefficients.
         */
        if (givenEquation.head<3>().dot(mappedEquation.head<3>()) < 0.0)
        {
            mappedEquation *= -1.0;
        }

        /* Keep observations from opposite sides as distinct wall faces. */
        if (observedPlaneType_in == geometric::Plane::PlaneVariant::WALL &&
            observationOrigin_World_m_in.has_value() &&
            observationOrigin_World_m_in->allFinite())
        {
            ObservationSideEvidence mappedObservationSide{};
            if (getMedianObservationSide_World_m(p_mappedPlane,
                                                 mappedEquation,
                                                 mappedObservationSide) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                // getMedianObservationSide_World_m cannot fail; continue as
                // before.
            }

            const double givenObservationSide_m =
                mappedEquation.head<3>().dot(
                    observationOrigin_World_m_in.value()) +
                mappedEquation(3);

            constexpr double minimumReliableSideDistance_m = 0.10;

            if (mappedObservationSide.isAmbiguous)
            {
                continue;
            }

            if (mappedObservationSide.medianSignedDistance_m.has_value() &&
                std::abs(givenObservationSide_m) >=
                    minimumReliableSideDistance_m &&
                mappedObservationSide.medianSignedDistance_m.value() *
                        givenObservationSide_m <
                    0.0)
            {
                continue;
            }
        }

        /* Calculate the angular difference between the plane normals */
        const double normalAlignment =
            std::clamp(givenEquation.head<3>().dot(mappedEquation.head<3>()),
                       -1.0,
                       1.0);

        const double angularDifference = std::acos(normalAlignment);

        /* Reject planes whose normals are not sufficiently aligned */
        if (angularDifference > maximumAngularDifference)
        {
            continue;
        }

        /* Calculate perpendicular separation between the planes */
        const double planeDistance =
            std::abs(givenEquation(3) - mappedEquation(3));

        /* Reject parallel planes which are physically separated */
        if (planeDistance > maximumPlaneDistance)
        {
            continue;
        }

        bool areCompatible{};
        if ((useWallExtension) &&
            finiteWallExtentsAreCompatible(
                p_mappedCloud,
                p_observedCloud_in,
                mappedEquation.head<3>(),
                p_sysParams->semSeg.reassociate.wallExtension
                    .maximumInPlaneGap_m,
                p_sysParams->semSeg.reassociate.wallExtension
                    .minimumOrthogonalOverlap_m,
                areCompatible) != UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            // finiteWallExtentsAreCompatible cannot fail; continue as before.
        }
        const bool finiteWallExtentsCompatible =
            useWallExtension && areCompatible;

        /* Extract the global mapped-plane centroid */
        const Eigen::Vector3d mappedCentroid = mappedGeometry.centroid_World_m;

        /* Calculate the global centroid distance */
        const double centroidDistance = (givenCentroid - mappedCentroid).norm();

        /*!
         * Measure finite-cloud compatibility using nearest-neighbour distance.
         *
         * This prevents distant coplanar surfaces from being merged while
         * allowing neighbouring fragments of the same physical wall to join.
         */
        pcl::KdTreeFLANN<pcl::PointXYZRGBA> mappedCloudSearch;

        mappedCloudSearch.setInputCloud(p_mappedCloud);

        const std::size_t samplingStride = std::max<std::size_t>(
            1,
            p_observedCloud_in->size() / maximumSampleCount);

        std::size_t sampledPointCount     = 0;
        std::size_t overlappingPointCount = 0;

        double minimumCloudDistance = std::numeric_limits<double>::max();

        std::vector<int> nearestPointIndex(1);

        std::vector<float> nearestSquaredDistance(1);

        for (std::size_t pointIndex = 0;
             pointIndex < p_observedCloud_in->size();
             pointIndex += samplingStride)
        {
            const pcl::PointXYZRGBA &queryPoint =
                p_observedCloud_in->points[pointIndex];

            if (!pcl::isFinite(queryPoint))
            {
                continue;
            }

            sampledPointCount++;

            const int neighbourCount =
                mappedCloudSearch.nearestKSearch(queryPoint,
                                                 1,
                                                 nearestPointIndex,
                                                 nearestSquaredDistance);

            if (neighbourCount <= 0)
            {
                continue;
            }

            const double cloudDistance =
                std::sqrt(static_cast<double>(nearestSquaredDistance.front()));

            minimumCloudDistance =
                std::min(minimumCloudDistance, cloudDistance);

            if (cloudDistance <= maximumFiniteCloudDistance)
            {
                overlappingPointCount++;
            }
        }

        const double finiteOverlapRatio =
            sampledPointCount > 0 ? static_cast<double>(overlappingPointCount) /
                                        static_cast<double>(sampledPointCount)
                                  : 0.0;

        if (useWallExtension && !finiteWallExtentsCompatible &&
            finiteOverlapRatio < minimumFiniteOverlapRatio)
        {
            continue;
        }

        /*!
         * Planes with very close centroids and compatible normals are likely
         * duplicate estimates of the same physical surface.
         */
        const bool centroidsAreClose =
            centroidDistance <= maximumCentroidDistance;

        /*!
         * Plane fragments with separated centroids may still belong to the same
         * physical wall when their finite point clouds overlap or are adjacent.
         */
        const std::size_t minimumAdjacentPointCount = std::max<std::size_t>(
            3,
            static_cast<std::size_t>(
                std::ceil(0.02 * static_cast<double>(sampledPointCount))));

        const bool finiteCloudsCompatible =
            finiteWallExtentsCompatible ||
            finiteOverlapRatio >= minimumFiniteOverlapRatio ||
            (minimumCloudDistance <= maximumFiniteCloudDistance &&
             overlappingPointCount >= minimumAdjacentPointCount);

        /*!
         * Require either a direct centroid match or finite point-cloud
         * compatibility.
         *
         * @note        The angular and perpendicular plane-distance checks have
         *              already been applied above. Therefore, planes with the
         * same centroid but significantly different normals are not merged.
         */
        if (!centroidsAreClose && !finiteCloudsCompatible)
        {
            continue;
        }

        /* Normalize each component used by the association score */
        const double normalizedAngularDifference =
            angularDifference / maximumAngularDifference;

        const double normalizedPlaneDistance =
            planeDistance / maximumPlaneDistance;

        const double normalizedCentroidDistance =
            std::min(centroidDistance / maximumCentroidDistance, 2.0);

        const double normalizedCloudDistance =
            std::isfinite(minimumCloudDistance)
                ? std::min(minimumCloudDistance / maximumFiniteCloudDistance,
                           2.0)
                : 2.0;

        /*!
         * Calculate a combined score for the mapped-plane candidate.
         *
         * Lower scores represent stronger associations.
         */
        const double associationScore =
            3.0 * normalizedAngularDifference + 4.0 * normalizedPlaneDistance +
            0.25 * normalizedCentroidDistance + 0.50 * normalizedCloudDistance -
            2.0 * finiteOverlapRatio;

        /* Keep the strongest valid mapped-plane candidate */
        if (associationScore < bestAssociationScore)
        {
            bestAssociationScore = associationScore;

            bestPlaneId = p_mappedPlane->getId();
        }
    }

    matchedPlaneId_out = bestPlaneId;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
