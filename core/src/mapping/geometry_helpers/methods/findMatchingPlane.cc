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
 * @file            findMatchingPlane.cc
 *
 * @brief           Implements Utils::findMatchingPlane(), declared in
 *                  mapping/geometry_helpers/objects/UtilsClass.h.
 */

#include "mapping/geometry_helpers/private_functions/private_functions.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <variant>

#include <pcl/common/point_tests.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus Utils::findMatchingPlane(
    const std::vector<geometric::Plane *> &mappedPlanes_in,
    const PlaneEvidence                   &observedPlane_in,
    const PlaneMatchSettings              &matchSettings_in,
    int                                   &matchedPlaneId_out,
    std::vector<PlaneMatchVerdict>        *p_candidateVerdicts_out)
{
    if (p_candidateVerdicts_out != nullptr)
    {
        p_candidateVerdicts_out->clear();
    }

    if (matchSettings_in.matchMode == PlaneMatchMode::OPPOSITE_FACE)
    {
        /* Most observations wins; the earliest candidate wins a tie. */
        matchedPlaneId_out = -1;

        std::size_t bestObservationCount = 0U;

        for (geometric::Plane *p_mappedPlane : mappedPlanes_in)
        {
            PlaneMatchVerdict candidateVerdict{};
            if (checkOppositeFacePair(observedPlane_in,
                                      p_mappedPlane,
                                      matchSettings_in,
                                      candidateVerdict) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: checkOppositeFacePair returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            if (p_candidateVerdicts_out != nullptr)
            {
                p_candidateVerdicts_out->push_back(candidateVerdict);
            }
            if (candidateVerdict.decidingCheck != TwinFaceCheck::PLAUSIBLE)
            {
                continue;
            }

            std::size_t observationCount{};
            if (p_mappedPlane->getObservationCount(observationCount) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getObservationCount returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
            if (matchedPlaneId_out < 0 ||
                observationCount > bestObservationCount)
            {
                int mappedPlaneId{};
                if (p_mappedPlane->getId(mappedPlaneId) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status "
                                 "although it cannot fail; continuing as "
                                 "before.",
                                 __func__);
                }
                matchedPlaneId_out   = mappedPlaneId;
                bestObservationCount = observationCount;
            }
        }
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* SAME_FACE from here on. */
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_observedCloud =
        observedPlane_in.p_supportCloud_world_m;
    const geometric::Plane::PlaneVariant observedLabel =
        observedPlane_in.planeLabel;

    /* Return no association when no mapped planes are available */
    if (mappedPlanes_in.empty())
    {
        matchedPlaneId_out = -1;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* Confirm the observed plane point cloud is valid */
    if (p_observedCloud == nullptr || p_observedCloud->empty())
    {
        matchedPlaneId_out = -1;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* Extract the system parameters */
    types::SystemParams *p_sysParams = nullptr;
    if (types::SystemParams::getParams(p_sysParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * Normalize the observed equation by its normal length only, keeping its
     * sign: an observing-camera snapshot is taken against this orientation.
     */
    Eigen::Vector4d givenEquation   = observedPlane_in.planeEquation_world;
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

    for (const pcl::PointXYZRGBA &point : p_observedCloud->points)
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

    /* Association thresholds, each raised to a floor. */
    const double maximumAngularDifference =
        std::max(0.01, matchSettings_in.maximumAngle_rad);

    const double maximumPlaneDistance =
        std::max(0.01, matchSettings_in.maximumPlaneDistance_m);

    const double maximumCentroidDistance =
        std::max(0.10, matchSettings_in.maximumCentroidDistance_m);

    const bool useWallExtension =
        observedLabel == geometric::Plane::PlaneVariant::WALL &&
        p_sysParams->semSeg.reassociate.wallExtension.enabled;

    const double maximumFiniteCloudDistance = std::max(
        0.05,
        useWallExtension
            ? static_cast<double>(p_sysParams->semSeg.reassociate.wallExtension
                                      .maximumInPlaneGap_m)
            : matchSettings_in.maximumCloudGap_m);

    /*!
     * Observer side of a wall: one camera centre, compared per mapped
     * equation below, or the observing cameras of a mapped plane.
     */
    const Eigen::Vector3d *p_cameraCentre_world_m =
        std::get_if<Eigen::Vector3d>(&observedPlane_in.observerSide);
    const geometric::Plane::ObservationSideSnapshot *p_observingCameras =
        std::get_if<geometric::Plane::ObservationSideSnapshot>(
            &observedPlane_in.observerSide);

    const bool hasCameraCentre = p_cameraCentre_world_m != nullptr &&
                                 p_cameraCentre_world_m->allFinite();

    const bool checkObserverSide =
        observedLabel == geometric::Plane::PlaneVariant::WALL &&
        (hasCameraCentre || p_observingCameras != nullptr);

    /*!
     * Minimum fraction of sampled observation points which must be close to
     * the mapped finite plane cloud when the centroids are far apart.
     */
    constexpr double minimumFiniteOverlapRatio = 0.10;

    /*!
     * Limit the number of nearest-neighbour searches for each candidate.
     */
    constexpr std::size_t maximumSampleCount = 300;

    /*!
     * A camera centre closer to the plane than this does not identify a
     * side, in metres.
     */
    constexpr double minimumReliableSideDistance_m = 0.10;

    /* Track the best mapped-plane candidate */
    int bestPlaneId = -1;

    double bestAssociationScore = std::numeric_limits<double>::max();

    /* Iterate through every mapped plane */
    for (geometric::Plane *p_mappedPlane : mappedPlanes_in)
    {
        /* Skip invalid mapped planes */
        bool mappedPlaneIsBad{};
        if (!(p_mappedPlane == nullptr) &&
            p_mappedPlane->isBad(mappedPlaneIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mappedPlane == nullptr || mappedPlaneIsBad)
        {
            continue;
        }

        /* Extract the mapped plane point cloud */
        geometric::Plane::GeometrySnapshot mappedGeometry{};
        if (p_mappedPlane->getGeometrySnapshot(mappedGeometry) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGeometrySnapshot returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
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
        geometric::Plane::PlaneVariant mappedExpectedType{};
        if (p_mappedPlane->getExpectedPlaneType(mappedExpectedType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getExpectedPlaneType returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        bool semanticTypesCompatible =
            observedLabel == geometric::Plane::PlaneVariant::UNDEFINED ||
            mappedExpectedType == geometric::Plane::PlaneVariant::UNDEFINED ||
            mappedExpectedType == observedLabel;

        if (matchSettings_in.typeRule == PlaneTypeRule::CONFIRMED_TYPE_STRICT)
        {
            geometric::Plane::PlaneVariant mappedConfirmedType{};
            if (p_mappedPlane->getPlaneType(mappedConfirmedType) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPlaneType returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            semanticTypesCompatible =
                semanticTypesCompatible && mappedConfirmedType == observedLabel;
        }

        if (!semanticTypesCompatible)
        {
            continue;
        }

        /* Normalize the mapped equation; both are in the world frame. */
        Eigen::Vector4d mappedEquation = mappedGeometry.planeEquation_world;

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

        /*!
         * Keep wall faces observed from opposite sides apart. Their equations
         * can be nearly identical when the wall is thinner than the distance
         * threshold, but they bound different rooms.
         */
        if (checkObserverSide)
        {
            ObservationSideEvidence mappedObservationSide{};
            if (getMedianObservationSide_world_m(p_mappedPlane,
                                                 mappedEquation,
                                                 mappedObservationSide) !=
                UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMedianObservationSide_world_m returned a "
                             "failure status although it cannot fail; "
                             "continuing as before.",
                             __func__);
            }

            std::optional<double> givenObservationSide_m{};
            bool                  givenSideIsAmbiguous = false;

            if (hasCameraCentre)
            {
                const double cameraSide_m =
                    mappedEquation.head<3>().dot(*p_cameraCentre_world_m) +
                    mappedEquation(3);

                if (std::abs(cameraSide_m) >= minimumReliableSideDistance_m)
                {
                    givenObservationSide_m = cameraSide_m;
                }
            }
            else
            {
                givenObservationSide_m =
                    p_observingCameras->medianSignedDistance_m;
                givenSideIsAmbiguous =
                    p_observingCameras->face ==
                    geometric::Plane::ObservationSideSnapshot::Face::AMBIGUOUS;
            }

            if (givenSideIsAmbiguous || mappedObservationSide.isAmbiguous)
            {
                continue;
            }

            if (givenObservationSide_m.has_value() &&
                mappedObservationSide.medianSignedDistance_m.has_value() &&
                givenObservationSide_m.value() *
                        mappedObservationSide.medianSignedDistance_m.value() <
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
                p_observedCloud,
                mappedEquation.head<3>(),
                p_sysParams->semSeg.reassociate.wallExtension
                    .maximumInPlaneGap_m,
                p_sysParams->semSeg.reassociate.wallExtension
                    .minimumOrthogonalOverlap_m,
                areCompatible) != UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: finiteWallExtentsAreCompatible returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        const bool finiteWallExtentsCompatible =
            useWallExtension && areCompatible;

        /* Extract the global mapped-plane centroid */
        const Eigen::Vector3d mappedCentroid =
            mappedGeometry.planeCentroid_world_m;

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

        const std::size_t samplingStride =
            std::max<std::size_t>(1,
                                  p_observedCloud->size() / maximumSampleCount);

        std::size_t sampledPointCount     = 0;
        std::size_t overlappingPointCount = 0;

        double minimumCloudDistance = std::numeric_limits<double>::max();

        std::vector<int> nearestPointIndex(1);

        std::vector<float> nearestSquaredDistance(1);

        for (std::size_t pointIndex = 0; pointIndex < p_observedCloud->size();
             pointIndex += samplingStride)
        {
            const pcl::PointXYZRGBA &queryPoint =
                p_observedCloud->points[pointIndex];

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
         * @note            The angular and perpendicular plane-distance checks
         *                  have
         *                               already been applied above. Therefore,
         *                               planes with the
         *                  same centroid but significantly different normals
         *                  are not merged.
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

            int mappedPlaneGetId{};
            if (p_mappedPlane->getId(mappedPlaneGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            bestPlaneId = mappedPlaneGetId;
        }
    }

    matchedPlaneId_out = bestPlaneId;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
