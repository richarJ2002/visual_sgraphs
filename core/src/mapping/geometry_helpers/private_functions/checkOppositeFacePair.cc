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
 * @file            checkOppositeFacePair.cc
 *
 * @brief           Implements checkOppositeFacePair(), declared in
 *                  private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <variant>

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

UtilsStatus checkOppositeFacePair(const PlaneEvidence      &observedPlane_in,
                                  geometric::Plane         *p_mappedPlane_in,
                                  const PlaneMatchSettings &matchSettings_in,
                                  PlaneMatchVerdict &candidateVerdict_out)
{
    candidateVerdict_out = PlaneMatchVerdict{};

    bool mappedPlaneIsBad{};
    if (p_mappedPlane_in != nullptr &&
        p_mappedPlane_in->isBad(mappedPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_mappedPlane_in == nullptr || mappedPlaneIsBad)
    {
        candidateVerdict_out.decidingCheck = TwinFaceCheck::INVALID_PLANE;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    geometric::Plane::GeometrySnapshot mappedGeometry{};
    if (p_mappedPlane_in->getGeometrySnapshot(mappedGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    Eigen::Vector4d observedEquation_world =
        observedPlane_in.planeEquation_world;
    Eigen::Vector4d mappedEquation_world = mappedGeometry.planeEquation_world;

    /* Parallel when the unsigned normal angle is below the threshold. */
    const double normalAlignment =
        std::clamp(std::abs(observedEquation_world.head<3>().normalized().dot(
                       mappedEquation_world.head<3>().normalized())),
                   -1.0,
                   1.0);
    if (!(std::acos(normalAlignment) < matchSettings_in.maximumAngle_rad))
    {
        candidateVerdict_out.decidingCheck = TwinFaceCheck::NOT_PARALLEL;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    const double observedNormalNorm = observedEquation_world.head<3>().norm();
    const double mappedNormalNorm   = mappedEquation_world.head<3>().norm();

    if (!observedEquation_world.allFinite() ||
        !mappedEquation_world.allFinite() || observedNormalNorm < 1e-8 ||
        mappedNormalNorm < 1e-8 ||
        !observedPlane_in.planeCentroid_world_m.has_value())
    {
        candidateVerdict_out.decidingCheck = TwinFaceCheck::INVALID_PLANE;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    observedEquation_world /= observedNormalNorm;
    mappedEquation_world /= mappedNormalNorm;

    /* Wall thickness is measured at the faces themselves: the mean distance
     * of each face's centroid from the other face's plane. Comparing the
     * plane offsets instead measures the gap next to the map origin, which
     * for faces a few degrees apart differs from the gap at the wall by
     * the tilt times the wall's distance along the plane from that point --
     * comparable to the thickness of a thin wall. */
    const Eigen::Vector3d &observedCentroid_world_m =
        observedPlane_in.planeCentroid_world_m.value();
    const Eigen::Vector3d &mappedCentroid_world_m =
        mappedGeometry.planeCentroid_world_m;
    const double faceSeparation_m =
        0.5 *
        (std::abs(observedEquation_world.head<3>().dot(mappedCentroid_world_m) +
                  observedEquation_world(3)) +
         std::abs(mappedEquation_world.head<3>().dot(observedCentroid_world_m) +
                  mappedEquation_world(3)));

    candidateVerdict_out.faceSeparation_m = faceSeparation_m;

    if (faceSeparation_m > matchSettings_in.maximumThickness_m)
    {
        candidateVerdict_out.decidingCheck =
            TwinFaceCheck::THICKNESS_OUT_OF_RANGE;
        candidateVerdict_out.decidingValue = faceSeparation_m;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* A wall's two faces are observed from cameras standing on opposite
     * exterior sides, so each face's observation origin must resolve to
     * opposite sides of the OTHER face's equation. */
    const Eigen::Vector3d *p_observedOrigin_world_m =
        std::get_if<Eigen::Vector3d>(&observedPlane_in.observerSide);
    std::optional<Eigen::Vector3d> mappedOrigin_world_m{};
    if (p_mappedPlane_in->getObservationOrigin_world(mappedOrigin_world_m) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationOrigin_world returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (p_observedOrigin_world_m == nullptr ||
        !p_observedOrigin_world_m->allFinite() ||
        !mappedOrigin_world_m.has_value() || !mappedOrigin_world_m->allFinite())
    {
        /* Planes created before the stamp existed carry no face identity;
         * make no claim rather than a wrong one. */
        candidateVerdict_out.decidingCheck =
            TwinFaceCheck::NO_OBSERVATION_ORIGIN;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    constexpr double minimumResolvableSide_m = 0.10;
    const double     observedSideOfObservedOrigin_m =
        observedEquation_world.head<3>().dot(*p_observedOrigin_world_m) +
        observedEquation_world(3);
    const double observedSideOfMappedOrigin_m =
        observedEquation_world.head<3>().dot(mappedOrigin_world_m.value()) +
        observedEquation_world(3);
    const double mappedSideOfObservedOrigin_m =
        mappedEquation_world.head<3>().dot(*p_observedOrigin_world_m) +
        mappedEquation_world(3);
    const double mappedSideOfMappedOrigin_m =
        mappedEquation_world.head<3>().dot(mappedOrigin_world_m.value()) +
        mappedEquation_world(3);

    const double smallestOriginSide_m =
        std::min({std::abs(observedSideOfObservedOrigin_m),
                  std::abs(observedSideOfMappedOrigin_m),
                  std::abs(mappedSideOfObservedOrigin_m),
                  std::abs(mappedSideOfMappedOrigin_m)});
    if (!std::isfinite(observedSideOfObservedOrigin_m) ||
        !std::isfinite(observedSideOfMappedOrigin_m) ||
        !std::isfinite(mappedSideOfObservedOrigin_m) ||
        !std::isfinite(mappedSideOfMappedOrigin_m) ||
        smallestOriginSide_m < minimumResolvableSide_m)
    {
        candidateVerdict_out.decidingCheck =
            TwinFaceCheck::OBSERVATION_SIDE_UNRESOLVED;
        candidateVerdict_out.decidingValue = smallestOriginSide_m;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    const bool areOppositeAcrossObserved =
        observedSideOfObservedOrigin_m * observedSideOfMappedOrigin_m < 0.0;
    const bool areOppositeAcrossMapped =
        mappedSideOfObservedOrigin_m * mappedSideOfMappedOrigin_m < 0.0;

    if (!areOppositeAcrossObserved || !areOppositeAcrossMapped)
    {
        candidateVerdict_out.decidingCheck =
            TwinFaceCheck::OBSERVED_FROM_SAME_SIDE;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    /* In-plane footprint overlap, projected onto one shared ground-anchored
     * tangent frame so the two planes' own local U/V axes need not agree. */
    const Eigen::Vector3d observedNormal_world =
        observedEquation_world.head<3>();
    const double groundNormalNorm = matchSettings_in.groundNormal_world.norm();
    Eigen::Vector3d axisU_world   = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisV_world   = Eigen::Vector3d::Zero();
    if (std::isfinite(groundNormalNorm) && groundNormalNorm > 1e-8)
    {
        const Eigen::Vector3d horizontalCandidate_world =
            (matchSettings_in.groundNormal_world / groundNormalNorm)
                .cross(observedNormal_world);
        const double horizontalNorm = horizontalCandidate_world.norm();
        if (std::isfinite(horizontalNorm) && horizontalNorm > 1e-3)
        {
            axisU_world = horizontalCandidate_world / horizontalNorm;
            axisV_world = axisU_world.cross(observedNormal_world).normalized();
        }
    }
    if (axisU_world.squaredNorm() < 0.5 || axisV_world.squaredNorm() < 0.5)
    {
        axisU_world = observedNormal_world.unitOrthogonal().normalized();
        axisV_world = observedNormal_world.cross(axisU_world).normalized();
    }

    ProjectedPlaneBounds observedBounds{};
    if (projectPlaneBounds(observedPlane_in.p_supportCloud_world_m,
                           axisU_world,
                           axisV_world,
                           observedBounds) != UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: projectPlaneBounds returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    ProjectedPlaneBounds mappedBounds{};
    if (projectPlaneBounds(mappedGeometry.supportCloud,
                           axisU_world,
                           axisV_world,
                           mappedBounds) != UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: projectPlaneBounds returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    const double overlapU_m =
        std::min(observedBounds.maximumU_m, mappedBounds.maximumU_m) -
        std::max(observedBounds.minimumU_m, mappedBounds.minimumU_m);
    const double overlapV_m =
        std::min(observedBounds.maximumV_m, mappedBounds.maximumV_m) -
        std::max(observedBounds.minimumV_m, mappedBounds.minimumV_m);

    if (!observedBounds.isValid || !mappedBounds.isValid || overlapU_m <= 0.0 ||
        overlapV_m <= 0.0)
    {
        candidateVerdict_out.decidingCheck =
            TwinFaceCheck::NO_FOOTPRINT_OVERLAP;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    const double overlapArea_m2 = overlapU_m * overlapV_m;
    const double observedArea_m2 =
        (observedBounds.maximumU_m - observedBounds.minimumU_m) *
        (observedBounds.maximumV_m - observedBounds.minimumV_m);
    const double mappedArea_m2 =
        (mappedBounds.maximumU_m - mappedBounds.minimumU_m) *
        (mappedBounds.maximumV_m - mappedBounds.minimumV_m);
    const double smallerArea_m2 = std::min(observedArea_m2, mappedArea_m2);

    if (!std::isfinite(smallerArea_m2) || smallerArea_m2 < 1e-6)
    {
        candidateVerdict_out.decidingCheck =
            TwinFaceCheck::NO_FOOTPRINT_OVERLAP;
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    candidateVerdict_out.decidingValue = overlapArea_m2 / smallerArea_m2;
    candidateVerdict_out.decidingCheck =
        candidateVerdict_out.decidingValue >=
                matchSettings_in.minimumOverlapRatio
            ? TwinFaceCheck::PLAUSIBLE
            : TwinFaceCheck::LOW_OVERLAP;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} /* namespace utils */
} /* namespace utils */
} /* namespace core */
} /* namespace vs_graphs */
