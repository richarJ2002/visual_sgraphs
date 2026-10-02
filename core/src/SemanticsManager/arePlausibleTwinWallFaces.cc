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
 * @file            arePlausibleTwinWallFaces.cc
 *
 * @brief           Implements arePlausibleTwinWallFaces(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "SemanticsManager.h"

#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"
#include "private_functions.h"

#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    arePlausibleTwinWallFaces(geometric::Plane      *p_first_in,
                              geometric::Plane      *p_second_in,
                              double                 minimumThickness_m_in,
                              double                 maximumThickness_m_in,
                              double                 minimumOverlapRatio_in,
                              const Eigen::Vector3d &groundNormal_world_in,
                              bool &arePlausibleTwinWallFaces_out)
{
    bool firstIsBad{};
    if (!(p_first_in == nullptr) &&
        p_first_in->isBad(firstIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool secondIsBad{};
    if (!(p_first_in == nullptr || firstIsBad || p_second_in == nullptr) &&
        p_second_in->isBad(secondIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_first_in == nullptr || firstIsBad || p_second_in == nullptr ||
        secondIsBad || p_first_in == p_second_in)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    bool arePlanesParallel2{};
    if (utils::utils::Utils::arePlanesParallel(p_first_in,
                                               p_second_in,
                                               arePlanesParallel2) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: arePlanesParallel returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!arePlanesParallel2)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    g2o::Plane3D firstGetGlobalEquation{};
    if (p_first_in->getGlobalEquation(firstGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d equation1 = firstGetGlobalEquation.coeffs();
    g2o::Plane3D    secondGetGlobalEquation{};
    if (p_second_in->getGlobalEquation(secondGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d equation2   = secondGetGlobalEquation.coeffs();
    const double    normalNorm1 = equation1.head<3>().norm();
    const double    normalNorm2 = equation2.head<3>().norm();

    if (!equation1.allFinite() || !equation2.allFinite() ||
        normalNorm1 < 1e-8 || normalNorm2 < 1e-8)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    equation1 /= normalNorm1;
    equation2 /= normalNorm2;

    /* Align equation2's sign to equation1's before comparing offsets --
     * plane-equation sign is arbitrary. */
    Eigen::Vector4d alignedEquation2 = equation2;
    if (equation1.head<3>().dot(equation2.head<3>()) < 0.0)
    {
        alignedEquation2 *= -1.0;
    }
    const double separation_m = std::abs(equation1(3) - alignedEquation2(3));

    if (separation_m < minimumThickness_m_in ||
        separation_m > maximumThickness_m_in)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Opposite-exterior-side check, generalising isWallFaceForeignToRoom's
     * side-sign math from plane-to-room to plane-to-plane: a wall's two
     * faces are observed from cameras standing on opposite exterior sides,
     * so each face's observation origin must resolve to opposite sides of
     * the OTHER face's equation. */
    std::optional<Eigen::Vector3d> origin1{};
    if (p_first_in->getObservationOrigin_world(origin1) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationOrigin_world returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::optional<Eigen::Vector3d> origin2{};
    if (p_second_in->getObservationOrigin_world(origin2) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationOrigin_world returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (!origin1.has_value() || !origin1->allFinite() || !origin2.has_value() ||
        !origin2->allFinite())
    {
        /* Planes created before the stamp existed carry no face identity;
         * make no claim rather than a wrong one. */
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    constexpr double minimumResolvableSide_m = 0.10;
    const double     side1AtOrigin1 =
        equation1.head<3>().dot(origin1.value()) + equation1(3);
    const double side1AtOrigin2 =
        equation1.head<3>().dot(origin2.value()) + equation1(3);
    const double side2AtOrigin1 =
        equation2.head<3>().dot(origin1.value()) + equation2(3);
    const double side2AtOrigin2 =
        equation2.head<3>().dot(origin2.value()) + equation2(3);

    if (!std::isfinite(side1AtOrigin1) || !std::isfinite(side1AtOrigin2) ||
        !std::isfinite(side2AtOrigin1) || !std::isfinite(side2AtOrigin2) ||
        std::abs(side1AtOrigin1) < minimumResolvableSide_m ||
        std::abs(side1AtOrigin2) < minimumResolvableSide_m ||
        std::abs(side2AtOrigin1) < minimumResolvableSide_m ||
        std::abs(side2AtOrigin2) < minimumResolvableSide_m)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const bool oppositeAcrossPlane1 = (side1AtOrigin1 * side1AtOrigin2) < 0.0;
    const bool oppositeAcrossPlane2 = (side2AtOrigin1 * side2AtOrigin2) < 0.0;

    if (!oppositeAcrossPlane1 || !oppositeAcrossPlane2)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* In-plane footprint overlap, projected onto one shared ground-anchored
     * tangent frame so the two planes' (possibly differently canonicalised)
     * own local U/V axes don't have to agree. */
    const double    groundNormalNorm = groundNormal_world_in.norm();
    Eigen::Vector3d axisU_world      = Eigen::Vector3d::Zero();
    Eigen::Vector3d axisV_world      = Eigen::Vector3d::Zero();
    if (std::isfinite(groundNormalNorm) && groundNormalNorm > 1e-8)
    {
        const Eigen::Vector3d unitGroundNormal_world =
            groundNormal_world_in / groundNormalNorm;
        const Eigen::Vector3d horizontalCandidate_world =
            unitGroundNormal_world.cross(equation1.head<3>());
        const double horizontalNorm = horizontalCandidate_world.norm();
        if (std::isfinite(horizontalNorm) && horizontalNorm > 1e-3)
        {
            axisU_world = horizontalCandidate_world / horizontalNorm;
            axisV_world = axisU_world.cross(equation1.head<3>()).normalized();
        }
    }
    if (axisU_world.squaredNorm() < 0.5 || axisV_world.squaredNorm() < 0.5)
    {
        axisU_world = equation1.head<3>().unitOrthogonal().normalized();
        axisV_world = equation1.head<3>().cross(axisU_world).normalized();
    }

    double minimumU1 = 0.0, maximumU1 = 0.0, minimumV1 = 0.0, maximumV1 = 0.0;
    double minimumU2 = 0.0, maximumU2 = 0.0, minimumV2 = 0.0, maximumV2 = 0.0;
    bool   isProjected{};
    if (projectPlaneFootprintOntoSharedAxes(p_first_in,
                                            axisU_world,
                                            axisV_world,
                                            minimumU1,
                                            maximumU1,
                                            minimumV1,
                                            maximumV1,
                                            isProjected) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: projectPlaneFootprintOntoSharedAxes returned a failure status "
            "although it cannot fail; continuing as before.",
            __func__);
    }
    bool isProjected2{};
    if (!(!isProjected) &&
        projectPlaneFootprintOntoSharedAxes(p_second_in,
                                            axisU_world,
                                            axisV_world,
                                            minimumU2,
                                            maximumU2,
                                            minimumV2,
                                            maximumV2,
                                            isProjected2) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("vs_graphs"),
            "%s: projectPlaneFootprintOntoSharedAxes returned a failure status "
            "although it cannot fail; continuing as before.",
            __func__);
    }
    if (!isProjected || !isProjected2)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double overlapU_m =
        std::min(maximumU1, maximumU2) - std::max(minimumU1, minimumU2);
    const double overlapV_m =
        std::min(maximumV1, maximumV2) - std::max(minimumV1, minimumV2);

    if (overlapU_m <= 0.0 || overlapV_m <= 0.0)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double overlapArea_m2 = overlapU_m * overlapV_m;
    const double area1_m2 = (maximumU1 - minimumU1) * (maximumV1 - minimumV1);
    const double area2_m2 = (maximumU2 - minimumU2) * (maximumV2 - minimumV2);
    const double smallerArea_m2 = std::min(area1_m2, area2_m2);

    if (!std::isfinite(smallerArea_m2) || smallerArea_m2 < 1e-6)
    {
        arePlausibleTwinWallFaces_out = false;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    arePlausibleTwinWallFaces_out =
        (overlapArea_m2 / smallerArea_m2) >= minimumOverlapRatio_in;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
