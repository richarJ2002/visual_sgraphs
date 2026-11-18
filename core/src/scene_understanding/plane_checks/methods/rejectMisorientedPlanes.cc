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
 * @file            rejectMisorientedPlanes.cc
 *
 * @brief           Implements SemanticsManager::rejectMisorientedPlanes(),
 *                  declared in
 * semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantic_constraints/plane_rules/plane_rules.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <cmath>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::rejectMisorientedPlanes(
    const std::vector<geometric::Plane *> &intakePlanes_in,
    const Eigen::Vector3d                 &upDirection_world_in)
{
    PROFILE_FUNCTION();
    for (geometric::Plane *p_intakePlane : intakePlanes_in)
    {
        /* Only level and wall planes have an orientation to validate */
        geometric::Plane::PlaneVariant expectedPlaneType{};
        if (p_intakePlane->getExpectedPlaneType(expectedPlaneType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getExpectedPlaneType returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (expectedPlaneType != geometric::Plane::PlaneVariant::GROUND &&
            expectedPlaneType != geometric::Plane::PlaneVariant::CEILING &&
            expectedPlaneType != geometric::Plane::PlaneVariant::WALL)
        {
            continue;
        }

        g2o::Plane3D globalPlaneEquation{};
        if (p_intakePlane->getGlobalEquation(globalPlaneEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Skip planes whose normal is unusable (non-finite or degenerate) */
        const Eigen::Vector4d planeEquation_world =
            globalPlaneEquation.coeffs();
        const double planeNormalNorm = planeEquation_world.head<3>().norm();
        if (!planeEquation_world.allFinite() ||
            !std::isfinite(planeNormalNorm) || planeNormalNorm < 1e-6)
        {
            continue;
        }

        /*!
         * Judging against the Atlas up direction (and not against the main
         * ground plane) stops a wall labelled as ground from becoming the
         * reference that is never rejected. The tilt rules of the plane rule
         * table decide; no height is given, so the ground step rule waits for
         * filterGroundPlanes().
         */
        semantic_constraints::PlaneRuleInput planeRuleInput{};
        planeRuleInput.planeLabel     = expectedPlaneType;
        planeRuleInput.hasOrientation = true;
        planeRuleInput.planeNormal_common =
            planeEquation_world.head<3>() / planeNormalNorm;
        planeRuleInput.upDirection_common = upDirection_world_in;
        if (!semantic_constraints::arePlaneRulesSatisfied(planeRuleInput,
                                                          *p_sysParams))
        {
            if (p_intakePlane->resetPlaneSemantics() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: resetPlaneSemantics returned a failure "
                             "status although it cannot fail; continuing as "
                             "before.",
                             __func__);
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
