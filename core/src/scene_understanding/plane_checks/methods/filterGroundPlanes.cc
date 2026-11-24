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
 * @file            filterGroundPlanes.cc
 *
 * @brief           Implements SemanticsManager::filterGroundPlanes(), declared
 *                  in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantic_constraints/plane_rules/plane_rules.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::filterGroundPlanes(
    const std::vector<geometric::Plane *> &intakePlanes_in,
    geometric::Plane                      *p_mainGroundPlane_in,
    const Eigen::Vector3d                 &upDirection_world_in)
{
    PROFILE_FUNCTION();
    /*!
     * Discard gound planes that have a height above a threshold from the
     * biggest ground plane.
     *
     * [TODO] - Should determine if it is better to use the biggest ground plane
     *          or the lowest ground plane.
     */

    /* Get the median height of the plane to compute the threshold */
    std::optional<float> mainGroundHeight_down_m{};
    if (computeGroundPlaneHeight(p_mainGroundPlane_in,
                                 upDirection_world_in,
                                 mainGroundHeight_down_m) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeGroundPlaneHeight returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!mainGroundHeight_down_m.has_value())
    {
        /* Nothing to filter against yet -- the main ground plane's support
           cloud is momentarily empty (e.g. right after creation/reset). */
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract the main associated ground plane */
    int mainGroundPlaneId{};
    if (p_mainGroundPlane_in->getId(mainGroundPlaneId) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* Go through all ground planes to check validity */
    for (geometric::Plane *p_intakePlane : intakePlanes_in)
    {
        /* Skip planes not classed as ground, or are the main ground plane */
        geometric::Plane::PlaneVariant expectedPlaneType{};
        if (p_intakePlane->getExpectedPlaneType(expectedPlaneType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getExpectedPlaneType returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int intakePlaneId{};
        if (!(expectedPlaneType != geometric::Plane::PlaneVariant::GROUND) &&
            p_intakePlane->getId(intakePlaneId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (expectedPlaneType != geometric::Plane::PlaneVariant::GROUND ||
            intakePlaneId == mainGroundPlaneId)
        {
            continue;
        }

        /* A plane whose support cloud is momentarily empty has no height
           yet; the ground step rule then keeps it. */
        semantic_constraints::PlaneRuleInput planeRuleInput{};
        planeRuleInput.planeLabel              = expectedPlaneType;
        planeRuleInput.mainGroundHeight_down_m = mainGroundHeight_down_m;
        if (computeGroundPlaneHeight(p_intakePlane,
                                     upDirection_world_in,
                                     planeRuleInput.planeHeight_down_m) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: computeGroundPlaneHeight returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Orientation was judged by rejectMisorientedPlanes(); no normal is
         * given, so only the ground step rule decides here. */
        if (!semantic_constraints::arePlaneRulesSatisfied(planeRuleInput,
                                                          *p_sysParams))
        {
            if (p_intakePlane->resetPlaneSemantics() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: resetPlaneSemantics returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
