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

#include "SemanticsManager.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::filterGroundPlanes(geometric::Plane *p_groundPlane_in)
{
    /*!
     * Discard gound planes that have a height above a threshold from the
     * biggest ground plane.
     *
     * [TODO] - Should determine if it is better to use the biggest ground plane
     *          or the lowest ground plane.
     */

    /* Get the median height of the plane to compute the threshold */
    std::optional<float> groundPlaneHeight =
        computeGroundPlaneHeight(p_groundPlane_in);
    if (!groundPlaneHeight.has_value())
    {
        /* Nothing to filter against yet -- the main ground plane's support
           cloud is momentarily empty (e.g. right after creation/reset). */
        return;
    }
    float thresholdY =
        *groundPlaneHeight - p_sysParams->semSeg.maxStepElevation;

    /* Extract the main associated ground plane */
    int groundPlaneId{};
    if (p_groundPlane_in->getId(groundPlaneId) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }

    /* Go through all ground planes to check validity */
    for (const auto &plane : p_atlas->getAllPlanes())
    {
        /* Skip planes not classed as ground, or are the main ground plane */
        geometric::Plane::PlaneVariant planeExpectedPlaneType{};
        if (plane->getExpectedPlaneType(planeExpectedPlaneType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getExpectedPlaneType returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int planeGetId{};
        if (!(planeExpectedPlaneType !=
              vs_graphs::core::geometric::Plane::PlaneVariant::GROUND) &&
            plane->getId(planeGetId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (planeExpectedPlaneType !=
                vs_graphs::core::geometric::Plane::PlaneVariant::GROUND ||
            planeGetId == groundPlaneId)
        {
            continue;
        }

        /* If planes above inverted y threshold, then reset plane semantics.
           Skip (don't filter) a plane whose support cloud is momentarily
           empty -- there's nothing to judge its height against yet. */
        std::optional<float> planeHeight = computeGroundPlaneHeight(plane);
        if (!planeHeight.has_value())
        {
            continue;
        }
        if (*planeHeight < thresholdY)
        {
            if (plane->resetPlaneSemantics() !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: resetPlaneSemantics returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            continue;
        }

        /* Find trnsform of the plane */
        g2o::Plane3D planeGetGlobalEquation{};
        if (plane->getGlobalEquation(planeGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector3f transformedPlaneCoefficients =
            transformPlaneEqToGroundReference(planeGetGlobalEquation.coeffs());

        /*!
         * If the transformed plane is horizontal based on absolute value, then
         * assign semantic.
         *
         * Ignore threshold should be lenient. With correct ground plane
         * reference, this value should be close to 0.00.
         */
        if (abs(transformedPlaneCoefficients(0)) >
            p_sysParams->semSeg.maxTiltGround)
        {
            if (plane->resetPlaneSemantics() !=
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
}

} // namespace core
} // namespace vs_graphs
