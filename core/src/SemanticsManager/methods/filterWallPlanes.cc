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

SemanticsManagerStatus SemanticsManager::filterWallPlanes(void)
{
    /* Iterate through all the planes and filter the walls */
    std::vector<vs_graphs::core::geometric::Plane *> atlasAllPlanes{};
    if (p_atlas->getAllPlanes(atlasAllPlanes) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *const &plane : atlasAllPlanes)
    {
        /* Skip planes which are not classed as walls */
        geometric::Plane::PlaneVariant planeExpectedPlaneType{};
        if (plane->getExpectedPlaneType(planeExpectedPlaneType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getExpectedPlaneType returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (planeExpectedPlaneType ==
            vs_graphs::core::geometric::Plane::PlaneVariant::WALL)
        {
            /*!
             * Wall validation based on the mPlanePoseMat only works if the
             * ground plane is set. Needs the correction matrix: mPlanePoseMat.
             */
            g2o::Plane3D planeGetGlobalEquation{};
            if (plane->getGlobalEquation(planeGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector3f transformedPlaneCoefficients{};
            if (transformPlaneEqToGroundReference(
                    planeGetGlobalEquation.coeffs(),
                    transformedPlaneCoefficients) !=
                SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: transformPlaneEqToGroundReference returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }

            /*!
             * If the transformed plane is vertical based on absolute value,
             * then assign semantic, otherwise ignore threshold should be
             * leniently set (ideally with correct ground plane reference, this
             * value should be close to 0.00)
             */
            if (std::abs(transformedPlaneCoefficients(1)) >
                p_sysParams->semSeg.maxTiltWall)
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

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
