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
 * declared in SemanticsManager.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"
#include <cmath>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::rejectMisorientedPlanes(void)
{
    PROFILE_FUNCTION();
    /* Iterate through all the planes and reject the misoriented ones */
    std::vector<vs_graphs::core::geometric::Plane *> atlasAllPlanes{};
    if (p_atlas->getAllPlanes(atlasAllPlanes) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (geometric::Plane *const &plane : atlasAllPlanes)
    {
        /* Only ground and wall planes have an orientation to validate */
        geometric::Plane::PlaneVariant planeExpectedPlaneType{};
        if (plane->getExpectedPlaneType(planeExpectedPlaneType) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getExpectedPlaneType returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool planeIsGround =
            planeExpectedPlaneType ==
            vs_graphs::core::geometric::Plane::PlaneVariant::GROUND;
        const bool planeIsWall =
            planeExpectedPlaneType ==
            vs_graphs::core::geometric::Plane::PlaneVariant::WALL;
        if (!planeIsGround && !planeIsWall)
        {
            continue;
        }

        g2o::Plane3D planeGetGlobalEquation{};
        if (plane->getGlobalEquation(planeGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Skip planes whose normal is unusable (non-finite or degenerate) */
        const Eigen::Vector4d planeEquation_world =
            planeGetGlobalEquation.coeffs();
        const double planeNormalNorm = planeEquation_world.head<3>().norm();
        if (!planeEquation_world.allFinite() ||
            !std::isfinite(planeNormalNorm) || planeNormalNorm < 1e-6)
        {
            continue;
        }

        /*!
         * The world vertical axis is Y, the same axis the ground-to-horizontal
         * transform assumes. Judging against it (and not against the main
         * ground plane) stops a wall labelled as ground from becoming the
         * reference that is never rejected. The tilt of a ground plane is its
         * normal's horizontal part; the tilt of a wall is its normal's
         * vertical part.
         */
        const Eigen::Vector3d planeNormal_world =
            planeEquation_world.head<3>() / planeNormalNorm;
        const double groundTilt =
            std::hypot(planeNormal_world.x(), planeNormal_world.z());
        const double wallTilt = std::abs(planeNormal_world.y());
        if ((planeIsGround && groundTilt > p_sysParams->semSeg.maxTiltGround) ||
            (planeIsWall && wallTilt > p_sysParams->semSeg.maxTiltWall))
        {
            if (plane->resetPlaneSemantics() !=
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
