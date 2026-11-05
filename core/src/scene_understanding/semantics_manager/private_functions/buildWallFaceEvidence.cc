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
 * @file            buildWallFaceEvidence.cc
 *
 * @brief           Implements buildWallFaceEvidence(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <optional>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    buildWallFaceEvidence(geometric::Plane            *p_wall_in,
                          utils::utils::PlaneEvidence &wallEvidence_out)
{
    geometric::Plane::GeometrySnapshot wallGeometry{};
    if (p_wall_in->getGeometrySnapshot(wallGeometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometrySnapshot returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::optional<Eigen::Vector3d> observationOrigin_world_m{};
    if (p_wall_in->getObservationOrigin_world(observationOrigin_world_m) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationOrigin_world returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    wallEvidence_out                        = utils::utils::PlaneEvidence{};
    wallEvidence_out.planeEquation_world    = wallGeometry.planeEquation_world;
    wallEvidence_out.p_supportCloud_world_m = wallGeometry.supportCloud;
    wallEvidence_out.planeCentroid_world_m = wallGeometry.planeCentroid_world_m;
    wallEvidence_out.planeLabel = geometric::Plane::PlaneVariant::WALL;
    if (observationOrigin_world_m.has_value())
    {
        wallEvidence_out.observerSide = observationOrigin_world_m.value();
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
