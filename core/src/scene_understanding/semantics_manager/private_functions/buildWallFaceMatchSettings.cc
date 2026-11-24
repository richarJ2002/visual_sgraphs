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
 * @file            buildWallFaceMatchSettings.cc
 *
 * @brief           Implements buildWallFaceMatchSettings(), declared in
 *                  SemanticsManager/private_functions.h.
 */

#include "configuration/types/objects/SystemParamsStatusEnum.h"
#include "configuration/types/types.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus buildWallFaceMatchSettings(
    const Eigen::Vector3d            &groundNormal_world_in,
    utils::utils::PlaneMatchSettings &matchSettings_out)
{
    types::SystemParams *p_systemParams = nullptr;
    if (types::SystemParams::getParams(p_systemParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    matchSettings_out           = utils::utils::PlaneMatchSettings{};
    matchSettings_out.matchMode = utils::utils::PlaneMatchMode::OPPOSITE_FACE;
    matchSettings_out.maximumAngle_rad =
        p_systemParams->roomSeg.wallsParallelismThresh *
        utils::utils::Utils::DEG_TO_RAD;
    matchSettings_out.maximumThickness_m = static_cast<double>(
        p_systemParams->semSeg.wallPairing.maximumThickness_m);
    matchSettings_out.minimumOverlapRatio = static_cast<double>(
        p_systemParams->semSeg.wallPairing.minimumOverlapRatio);
    matchSettings_out.groundNormal_world = groundNormal_world_in;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
