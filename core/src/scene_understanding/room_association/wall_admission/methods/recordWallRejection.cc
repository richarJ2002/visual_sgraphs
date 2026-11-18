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
 * @file            recordWallRejection.cc
 *
 * @brief           Implements SemanticsManager::recordWallRejection(), declared
 * in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::recordWallRejection(
    geometric::Plane            *p_plane_in,
    int                          planeId_in,
    long long                    activeMapIdRecord_in,
    const char                  *variantTypeName_in,
    const WallAdmissionEvidence &admissionEvidence_in)
{
    /* Init variable which will hold the voted variant of the plane */
    geometric::Plane::PlaneVariant votedPlanVariant{};

    /* Extract the expected plane variation */
    if (p_plane_in->getExpectedPlaneType(votedPlanVariant) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getExpectedPlaneType returned a failure "
                     "status although it cannot fail; continuing as "
                     "before.",
                     __func__);
    }

    /* Init variable which will track reason plane should be rejected */
    std::string rejectionReason;

    /* The votes outrank the fit, which outranks the observation age */
    if (votedPlanVariant != geometric::Plane::PlaneVariant::WALL)
    {
        rejectionReason = "CLASS_NOT_WALL";
    }
    else if (!admissionEvidence_in.hasAdequateFiniteFit)
    {
        rejectionReason = "INADEQUATE_FINITE_FIT";
    }
    else
    {
        rejectionReason = "INSUFFICIENT_OBSERVATIONS";
    }

    /* Emit the rejection record only when the reason changes. */
    if (loggedWallRejectionReasons[planeId_in] != rejectionReason)
    {
        loggedWallRejectionReasons[planeId_in] = rejectionReason;
        std::cout << "SG_PIPELINE {\"event\":\"wall_rejection\","
                     "\"map_id\":"
                  << activeMapIdRecord_in
                  << ",\"semantic_cycle\":" << pipelineSemanticCycle
                  << ",\"wall_id\":" << planeId_in << ",\"class\":\""
                  << variantTypeName_in
                  << "\",\"lifecycle\":\"REJECTED\","
                     "\"owner\":\"NONE\",\"reason\":\""
                  << rejectionReason
                  << "\",\"support\":" << admissionEvidence_in.fittedPointCount
                  << ",\"observations\":"
                  << admissionEvidence_in.observationCount << "}" << std::endl;
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
