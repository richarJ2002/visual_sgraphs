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
 * @file            runPlaneFusion.cc
 *
 * @brief           Implements SemanticsManager::runPlaneFusion(), declared in
 *                  semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::runPlaneFusion(void)
{
    /*!
     * Reconcile duplicate semantic planes before constructing room edges.
     *
     * Plane hypotheses are created from individual keyframe observations,
     * so one physical wall can initially exist as several overlapping or
     * contiguous fragments. Associating those fragments with a room first
     * creates duplicate graph edges and provisional structural elements.
     * The reconciliation pass validates equation, finite extent, and
     * observation-side compatibility before atomically rewiring every
     * existing reference to the retained plane (findMatchingPlane, then
     * relinkRetiredPlane and Plane::absorb, which thins the merged cloud).
     */
    if (p_sysParams->semSeg.reassociate.enabled &&
        reAssociateSemanticPlanes(p_atlas) !=
            SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reAssociateSemanticPlanes returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
