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
 * @file            updatePlaneSemantics.cc
 *
 * @brief           Implements SemanticSegmentation::updatePlaneSemantics(),
 *                  declared in SemanticSegmentation.h.
 */

#include "SemanticSegmentation.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticSegmentationStatus
    SemanticSegmentation::updatePlaneSemantics(int    planeId_in,
                                               int    clsId_in,
                                               double confidence_in)
{
    // retrieve the plane from the map
    geometric::Plane *p_matchedPlane = nullptr;
    if (p_atlas->getPlaneById(planeId_in, p_matchedPlane) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneById returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    // plane type compatible with the Plane class
    vs_graphs::core::geometric::Plane::PlaneVariant planeType{};
    if (utils::utils::Utils::getPlaneTypeFromClassId(clsId_in, planeType) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneTypeFromClassId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    // cast a vote for the plane semantics
    if (p_matchedPlane->castWeightedVote(planeType, confidence_in) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: castWeightedVote returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
