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
 * @file            computeDepthWeight.cc
 *
 * @brief           Implements computeDepthWeight(), declared in
 *                  semantic_segmentation/private_functions/private_functions.h.
 */

#include "scene_understanding/semantic_segmentation/private_functions/private_functions.h"

#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

SemanticSegmentationStatus computeDepthWeight(const float   depth_m_in,
                                              const float   nearDepth_m_in,
                                              const float   farDepth_m_in,
                                              std::uint8_t &depthWeight_out)
{
    if (depth_m_in < nearDepth_m_in)
    {
        depthWeight_out = 255;
    }
    else if (depth_m_in > farDepth_m_in)
    {
        depthWeight_out = 45;
    }
    else
    {
        depthWeight_out = static_cast<std::uint8_t>(
            255 - static_cast<int>(
                      210 * std::sqrt((depth_m_in - nearDepth_m_in) /
                                      (farDepth_m_in - nearDepth_m_in))));
    }
    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */
