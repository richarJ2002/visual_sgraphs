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
 * @file            computePlaneFitRms.cc
 *
 * @brief           Implements computePlaneFitRms(), declared in
 *                  semantic_segmentation/private_functions/private_functions.h.
 */

#include "scene_understanding/semantic_segmentation/private_functions/private_functions.h"

#include <cmath>
#include <limits>

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

SemanticSegmentationStatus computePlaneFitRms(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_planeCloud_in,
    const Eigen::Vector4d                              &planeEquation_in,
    double                                             &fitRms_m_out)
{
    fitRms_m_out              = std::numeric_limits<double>::infinity();
    const double normalLength = planeEquation_in.head<3>().norm();
    if (p_planeCloud_in == nullptr || p_planeCloud_in->empty() ||
        !planeEquation_in.allFinite() || normalLength < 1e-6)
    {
        return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
    }

    /* Point-to-plane distance is |n . p + d| / |n|, summed squared. */
    double squaredDistanceSum_m2 = 0.0;
    for (const pcl::PointXYZRGBA &planePoint : p_planeCloud_in->points)
    {
        const double pointDistance_m =
            (planeEquation_in.head<3>().dot(
                 planePoint.getVector3fMap().cast<double>()) +
             planeEquation_in(3)) /
            normalLength;
        squaredDistanceSum_m2 += pointDistance_m * pointDistance_m;
    }
    fitRms_m_out = std::sqrt(squaredDistanceSum_m2 /
                             static_cast<double>(p_planeCloud_in->size()));
    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */
