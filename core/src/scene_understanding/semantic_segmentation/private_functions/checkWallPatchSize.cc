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
 * @file            checkWallPatchSize.cc
 *
 * @brief           Implements checkWallPatchSize(), declared in
 *                  semantic_segmentation/private_functions/private_functions.h.
 */

#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/semantic_segmentation/private_functions/private_functions.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

SemanticSegmentationStatus checkWallPatchSize(
    const types::SystemParams::PlaneCheckpoint::WallSize &wallSize_in,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr              &p_wallCloud_inout,
    bool                                                 &isLargeEnough_out)
{
    isLargeEnough_out = false;
    if (p_wallCloud_inout == nullptr)
    {
        return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
    }

    /* Connected support: the largest component, or every point. */
    WallComponentSupport connectedSupport{};
    if (wallSize_in.connectivity.enabled)
    {
        if (findLargestWallComponent(
                p_wallCloud_inout,
                wallSize_in.connectivity.clusterTolerance_m,
                connectedSupport) !=
            SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            /* findLargestWallComponent cannot fail; continue as before. */
        }
    }
    else
    {
        connectedSupport.finitePointCount = p_wallCloud_inout->size();
        connectedSupport.componentRatio   = 1.0;
        connectedSupport.pointIndices.resize(p_wallCloud_inout->size());
        std::iota(connectedSupport.pointIndices.begin(),
                  connectedSupport.pointIndices.end(),
                  0);
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_connectedWallCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    p_connectedWallCloud->reserve(connectedSupport.pointIndices.size());
    for (const int sourcePointIndex : connectedSupport.pointIndices)
    {
        if (sourcePointIndex >= 0 &&
            static_cast<std::size_t>(sourcePointIndex) <
                p_wallCloud_inout->size())
        {
            p_connectedWallCloud->push_back(
                p_wallCloud_inout
                    ->points[static_cast<std::size_t>(sourcePointIndex)]);
        }
    }

    /* Finite in-plane dimensions of the connected support (PCA extents). */
    std::pair<double, double> wallDimensions_m{};
    if (utils::utils::Utils::computePlaneWidthHeight(p_connectedWallCloud,
                                                     wallDimensions_m) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        /* computePlaneWidthHeight cannot fail; continue as before. */
    }
    const double majorExtent_m =
        std::max(wallDimensions_m.first, wallDimensions_m.second);
    const double minorExtent_m =
        std::min(wallDimensions_m.first, wallDimensions_m.second);
    const double observedArea_m2 = majorExtent_m * minorExtent_m;

    /* Doorframe-sized, sparse or scattered patches are not walls. */
    const bool isConnectedEnough =
        !wallSize_in.connectivity.enabled ||
        (connectedSupport.pointIndices.size() >=
             wallSize_in.connectivity.minimumComponentPointCount &&
         connectedSupport.componentRatio >=
             wallSize_in.connectivity.minimumComponentRatio);
    isLargeEnough_out =
        p_connectedWallCloud->size() >= wallSize_in.minimumPointCount &&
        isConnectedEnough && std::isfinite(majorExtent_m) &&
        std::isfinite(minorExtent_m) && std::isfinite(observedArea_m2) &&
        majorExtent_m >= wallSize_in.minimumMajorExtent_m &&
        minorExtent_m >= wallSize_in.minimumMinorExtent_m &&
        observedArea_m2 >= wallSize_in.minimumArea_m2;

    /* Persist only the validated connected support. */
    if (isLargeEnough_out)
    {
        p_wallCloud_inout = p_connectedWallCloud;
    }
    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */
