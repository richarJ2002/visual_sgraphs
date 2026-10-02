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
 * @file            findLargestWallComponent.cc
 *
 * @brief           Implements findLargestWallComponent(), declared in
 *                  SemanticSegmentation/private_functions.h.
 */

#include "SemanticSegmentation.h"

#include "private_functions.h"

#include <cmath>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Finds the largest Euclidean component of a proposed wall
 *               plane.
 *
 *               The returned indices refer to the input cloud, allowing
 *               the same support to be selected in both camera and map
 *               frames. Invalid depth samples are excluded before
 *               building the search tree.
 *
 * @param[in]    p_wallCloud_in
 *               Proposed wall support cloud.
 * @param[in]    clusterTolerance_m_in
 *               Maximum Euclidean neighbour separation in metres.
 *
 * @param[out]   largestWallComponent_out
 *               Largest connected component and its support statistics.
 *
 * @return       SEMANTIC_SEGMENTATION_STATUS_SUCCESS.
 */
SemanticSegmentationStatus findLargestWallComponent(
    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &p_wallCloud_in,
    const double                                        clusterTolerance_m_in,
    WallComponentSupport &largestWallComponent_out)
{
    WallComponentSupport support;

    if (p_wallCloud_in == nullptr || p_wallCloud_in->empty() ||
        !std::isfinite(clusterTolerance_m_in) || clusterTolerance_m_in <= 0.0)
    {
        largestWallComponent_out = support;
        return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_finiteWallCloud(
        new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::vector<int> finiteSourceIndices;
    p_finiteWallCloud->reserve(p_wallCloud_in->size());
    finiteSourceIndices.reserve(p_wallCloud_in->size());

    for (std::size_t pointIndex = 0U; pointIndex < p_wallCloud_in->size();
         pointIndex++)
    {
        if (!pcl::isFinite(p_wallCloud_in->points[pointIndex]))
        {
            continue;
        }

        p_finiteWallCloud->push_back(p_wallCloud_in->points[pointIndex]);
        finiteSourceIndices.push_back(static_cast<int>(pointIndex));
    }

    support.finitePointCount = p_finiteWallCloud->size();

    if (p_finiteWallCloud->empty())
    {
        largestWallComponent_out = support;
        return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
    }

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr p_searchTree(
        new pcl::search::KdTree<pcl::PointXYZRGBA>);
    p_searchTree->setInputCloud(p_finiteWallCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clusterExtraction;
    clusterExtraction.setClusterTolerance(clusterTolerance_m_in);
    clusterExtraction.setMinClusterSize(1);
    clusterExtraction.setMaxClusterSize(
        static_cast<int>(p_finiteWallCloud->size()));
    clusterExtraction.setSearchMethod(p_searchTree);
    clusterExtraction.setInputCloud(p_finiteWallCloud);

    std::vector<pcl::PointIndices> connectedComponents;
    clusterExtraction.extract(connectedComponents);

    if (connectedComponents.empty())
    {
        largestWallComponent_out = support;
        return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
    }

    const std::vector<pcl::PointIndices>::iterator largestComponentIterator =
        std::max_element(connectedComponents.begin(),
                         connectedComponents.end(),
                         [](const pcl::PointIndices &leftComponent,
                            const pcl::PointIndices &rightComponent) {
                             return leftComponent.indices.size() <
                                    rightComponent.indices.size();
                         });

    support.pointIndices.reserve(largestComponentIterator->indices.size());

    for (const int finitePointIndex : largestComponentIterator->indices)
    {
        if (finitePointIndex < 0 ||
            static_cast<std::size_t>(finitePointIndex) >=
                finiteSourceIndices.size())
        {
            continue;
        }

        support.pointIndices.push_back(
            finiteSourceIndices[static_cast<std::size_t>(finitePointIndex)]);
    }

    support.componentRatio = static_cast<double>(support.pointIndices.size()) /
                             static_cast<double>(support.finitePointCount);

    largestWallComponent_out = support;
    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
