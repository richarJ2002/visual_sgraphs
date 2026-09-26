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

#include "SemanticSegmentation.h"

namespace vs_graphs
{
namespace core
{

std::vector<std::vector<
    std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>>
    SemanticSegmentation::getPlanesFromClassClouds(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs)
{
    std::vector<std::vector<
        std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>>
        clsPlanes;

    /* Downsample/filter the pointcloud and extract planes */
    for (size_t i = 0; i < clsCloudPtrs.size(); i++)
    {
        // [TODO?] - Perhaps consider points in order of confidence instead of
        // downsampling Downsample the given pointcloud after filtering based on
        // distance

        /* Init variable for the filtered point cloud */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud;

        /*!
         * Filter points based on depth from sensor.
         *
         * @note        Parameter for min and max distance are defined as
         *              default values in:
         *              `visual_sgraphs/core/include/Types/SystemParams.h`
         */
        filteredCloud =
            utils::utils::Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(
                clsCloudPtrs[i]);

        /* Downsample points into grid based on points within voxel grid */
        filteredCloud =
            utils::utils::Utils::pointcloudDownsample<pcl::PointXYZRGBA>(
                filteredCloud,
                p_sysParams->semSeg.pointcloud.downsample.leafSize,
                p_sysParams->semSeg.pointcloud.downsample.minPointsPerVoxel);

        /* Remove points that are statically isolated from neighbors */
        filteredCloud =
            utils::utils::Utils::pointcloudOutlierRemoval<pcl::PointXYZRGBA>(
                filteredCloud,
                p_sysParams->semSeg.pointcloud.outlierRemoval.stdThreshold,
                p_sysParams->semSeg.pointcloud.outlierRemoval.meanThreshold);

        /*!
         * Filtering removes arbitrary points, so the result is no longer an
         * organized image cloud. Normalize its metadata before copying it;
         * retaining the input image width makes PCL infer an invalid height
         * and emits a warning on every semantic update.
         */
        filteredCloud->width  = filteredCloud->size();
        filteredCloud->height = 1;

        /* Skip point clouds which are empty or have incalid width/height */
        if (filteredCloud->width == 0 || filteredCloud->height == 0 ||
            filteredCloud->empty())
        {
            continue;
        }

        /* Copy the filtered cloud for later storage into the keyframe */
        pcl::copyPointCloud(*filteredCloud, *clsCloudPtrs[i]);

        /* Initialize object to contain extracted point clouds */
        std::vector<
            std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
            extractedPlanes;

        /*!
         * Extract planes from filtered point cloud if the number of points is
         * greater than a threshold. This parameter is set in
         * `system_params.yaml`
         */
        if (filteredCloud->points.size() > p_sysParams->seg.pointcloudsThresh)
        {
            extractedPlanes = utils::utils::Utils::ransacPlaneFitting<
                pcl::PointXYZRGBA,
                pcl::WeightedSACSegmentation>(filteredCloud);
        }
        clsPlanes.push_back(extractedPlanes);
    }
    return clsPlanes;
}

} // namespace core
} // namespace vs_graphs
