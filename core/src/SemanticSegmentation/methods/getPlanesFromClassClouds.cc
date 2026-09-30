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
 * @file            getPlanesFromClassClouds.cc
 *
 * @brief           Implements SemanticSegmentation::getPlanesFromClassClouds(),
 *                  declared in SemanticSegmentation.h.
 */

#include "SemanticSegmentation.h"
#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/objects/UtilsStatus.h"
#include <pcl/point_cloud.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticSegmentationStatus SemanticSegmentation::getPlanesFromClassClouds(
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &p_clsCloudPtrs_in,
    std::vector<std::vector<
        std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>>
        &planesFromClassClouds_out)
{
    std::vector<std::vector<
        std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>>
        p_clsPlanes;

    /* Downsample/filter the pointcloud and extract planes */
    for (size_t clsCloudPtrIndex = 0;
         clsCloudPtrIndex < p_clsCloudPtrs_in.size();
         clsCloudPtrIndex++)
    {
        // [TODO?] - Perhaps consider points in order of confidence instead of
        // downsampling Downsample the given pointcloud after filtering based on
        // distance

        /* Init variable for the filtered point cloud */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_filteredCloud;

        /*!
         * Filter points based on depth from sensor.
         *
         * @note        Parameter for min and max distance are defined as
         *              default values in:
         *              `visual_sgraphs/core/include/Types/SystemParams.h`
         */
        if (utils::utils::Utils::pointcloudDistanceFilter<pcl::PointXYZRGBA>(
                p_clsCloudPtrs_in[clsCloudPtrIndex],
                p_filteredCloud) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: pointcloudDistanceFilter returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Downsample points into grid based on points within voxel grid */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_downsampledCloud;
        if (utils::utils::Utils::pointcloudDownsample<pcl::PointXYZRGBA>(
                p_filteredCloud,
                p_sysParams->semSeg.pointcloud.downsample.leafSize,
                p_sysParams->semSeg.pointcloud.downsample.minPointsPerVoxel,
                p_downsampledCloud) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: pointcloudDownsample returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_filteredCloud = std::move(p_downsampledCloud);

        /*!
         * Filtering removes arbitrary points, so the result is no longer an
         * organized image cloud. Normalize its metadata before copying it;
         * retaining the input image width makes PCL infer an invalid height
         * and emits a warning on every semantic update.
         */
        p_filteredCloud->width = static_cast<uint32_t>(p_filteredCloud->size());
        p_filteredCloud->height = 1;

        /* Skip point clouds which are empty or have incalid width/height */
        if (p_filteredCloud->width == 0 || p_filteredCloud->height == 0 ||
            p_filteredCloud->empty())
        {
            continue;
        }

        /* Copy the filtered cloud for later storage into the keyframe */
        pcl::copyPointCloud(*p_filteredCloud,
                            *p_clsCloudPtrs_in[clsCloudPtrIndex]);

        /* Initialize object to contain extracted point clouds */
        std::vector<
            std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
            p_extractedPlanes;

        /*!
         * Extract planes from filtered point cloud if the number of points is
         * greater than a threshold. This parameter is set in
         * `system_params.yaml`
         */
        if (p_filteredCloud->points.size() > p_sysParams->seg.pointcloudsThresh)
        {
            if (utils::utils::Utils::ransacPlaneFitting<
                    pcl::PointXYZRGBA,
                    pcl::WeightedSACSegmentation>(p_filteredCloud,
                                                  p_extractedPlanes) !=
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: ransacPlaneFitting returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        p_clsPlanes.push_back(p_extractedPlanes);
    }
    planesFromClassClouds_out = p_clsPlanes;
    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
