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
 * @file            ransacPlaneFitting.cc
 *
 * @brief           Implements the
 *                  Utils::ransacPlaneFitting(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"

#include <iostream>
#include <utility>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

template <typename PointT, template <typename> class SegmentationType>
UtilsStatus Utils::ransacPlaneFitting(
    typename pcl::PointCloud<PointT>::Ptr   &cloud_inout,
    std::vector<std::pair<typename pcl::PointCloud<PointT>::Ptr,
                          Eigen::Vector4d>> &planes_out)
{
    /* Initialize Variables */
    std::vector<
        std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>>
                         p_extractedPlanes;
    types::SystemParams *p_sysParams = nullptr;
    if (types::SystemParams::getParams(p_sysParams) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getParams returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Extract planes from point clouds */
    for (unsigned int i = 0;
         i < p_sysParams->seg.ransac.maxPlanes &&
         cloud_inout->points.size() > p_sysParams->seg.pointcloudsThresh;
         i++)
    {
        try
        {
            /* Create objects for RANSAC plane segmentation */
            typename pcl::ExtractIndices<PointT> extract;
            pcl::PointIndices::Ptr      p_inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr p_coeffs(new pcl::ModelCoefficients);

            /* Create the SAC segmentation object */
            SegmentationType<PointT> seg;

            /* Fill the values of the segmentation object */
            seg.setInputCloud(cloud_inout);
            seg.setNumberOfThreads(8);
            seg.setMaxIterations(p_sysParams->seg.ransac.maxIterations);
            seg.setDistanceThreshold(p_sysParams->seg.ransac.distanceThresh);
            seg.setOptimizeCoefficients(true);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setModelType(pcl::SACMODEL_PLANE);

            /*!
             * Apply RANSAC segmentation.
             * Inliers contains the index of points used to find coefficients
             */
            seg.segment(*p_inliers, *p_coeffs);

            if (p_inliers->indices.empty() || p_coeffs->values.size() < 4)
            {
                break;
            }

            /* Calculate normal on the plane */
            Eigen::Vector4d planeEquation(p_coeffs->values[0],
                                          p_coeffs->values[1],
                                          p_coeffs->values[2],
                                          p_coeffs->values[3]);

            /* Calculate the closest points */
            Eigen::Vector4d plane;
            Eigen::Vector3d closestPoint =
                planeEquation.head(3) * planeEquation(3);
            plane.head(3) = closestPoint / closestPoint.norm();
            plane(3)      = closestPoint.norm();

            /* Init an object to contain a pointcloud within the plane */
            typename pcl::PointCloud<PointT>::Ptr p_extractedCloud(
                new pcl::PointCloud<PointT>);

            /* Create a point cloud_inout containing the points within the plane
             */
            for (const int &inlierIndex : p_inliers->indices)
            {
                /* Fill the point cloud_inout with indices */
                PointT inPoint;
                inPoint.r = cloud_inout->points[inlierIndex].r;
                inPoint.g = cloud_inout->points[inlierIndex].g;
                inPoint.b = cloud_inout->points[inlierIndex].b;
                inPoint.x = cloud_inout->points[inlierIndex].x;
                inPoint.y = cloud_inout->points[inlierIndex].y;
                inPoint.z = cloud_inout->points[inlierIndex].z;
                inPoint.a = cloud_inout->points[inlierIndex].a;

                /* Add the point to the extrcated cloud_inout of the plane */
                p_extractedCloud->points.push_back(inPoint);
            }

            /* Set the width of the cloud_inout */
            p_extractedCloud->width =
                static_cast<std::uint32_t>(p_extractedCloud->points.size());

            /* Set the height of the cloud_inout */
            p_extractedCloud->height = 1;

            /* Extract flag to indicate that the cloud_inout is dense */
            p_extractedCloud->is_dense = cloud_inout->is_dense;

            /* Add the extracted cloud_inout to the vector */
            p_extractedPlanes.push_back(
                std::make_pair(p_extractedCloud, plane));

            /* Remove the inliers into a distinct output cloud_inout. PCL
             * filters do not preserve organized-cloud_inout metadata reliably
             * when their input and output alias; that produced width/size
             * mismatches during repeated plane extraction. */
            extract.setInputCloud(cloud_inout);
            extract.setIndices(p_inliers);
            extract.setNegative(true);
            typename pcl::PointCloud<PointT>::Ptr p_remainingCloud(
                new pcl::PointCloud<PointT>);
            extract.filter(*p_remainingCloud);
            p_remainingCloud->header   = cloud_inout->header;
            p_remainingCloud->width    = p_remainingCloud->size();
            p_remainingCloud->height   = 1;
            p_remainingCloud->is_dense = cloud_inout->is_dense;
            cloud_inout                = std::move(p_remainingCloud);
        }
        catch (const std::exception &e)
        {
            std::cout << "RANSAC model error!" << std::endl;
        }
    }
    planes_out = p_extractedPlanes;
    return UtilsStatus::UTILS_STATUS_SUCCESS;
}
template UtilsStatus
    Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::SACSegmentation>(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &,
        std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                              Eigen::Vector4d>> &);
template UtilsStatus
    Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::WeightedSACSegmentation>(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &,
        std::vector<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                              Eigen::Vector4d>> &);

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
