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

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

template <typename PointT, template <typename> class SegmentationType>
std::vector<std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>>
    Utils::ransacPlaneFitting(
        typename pcl::PointCloud<PointT>::Ptr &cloud_inout)
{
    /* Initialize Variables */
    std::vector<
        std::pair<typename pcl::PointCloud<PointT>::Ptr, Eigen::Vector4d>>
                         extractedPlanes;
    types::SystemParams *p_sysParams = types::SystemParams::getParams();

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
            pcl::PointIndices::Ptr               inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

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
            seg.segment(*inliers, *coeffs);

            if (inliers->indices.empty() || coeffs->values.size() < 4)
            {
                break;
            }

            /* Calculate normal on the plane */
            Eigen::Vector4d planeEquation(coeffs->values[0],
                                          coeffs->values[1],
                                          coeffs->values[2],
                                          coeffs->values[3]);

            /* Calculate the closest points */
            Eigen::Vector4d plane;
            Eigen::Vector3d closestPoint =
                planeEquation.head(3) * planeEquation(3);
            plane.head(3) = closestPoint / closestPoint.norm();
            plane(3)      = closestPoint.norm();

            /* Init an object to contain a pointcloud within the plane */
            typename pcl::PointCloud<PointT>::Ptr extractedCloud(
                new pcl::PointCloud<PointT>);

            /* Create a point cloud_inout containing the points within the plane
             */
            for (const auto &idx : inliers->indices)
            {
                /* Fill the point cloud_inout with indices */
                PointT inPoint;
                inPoint.r = cloud_inout->points[idx].r;
                inPoint.g = cloud_inout->points[idx].g;
                inPoint.b = cloud_inout->points[idx].b;
                inPoint.x = cloud_inout->points[idx].x;
                inPoint.y = cloud_inout->points[idx].y;
                inPoint.z = cloud_inout->points[idx].z;
                inPoint.a = cloud_inout->points[idx].a;

                /* Add the point to the extrcated cloud_inout of the plane */
                extractedCloud->points.push_back(inPoint);
            }

            /* Set the width of the cloud_inout */
            extractedCloud->width =
                static_cast<std::uint32_t>(extractedCloud->points.size());

            /* Set the height of the cloud_inout */
            extractedCloud->height = 1;

            /* Extract flag to indicate that the cloud_inout is dense */
            extractedCloud->is_dense = cloud_inout->is_dense;

            /* Add the extracted cloud_inout to the vector */
            extractedPlanes.push_back(std::make_pair(extractedCloud, plane));

            /* Remove the inliers into a distinct output cloud_inout. PCL
             * filters do not preserve organized-cloud_inout metadata reliably
             * when their input and output alias; that produced width/size
             * mismatches during repeated plane extraction. */
            extract.setInputCloud(cloud_inout);
            extract.setIndices(inliers);
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
    return extractedPlanes;
}
template std::vector<
    std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
    Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::SACSegmentation>(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);
template std::vector<
    std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::Vector4d>>
    Utils::ransacPlaneFitting<pcl::PointXYZRGBA, pcl::WeightedSACSegmentation>(
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
