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
 * @file         SemanticSegmentation.cc
 *
 * @brief        Implements segmentation in SemanticSegmentation.h.
 */

#include "SemanticSegmentation.h"

namespace vs_graphs
{
namespace core
{

void SemanticSegmentation::threshSeparatePointCloud(
    pcl::PCLPointCloud2::Ptr                              pclPc2SegPrb,
    cv::Mat                                              &segImgUncertainity,
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clsCloudPtrs,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr         &thisKFPointCloud)
{
    /* Extract parameters on thresholds */
    const uint8_t confidenceThresh = p_sysParams->semSeg.confThresh * 255;
    const float   probThresh       = p_sysParams->semSeg.probThresh;
    const float   distanceThreshNear =
        p_sysParams->pointcloud.distanceThresh.first;
    const float distanceThreshFar =
        p_sysParams->pointcloud.distanceThresh.second;

    /* Parse the PointCloud2 message */
    const int width      = pclPc2SegPrb->width;
    const int numPoints  = width * pclPc2SegPrb->height;
    const int pointStep  = pclPc2SegPrb->point_step;
    const int numClasses = pointStep / bytesPerClassProb;

    /* Clear the seperated point cloud vector `clsCloudPtrs` */
    clsCloudPtrs.clear();
    clsCloudPtrs.reserve(numClasses);

    /*!
     * For each semantic class, add an instance to the output clsCloudPtrs list.
     * This way the pointclouds can be put into there respective class.
     */
    for (int i = 0; i < numClasses; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        pointCloud->is_dense = false;
        pointCloud->height   = 1;
        clsCloudPtrs.push_back(pointCloud);
    }

    /* Extract the data from the inputted segmented point cloud */
    const uint8_t *data = pclPc2SegPrb->data.data();

    /*!
     * Iterate through the number of classes.
     *  j -> class index
     *  i -> flattened pixel index
     */
    for (int j = 0; j < numClasses; j++)
    {
        /* Iterate through the number of points */
        for (int i = 0; i < numPoints; i++)
        {
            /*!
             * Initilize variable containing probability point i belongs to
             * class j
             */
            float probability;

            /* Extract probabliilty that point i belongs to class j */
            std::memcpy(&probability,
                        data + pointStep * i + bytesPerClassProb * j +
                            pclPc2SegPrb->fields[0].offset,
                        bytesPerClassProb);

            /*!
             * Apply thresholding and track confidence (complement of
             * uncertainty). This is to check that the probability of the
             * semantic.
             */
            if (probability >= probThresh)
            {
                // /* Inject coordinates as a point to respective point cloud */
                /* Initialize point to be filtered into class point cloud */
                pcl::PointXYZRGBA point;

                /* Find the pixel index of the point in the image */
                point.y = static_cast<int>(i / width);
                point.x = i % width;

                /* Extract the original point from the keyframe point cloud */
                const pcl::PointXYZRGB origPoint =
                    thisKFPointCloud->at(point.x, point.y);

                /* If the original point has invalid data, skip data point */
                if (!pcl::isFinite(origPoint))
                {
                    continue;
                }

                /* Extract the rgb uncertainty from the image */
                cv::Vec3b vec =
                    segImgUncertainity.at<cv::Vec3b>(point.y, point.x);

                /*!
                 * Convert the rgb uncertainty of the pixel to a single value
                 * and store in the alpha channel.
                 */
                point.a =
                    255 - static_cast<int>(0.299 * vec[2] + 0.587 * vec[1] +
                                           0.114 * vec[0]);

                /*!
                 * Exclude the points with low confidence that segmentation was
                 * correct.
                 */
                if (point.a < confidenceThresh)
                {
                    continue;
                }

                /* Assign the XYZ and RGB values to the surviving point */
                point.x = origPoint.x;
                point.y = origPoint.y;
                point.z = origPoint.z;
                point.r = origPoint.r;
                point.g = origPoint.g;
                point.b = origPoint.b;

                /*!
                 * Confidence as the squared inverse depth - interpolated
                 * between near and far thresholds confidence = 255 for near, 45
                 * for far, and interpolated according to squared distance
                 */
                if (point.z < distanceThreshNear)
                {
                    point.a = 255;
                }
                else if (point.z > distanceThreshFar)
                {
                    point.a = 45;
                }
                else
                {
                    point.a =
                        255 - static_cast<int>(
                                  210 * sqrt((point.z - distanceThreshNear) /
                                             (distanceThreshFar -
                                              distanceThreshNear)));
                }

                /* Add the point to the respective class specific point cloud */
                clsCloudPtrs[j]->push_back(point);
            }
        }
    }

    /* Specify size/width and header for each class specific point cloud */
    for (int i = 0; i < numClasses; i++)
    {
        clsCloudPtrs[i]->width  = clsCloudPtrs[i]->size();
        clsCloudPtrs[i]->header = pclPc2SegPrb->header;
    }
}

} // namespace core
} // namespace vs_graphs
