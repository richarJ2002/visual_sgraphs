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

SemanticSegmentationStatus SemanticSegmentation::threshSeparatePointCloud(
    pcl::PCLPointCloud2::Ptr p_pclPc2SegPrb_in,
    cv::Mat                 &segImageUncertainity_in,
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &p_clsCloudPtrs_out,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_thisKeyFramePointCloud_in)
{
    /* Extract parameters on thresholds */
    const uint8_t confidenceThreshold =
        static_cast<uint8_t>(p_sysParams->semSeg.confThresh * 255);
    const float probThreshold = p_sysParams->semSeg.probThresh;
    const float distanceThresholdNear =
        p_sysParams->pointcloud.distanceThresh.first;
    const float distanceThresholdFar =
        p_sysParams->pointcloud.distanceThresh.second;

    /* Parse the PointCloud2 message */
    const int width      = p_pclPc2SegPrb_in->width;
    const int pointCount = width * p_pclPc2SegPrb_in->height;
    const int pointStep  = p_pclPc2SegPrb_in->point_step;
    const int classCount = pointStep / bytesPerClassProb;

    /* Clear the seperated point cloud vector `clsCloudPtrs` */
    p_clsCloudPtrs_out.clear();
    p_clsCloudPtrs_out.reserve(classCount);

    /*!
     * For each semantic class, add an instance to the output clsCloudPtrs list.
     * This way the pointclouds can be put into there respective class.
     */
    for (int classIndex = 0; classIndex < classCount; classIndex++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_pointCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        p_pointCloud->is_dense = false;
        p_pointCloud->height   = 1;
        p_clsCloudPtrs_out.push_back(p_pointCloud);
    }

    /* Extract the data from the inputted segmented point cloud */
    const uint8_t *p_data = p_pclPc2SegPrb_in->data.data();

    /*!
     * Iterate through the number of classes.
     *  j -> class index
     *  i -> flattened pixel index
     */
    for (int j = 0; j < classCount; j++)
    {
        /* Iterate through the number of points */
        for (int classIndex = 0; classIndex < pointCount; classIndex++)
        {
            /*!
             * Initilize variable containing probability point i belongs to
             * class j
             */
            float probability;

            /* Extract probabliilty that point i belongs to class j */
            std::memcpy(&probability,
                        p_data + pointStep * classIndex +
                            bytesPerClassProb * j +
                            p_pclPc2SegPrb_in->fields[0].offset,
                        bytesPerClassProb);

            /*!
             * Apply thresholding and track confidence (complement of
             * uncertainty). This is to check that the probability of the
             * semantic.
             */
            if (probability >= probThreshold)
            {
                // /* Inject coordinates as a point to respective point cloud */
                /* Initialize point to be filtered into class point cloud */
                pcl::PointXYZRGBA point;

                /* Find the pixel of the point in the image */
                const int pixelRow    = classIndex / width;
                const int pixelColumn = classIndex % width;

                /* Extract the original point from the keyframe point cloud */
                const pcl::PointXYZRGB origPoint =
                    p_thisKeyFramePointCloud_in->at(pixelColumn, pixelRow);

                /* If the original point has invalid data, skip data point */
                if (!pcl::isFinite(origPoint))
                {
                    continue;
                }

                /* Extract the rgb uncertainty from the image */
                cv::Vec3b vector =
                    segImageUncertainity_in.at<cv::Vec3b>(pixelRow,
                                                          pixelColumn);

                /*!
                 * Convert the rgb uncertainty of the pixel to a single value
                 * and store in the alpha channel.
                 */
                point.a = static_cast<uint8_t>(
                    255 -
                    static_cast<int>(0.299 * vector[2] + 0.587 * vector[1] +
                                     0.114 * vector[0]));

                /*!
                 * Exclude the points with low confidence that segmentation was
                 * correct.
                 */
                if (point.a < confidenceThreshold)
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
                if (point.z < distanceThresholdNear)
                {
                    point.a = 255;
                }
                else if (point.z > distanceThresholdFar)
                {
                    point.a = 45;
                }
                else
                {
                    point.a = static_cast<uint8_t>(
                        255 -
                        static_cast<int>(
                            210 * std::sqrt((point.z - distanceThresholdNear) /
                                            (distanceThresholdFar -
                                             distanceThresholdNear))));
                }

                /* Add the point to the respective class specific point cloud */
                p_clsCloudPtrs_out[j]->push_back(point);
            }
        }
    }

    /* Specify size/width and header for each class specific point cloud */
    for (int classIndex = 0; classIndex < classCount; classIndex++)
    {
        p_clsCloudPtrs_out[classIndex]->width =
            static_cast<uint32_t>(p_clsCloudPtrs_out[classIndex]->size());
        p_clsCloudPtrs_out[classIndex]->header = p_pclPc2SegPrb_in->header;
    }

    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
