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
 * @file            separateKeyFrameCloud.cc
 *
 * @brief           Implements SemanticSegmentation::separateKeyFrameCloud(),
 *                  declared in
 *                  semantic_segmentation/objects/SemanticSegmentationClass.h.
 */

#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "scene_understanding/semantic_segmentation/objects/SemanticSegmentationClass.h"
#include "scene_understanding/semantic_segmentation/private_functions/private_functions.h"

#include <cstdint>
#include <iostream>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic_segmentation
{

SemanticSegmentationStatus SemanticSegmentation::separateKeyFrameCloud(
    keyframe::KeyFrame                           *p_keyFrame_in,
    const pcl::PCLPointCloud2::Ptr               &p_segmentationCloud_in,
    const cv::Mat                                &uncertaintyImage_in,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_keyFrameCloud_in,
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &p_clsCloudPtrs_out)
{
    if (isSegmenterOff)
    {
        /*!
         * The whole keyframe cloud fills one slot. Each point's weight is its
         * depth weight alone, the weight threshSeparatePointCloud() gives a
         * point the segmenter is sure of.
         */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_wholeCloud(
            new pcl::PointCloud<pcl::PointXYZRGBA>);
        p_wholeCloud->is_dense = false;
        p_wholeCloud->height   = 1;
        for (const pcl::PointXYZRGB &keyFramePoint : p_keyFrameCloud_in->points)
        {
            if (!pcl::isFinite(keyFramePoint))
            {
                continue;
            }
            pcl::PointXYZRGBA wholeCloudPoint;
            wholeCloudPoint.x = keyFramePoint.x;
            wholeCloudPoint.y = keyFramePoint.y;
            wholeCloudPoint.z = keyFramePoint.z;
            wholeCloudPoint.r = keyFramePoint.r;
            wholeCloudPoint.g = keyFramePoint.g;
            wholeCloudPoint.b = keyFramePoint.b;
            if (computeDepthWeight(
                    keyFramePoint.z,
                    p_sysParams->pointcloud.distanceThresh.first,
                    p_sysParams->pointcloud.distanceThresh.second,
                    wholeCloudPoint.a) !=
                SemanticSegmentationStatus::
                    SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
            {
                /* computeDepthWeight cannot fail; continue as before. */
            }
            p_wholeCloud->push_back(wholeCloudPoint);
        }
        p_clsCloudPtrs_out.assign(1U, p_wholeCloud);
        return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
    }

    /* Extract the segmentation uncertainties from the image */
    cv::Mat segImageUncertainity = uncertaintyImage_in;

    /*!
     * Seperate points into classes and filtering based on thresholds.
     *
     * clsCloudPtrs contains the points from the segmented point cloud
     * seperated into a list of point clouds with the index of the list
     * representing the semantic type of each point.
     */
    if (threshSeparatePointCloud(p_segmentationCloud_in,
                                 segImageUncertainity,
                                 p_clsCloudPtrs_out,
                                 p_keyFrameCloud_in) !=
        SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: threshSeparatePointCloud returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /*!
     * Diagnostic visibility for a previously-silent failure mode: if the
     * model finds no (or very few) confident WALL-class pixels in this
     * keyframe -- e.g. a close, texture-poor, low-context wall filling
     * the frame -- getPlanesFromClassClouds() simply `continue`s
     * past an empty/undersized class cloud with no trace anywhere. That
     * made "the segmenter found nothing this frame" indistinguishable
     * from "there was nothing to find" in every existing log. Class
     * index 1 is WALL (utils::utils::Utils::getPlaneTypeFromClassId). Gate
     * on a small count, not just empty, so this stays quiet on ordinary
     * frames.
     */
    constexpr std::size_t WALL_CLASS_INDEX               = 1U;
    constexpr std::size_t WALL_SILENT_DROP_LOG_THRESHOLD = 50U;
    if (p_clsCloudPtrs_out.size() > WALL_CLASS_INDEX &&
        p_clsCloudPtrs_out[WALL_CLASS_INDEX]->size() <
            WALL_SILENT_DROP_LOG_THRESHOLD)
    {
        Eigen::Vector3f cameraCenter_world{};
        if (p_keyFrame_in->getCameraCenter(cameraCenter_world) !=
            keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCameraCenter returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemSeg] KF#" << p_keyFrame_in->id
                  << " wall-class points after confidence gating: "
                  << p_clsCloudPtrs_out[WALL_CLASS_INDEX]->size()
                  << " (camera at " << cameraCenter_world.x() << ','
                  << cameraCenter_world.y() << ',' << cameraCenter_world.z()
                  << ')' << std::endl;
    }

    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} /* namespace semantic_segmentation */
} /* namespace core */
} /* namespace vs_graphs */
