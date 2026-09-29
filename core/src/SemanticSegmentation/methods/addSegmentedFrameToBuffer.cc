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

#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticSegmentationStatus SemanticSegmentation::addSegmentedFrameToBuffer(
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *p_tuple_in)
{
    const std::uint64_t keyFrameId = std::get<0>(*p_tuple_in);
    KeyFrame           *p_keyFrame = nullptr;
    if (p_atlas->getKeyFrameById(keyFrameId, p_keyFrame) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKeyFrameById returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Map *p_keyFrameMap = nullptr;
    if (!(p_keyFrame == nullptr) &&
        p_keyFrame->getMap(p_keyFrameMap) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Map *p_sourceMap = p_keyFrame == nullptr ? nullptr : p_keyFrameMap;

    WorkItem droppedItem;
    bool     didDrop = false;
    {
        std::lock_guard<std::mutex> lock(newKeyFramesMutex);
        if (segmentedImageBuffer.size() >= MAX_BUFFERED_WORK_ITEMS)
        {
            droppedItem = std::move(segmentedImageBuffer.front());
            segmentedImageBuffer.pop_front();
            didDrop = true;
        }

        WorkItem workItem;
        workItem.keyFrameId = keyFrameId;
        unsigned long sourceMapId2{};
        if (!(p_sourceMap == nullptr) &&
            p_sourceMap->getId(sourceMapId2) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        workItem.sourceMapId       = p_sourceMap == nullptr
                                         ? std::numeric_limits<std::uint64_t>::max()
                                         : sourceMapId2;
        workItem.uncertaintyImage  = std::get<1>(*p_tuple_in);
        workItem.segmentationCloud = std::get<2>(*p_tuple_in);
        segmentedImageBuffer.push_back(std::move(workItem));
        enqueuedCount.fetch_add(1U, std::memory_order_relaxed);

        const std::uint32_t queueDepth =
            static_cast<std::uint32_t>(segmentedImageBuffer.size());
        std::uint32_t priorHighWatermark =
            queueHighWatermark.load(std::memory_order_relaxed);
        while (queueDepth > priorHighWatermark &&
               !queueHighWatermark.compare_exchange_weak(
                   priorHighWatermark,
                   queueDepth,
                   std::memory_order_relaxed))
        {}
    }

    if (didDrop)
    {
        if (recordTerminalOutcome(droppedItem.keyFrameId,
                                  TerminalOutcome::QUEUE_DROPPED) !=
            SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: recordTerminalOutcome returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return SemanticSegmentationStatus::SEMANTIC_SEGMENTATION_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
