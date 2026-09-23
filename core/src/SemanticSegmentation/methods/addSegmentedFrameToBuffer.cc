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

#include <limits>

namespace vs_graphs
{
namespace core
{

void SemanticSegmentation::addSegmentedFrameToBuffer(
    std::tuple<uint64_t, cv::Mat, pcl::PCLPointCloud2::Ptr> *tuple)
{
    const std::uint64_t keyFrameId = std::get<0>(*tuple);
    KeyFrame           *p_keyFrame = p_atlas->getKeyFrameById(keyFrameId);
    Map *p_sourceMap = p_keyFrame == nullptr ? nullptr : p_keyFrame->getMap();

    WorkItem droppedItem;
    bool     didDrop = false;
    {
        std::lock_guard<std::mutex> lock(mMutexNewKFs);
        if (segmentedImageBuffer.size() >= MAX_BUFFERED_WORK_ITEMS)
        {
            droppedItem = std::move(segmentedImageBuffer.front());
            segmentedImageBuffer.pop_front();
            didDrop = true;
        }

        WorkItem workItem;
        workItem.keyFrameId        = keyFrameId;
        workItem.sourceMapId       = p_sourceMap == nullptr
                                         ? std::numeric_limits<std::uint64_t>::max()
                                         : p_sourceMap->getId();
        workItem.uncertaintyImage  = std::get<1>(*tuple);
        workItem.segmentationCloud = std::get<2>(*tuple);
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
        recordTerminalOutcome(droppedItem.keyFrameId,
                              TerminalOutcome::QUEUE_DROPPED);
    }
}

} // namespace core
} // namespace vs_graphs
