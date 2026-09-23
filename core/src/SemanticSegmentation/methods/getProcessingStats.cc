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

SemanticSegmentation::ProcessingStats SemanticSegmentation::getProcessingStats()
{
    ProcessingStats stats;
    stats.enqueuedCount = enqueuedCount.load(std::memory_order_relaxed);
    stats.dequeuedCount = dequeuedCount.load(std::memory_order_relaxed);
    stats.terminalCount = terminalCount.load(std::memory_order_relaxed);
    stats.acceptedCount = acceptedCount.load(std::memory_order_relaxed);
    stats.droppedCount  = droppedCount.load(std::memory_order_relaxed);
    stats.missingKeyFrameCount =
        missingKeyFrameCount.load(std::memory_order_relaxed);
    stats.missingCloudCount = missingCloudCount.load(std::memory_order_relaxed);
    stats.staleMapCount     = staleMapCount.load(std::memory_order_relaxed);
    stats.lastTerminalKeyFrameId =
        lastTerminalKeyFrameId.load(std::memory_order_relaxed);
    stats.queueHighWatermark =
        queueHighWatermark.load(std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(mMutexNewKFs);
        stats.queueDepth =
            static_cast<std::uint32_t>(segmentedImageBuffer.size());
    }
    return stats;
}

} // namespace core
} // namespace vs_graphs
