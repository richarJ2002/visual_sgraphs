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

void SemanticSegmentation::recordTerminalOutcome(std::uint64_t   keyFrameId_in,
                                                 TerminalOutcome outcome_in)
{
    terminalCount.fetch_add(1U, std::memory_order_relaxed);
    lastTerminalKeyFrameId.store(keyFrameId_in, std::memory_order_relaxed);
    switch (outcome_in)
    {
    case TerminalOutcome::ACCEPTED:
        acceptedCount.fetch_add(1U, std::memory_order_relaxed);
        break;
    case TerminalOutcome::QUEUE_DROPPED:
        droppedCount.fetch_add(1U, std::memory_order_relaxed);
        break;
    case TerminalOutcome::MISSING_KEYFRAME:
        missingKeyFrameCount.fetch_add(1U, std::memory_order_relaxed);
        break;
    case TerminalOutcome::MISSING_CLOUD:
        missingCloudCount.fetch_add(1U, std::memory_order_relaxed);
        break;
    case TerminalOutcome::STALE_MAP:
        staleMapCount.fetch_add(1U, std::memory_order_relaxed);
        break;
    }
}

} // namespace core
} // namespace vs_graphs
