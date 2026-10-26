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
 * @file            getVerdictCounters.cc
 *
 * @brief           Implements Passage::getVerdictCounters(), declared in
 *                  PassageClass.h.
 */

#include "scene_understanding/semantic/passage/objects/PassageClass.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageStatus
    Passage::getVerdictCounters(std::size_t  &consecutiveOpenVerdicts_out,
                                std::size_t  &consecutiveClosedVerdicts_out,
                                std::int64_t &lastVerdictStamp_ns_out) const
{
    std::lock_guard<std::mutex> lock(typeMutex);
    consecutiveOpenVerdicts_out   = consecutiveOpenVerdicts;
    consecutiveClosedVerdicts_out = consecutiveClosedVerdicts;
    lastVerdictStamp_ns_out       = lastVerdictStamp_ns;

    return PassageStatus::PASSAGE_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
