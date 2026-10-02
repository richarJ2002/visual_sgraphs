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
 * @file            mergeKnownSideProvenance.cc
 *
 * @brief           Implements Passage::mergeKnownSideProvenance(), declared in
 *                  Semantic/Passage.h.
 */

#include "Semantic/Passage.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

PassageStatus
    Passage::mergeKnownSideProvenance(const KnownSideProvenance &provenance_in)
{
    std::lock_guard<std::mutex> lock(geometryMutex);
    bool                        knownSideProvenanceHasDirection{};
    if (knownSideProvenance.hasDirection(knownSideProvenanceHasDirection) !=
        KnownSideProvenanceStatus::KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool provenanceHasDirection{};
    if ((!knownSideProvenanceHasDirection) &&
        provenance_in.hasDirection(provenanceHasDirection) !=
            KnownSideProvenanceStatus::KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!knownSideProvenanceHasDirection && provenanceHasDirection)
    {
        knownSideProvenance.direction_world = provenance_in.direction_world;
    }
    if (knownSideProvenance.p_room == nullptr)
    {
        knownSideProvenance.p_room = provenance_in.p_room;
    }

    return PassageStatus::PASSAGE_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
