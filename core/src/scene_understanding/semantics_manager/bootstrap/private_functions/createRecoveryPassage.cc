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

/*!
 * @file            createRecoveryPassage.cc
 *
 * @brief           Implements createRecoveryPassage(), declared in
 *                  bootstrap/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

std::unique_ptr<semantic::Passage>
    createRecoveryPassage(atlas::Atlas                   *p_atlas_inout,
                          map::Map                       *p_activeMap_inout,
                          const semantic::PassageContext &passageContext_in)
{
    std::unique_ptr<semantic::Passage> p_passage =
        std::make_unique<semantic::Passage>();

    int passageId{};
    if (p_atlas_inout->reservePassageIdentity(passageId) !=
            atlas::AtlasStatus::ATLAS_STATUS_SUCCESS ||
        p_passage->setId(passageId) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: reservePassageIdentity/setId returned a failure "
                     "status although they cannot fail; continuing as before.",
                     __func__);
    }

    /* Position, orientation and aperture size are not knowable across a map
     * break, so only frame-free state is restored. A zero-sized aperture
     * shrinks geometric tests to their margin sliver. */
    if (p_passage->setMap(p_activeMap_inout) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage->setPassable(passageContext_in.isPassable) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage->setPassageType(semantic::Passage::PassageVariant::DOORWAY) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
        p_passage->setRecoveryProxy(true) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: a passage setter returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    /* The traversal history is replayed one observation at a time. */
    const std::size_t observationCounts[3] = {
        passageContext_in.traversalKnownToFarCount,
        passageContext_in.traversalFarToKnownCount,
        passageContext_in.traversalUnknownCount};
    const semantic::Passage::TraversalDirection directions[3] = {
        semantic::Passage::TraversalDirection::KNOWN_TO_FAR,
        semantic::Passage::TraversalDirection::FAR_TO_KNOWN,
        semantic::Passage::TraversalDirection::UNKNOWN};
    for (std::size_t directionIndex = 0U; directionIndex < 3U; directionIndex++)
    {
        for (std::size_t observationIndex = 0U;
             observationIndex < observationCounts[directionIndex];
             observationIndex++)
        {
            /* addTraversalObservation cannot fail; the status is not used. */
            static_cast<void>(
                p_passage->addTraversalObservation(directions[directionIndex]));
        }
    }
    return p_passage;
}

} /* namespace core */
} /* namespace vs_graphs */
