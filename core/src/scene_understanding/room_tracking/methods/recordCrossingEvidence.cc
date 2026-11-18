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
 * @file            recordCrossingEvidence.cc
 *
 * @brief           Implements SemanticsManager::recordCrossingEvidence(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include <functional>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus
    SemanticsManager::recordCrossingEvidence(semantic::Passage *p_passage_inout,
                                             bool addedTraversal_in,
                                             bool wasSettled_in)
{
    /* Only newly accepted segment evidence is a new tracker event.
     * The passage owns segment deduplication, so replayed history
     * must not republish crossing evidence. */
    /* A new crossing proves a blocked passage open again. */
    bool passageIsBlocked{};
    if (addedTraversal_in &&
        p_passage_inout->isBlocked(passageIsBlocked) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBlocked returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (addedTraversal_in && passageIsBlocked)
    {
        int blockedPassageId{};
        if (p_passage_inout->setBlocked(false) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_passage_inout->setVerdictCounters(0U, 0U, 0) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            p_passage_inout->getId(blockedPassageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBlocked/setVerdictCounters/getId "
                         "returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] Passage#" << blockedPassageId
                  << " unblocked: crossed again." << std::endl;
    }

    if (addedTraversal_in)
    {
        std::lock_guard<std::mutex> currentRoomLock(currentRoomMutex);
        isCrossingEventPending = true;
        bool passageHasBidirectionalTraversalEvidence{};
        if (!(isCrossingBothSidesPending) &&
            p_passage_inout->hasBidirectionalTraversalEvidence(
                passageHasBidirectionalTraversalEvidence) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasBidirectionalTraversalEvidence "
                         "returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        isCrossingBothSidesPending = isCrossingBothSidesPending ||
                                     passageHasBidirectionalTraversalEvidence;
        /* Test seam: empty outside tests (SemanticsManager.h). */
        std::function<void()> publishHook =
            std::move(roomTrackerPendingPublishHook);
        if (publishHook)
        {
            publishHook();
        }
    }

    if (addedTraversal_in && !wasSettled_in)
    {
        int passageId2{};
        if (p_passage_inout->getId(passageId2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[SemMgr] Passage#" << passageId2
                  << " traversed (traversal evidence settled)." << std::endl;
    }

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
