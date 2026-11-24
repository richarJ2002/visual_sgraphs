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
 * @file            isSeenThroughPassage.cc
 *
 * @brief           Implements the passage visibility rule.
 */

#include "scene_understanding/semantic_constraints/visibility_rules/public_functions/public_functions.h"

#include <cstddef>
#include <vector>

#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace semantic_constraints
{

bool isSeenThroughPassage(const VisibilityRuleInput &visibilityInput_in,
                          const types::SystemParams &systemParams_in,
                          geometric::Plane         *&p_blockingWall_out)
{
    p_blockingWall_out = nullptr;
    const std::vector<Eigen::Vector3d> &observerViewpoints =
        visibilityInput_in.observerViewpoints_world_m;
    if (!systemParams_in.roomSeg.isSeeThroughGateEnabled ||
        (visibilityInput_in.isCameraViewpoint && observerViewpoints.empty()))
    {
        return false;
    }

    /* Walls of every valid passage, without the candidate and its twin. */
    std::vector<geometric::Plane *> passageWalls{};
    for (semantic::Passage *p_passage : *visibilityInput_in.p_mapPassages)
    {
        bool                            passageIsBad{true};
        std::vector<geometric::Plane *> associateWalls{};
        if (p_passage == nullptr ||
            p_passage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS ||
            passageIsBad ||
            p_passage->getAssociateWalls(associateWalls) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            continue;
        }
        for (geometric::Plane *p_passageWall : associateWalls)
        {
            if (p_passageWall != visibilityInput_in.p_candidateWall &&
                p_passageWall != visibilityInput_in.p_candidateTwinWall)
            {
                passageWalls.push_back(p_passageWall);
            }
        }
    }

    /* Without observers the line runs from the room centroid. */
    const std::vector<Eigen::Vector3d> sightLineStarts =
        observerViewpoints.empty()
            ? std::vector<Eigen::Vector3d>{visibilityInput_in
                                               .roomCentroid_world_m}
            : observerViewpoints;
    std::size_t blockedSightLineCount = 0U;
    for (const Eigen::Vector3d &sightLineStart_world_m : sightLineStarts)
    {
        if (hasSeparatingFiniteWall(
                passageWalls,
                sightLineStart_world_m,
                visibilityInput_in.candidateCentroid_world_m,
                static_cast<double>(systemParams_in.semSeg.reassociate
                                        .wallExtension.maximumInPlaneGap_m),
                true))
        {
            blockedSightLineCount++;
        }
    }
    return 2U * blockedSightLineCount > sightLineStarts.size();
}

} /* namespace semantic_constraints */
} /* namespace core */
} /* namespace vs_graphs */
