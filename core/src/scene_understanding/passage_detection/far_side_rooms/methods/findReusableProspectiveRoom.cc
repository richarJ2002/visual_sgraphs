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
 * @file            findReusableProspectiveRoom.cc
 *
 * @brief           Implements SemanticsManager::findReusableProspectiveRoom(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "configuration/types/types.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/floor/objects/FloorStatusEnum.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/KnownSideProvenanceStatusEnum.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantics_manager/semantics_manager.h"

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "scene_understanding/geo_sem_helpers/objects/GeoSemHelpersStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <rclcpp/logging.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::findReusableProspectiveRoom(
    const std::vector<semantic::Room *>    &candidateRooms_in,
    const std::vector<semantic::Passage *> &allPassages_in,
    semantic::Passage                      *p_passage_inout,
    const Eigen::Vector4d                  &passageEquation_world_in,
    const Eigen::Vector3d                  &passageCentroid_world_m_in,
    const Eigen::Vector3d                  &prospectiveCentroid_world_m_in,
    bool                                   &prospectiveExists_inout)
{
    for (vs_graphs::core::semantic::Room *p_candidate : candidateRooms_in)
    {
        bool candidateIsBad2{};
        if (!(p_candidate == nullptr) &&
            p_candidate->isBad(candidateIsBad2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status "
                         "although it cannot fail; continuing "
                         "as before.",
                         __func__);
        }
        semantic::Room::RoomVariant candidateRoomVariant2{};
        if (!(p_candidate == nullptr || candidateIsBad2) &&
            p_candidate->getRoomVariant(candidateRoomVariant2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            /*!
             * getRoomVariant cannot fail; continue as
             * before.
             */
        }
        if (p_candidate == nullptr || candidateIsBad2 ||
            candidateRoomVariant2 !=
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            continue;
        }
        Eigen::Vector3d candidateCentroid{};
        if (p_candidate->getCentroid(candidateCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            /*!
             * getCentroid cannot fail; continue as
             * before.
             */
        }
        const double distance =
            (candidateCentroid - prospectiveCentroid_world_m_in).norm();

        bool passageIdentityMatches = false;
        if (distance <= kProspectiveDedupDistance_m)
        {
            for (semantic::Passage *p_candidatePassage : allPassages_in)
            {
                vs_graphs::core::semantic::Room
                    *p_candidatePassageProspectiveRoom = nullptr;
                if (!(p_candidatePassage == nullptr ||
                      p_candidatePassage == p_passage_inout) &&
                    p_candidatePassage->getProspectiveRoom(
                        p_candidatePassageProspectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getProspectiveRoom "
                                 "returned a failure status "
                                 "although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                if (p_candidatePassage == nullptr ||
                    p_candidatePassage == p_passage_inout ||
                    p_candidatePassageProspectiveRoom != p_candidate)
                {
                    continue;
                }

                g2o::Plane3D candidatePassageGlobalEquation{};
                if (p_candidatePassage->getGlobalEquation(
                        candidatePassageGlobalEquation) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getGlobalEquation "
                                 "returned a failure status "
                                 "although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                Eigen::Vector4d candidatePassageEq =
                    candidatePassageGlobalEquation.coeffs();
                const double candidatePassageNorm =
                    candidatePassageEq.head<3>().norm();
                if (!candidatePassageEq.allFinite() ||
                    candidatePassageNorm < 1e-8)
                {
                    continue;
                }
                candidatePassageEq /= candidatePassageNorm;

                Eigen::Vector3d candidatePassageCentroid{};
                if (p_candidatePassage->getCentroid(candidatePassageCentroid) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    /*!
                     * getCentroid cannot fail; continue
                     * as before.
                     */
                }
                const double openingDistance_m =
                    (candidatePassageCentroid - passageCentroid_world_m_in)
                        .norm();
                const double normalAlignment =
                    std::abs(candidatePassageEq.head<3>().dot(
                        passageEquation_world_in.head<3>()));
                Eigen::Vector3d candidatePassageCentroid2{};
                if (p_candidatePassage->getCentroid(
                        candidatePassageCentroid2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    /*!
                     * getCentroid cannot fail; continue
                     * as before.
                     */
                }
                const double planeResidual_m =
                    std::abs(passageEquation_world_in.head<3>().dot(
                                 candidatePassageCentroid2) +
                             passageEquation_world_in(3));

                if (openingDistance_m <= p_sysParams->semSeg.passageDetection
                                             .duplicatePassageDistance_m &&
                    normalAlignment >= p_sysParams->semSeg.passageDetection
                                           .duplicateNormalAlignment &&
                    planeResidual_m <= 0.30)
                {
                    passageIdentityMatches = true;
                    break;
                }
            }
        }

        if (passageIdentityMatches)
        {
            /* Cross-passage reuse requires equivalent
             * supporting-plane/opening geometry. */
            if (p_passage_inout->setProspectiveRoom(p_candidate) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a "
                             "failure status although it cannot "
                             "fail; continuing as before.",
                             __func__);
            }
            if (p_candidate->addPassageAssociation(p_passage_inout) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                /*!
                 * addPassageAssociation cannot fail; continue as
                 * before.
                 */
            }
            prospectiveExists_inout = true;
            int candidateId{};
            if (p_candidate->getId(candidateId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                /*!
                 * getId cannot fail; continue as
                 * before.
                 */
            }
            int passageId16{};
            if (p_passage_inout->getId(passageId16) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                /*!
                 * getId cannot fail; continue as
                 * before.
                 */
            }
            std::cout << "[SemMgr] Reusing existing "
                         "prospective Room#"
                      << candidateId << " (dist=" << distance
                      << "m) for Passage#" << passageId16 << std::endl;
            break;
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
