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
 * @file            createProspectiveFarSideRoom.cc
 *
 * @brief           Implements SemanticsManager::createProspectiveFarSideRoom(),
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

SemanticsManagerStatus SemanticsManager::createProspectiveFarSideRoom(
    const std::vector<semantic::Room *>    &allRooms_in,
    const std::vector<semantic::Passage *> &allPassages_in,
    semantic::Passage                      *p_passage_inout,
    PassageRoomLinkState                   &linkState_inout)
{
    bool passageHasProspectiveRoom2{};
    bool knownSideHasDirection5{};
    if (!(linkState_inout.confirmedAssociatedRoomCount == 1) &&
        (linkState_inout.confirmedAssociatedRoomCount == 0) &&
        linkState_inout.knownSide.hasDirection(knownSideHasDirection5) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (((linkState_inout.confirmedAssociatedRoomCount == 1 ||
          (linkState_inout.confirmedAssociatedRoomCount == 0 &&
           knownSideHasDirection5))) &&
        p_passage_inout->hasProspectiveRoom(passageHasProspectiveRoom2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasProspectiveRoom returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool knownSideHasDirection6{};
    if (!(linkState_inout.confirmedAssociatedRoomCount == 1) &&
        (linkState_inout.confirmedAssociatedRoomCount == 0) &&
        linkState_inout.knownSide.hasDirection(knownSideHasDirection6) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if ((linkState_inout.confirmedAssociatedRoomCount == 1 ||
         (linkState_inout.confirmedAssociatedRoomCount == 0 &&
          knownSideHasDirection6)) &&
        !passageHasProspectiveRoom2)
    {
        /* Passage limit: don't create prospective if passage already has 2
         * rooms */
        if (linkState_inout.associatedRoomCount >= 2)
        {
            int passageId13{};
            if (p_passage_inout->getId(passageId13) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] Passage#" << passageId13
                      << " already has 2 associated rooms; skipping "
                         "prospective creation."
                      << std::endl;
        }
        else
        {
            g2o::Plane3D passageGlobalEquation2{};
            if (p_passage_inout->getGlobalEquation(passageGlobalEquation2) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getGlobalEquation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Vector4d passageEq  = passageGlobalEquation2.coeffs();
            const double    normalNorm = passageEq.head<3>().norm();

            if (std::isfinite(normalNorm) && normalNorm > 1e-8)
            {
                passageEq /= normalNorm;
                const Eigen::Vector3d passageNormal = passageEq.head<3>();
                Eigen::Vector3d       passageCentroid3{};
                if (p_passage_inout->getCentroid(passageCentroid3) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getCentroid returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                const Eigen::Vector3d passageCentroid =
                    passageCentroid3.cast<double>();

                /* Persisted provenance, not the current camera pose,
                 * defines the side opposite which the stable handle is
                 * created. */
                vs_graphs::core::semantic::Room *p_knownRoom =
                    linkState_inout.p_confirmedAssociatedRoom;
                bool knownSideHasDirection7{};
                if (linkState_inout.knownSide.hasDirection(
                        knownSideHasDirection7) !=
                    semantic::KnownSideProvenanceStatus::
                        KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                Eigen::Vector3d knownSideDirection =
                    knownSideHasDirection7
                        ? linkState_inout.knownSide.knownSideDirection_world
                        : Eigen::Vector3d::Zero();
                bool knownSideHasDirection8{};
                if (linkState_inout.knownSide.hasDirection(
                        knownSideHasDirection8) !=
                    semantic::KnownSideProvenanceStatus::
                        KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (!knownSideHasDirection8 && p_knownRoom != nullptr)
                {
                    Eigen::Vector3d knownRoomCentroid2{};
                    if (p_knownRoom->getCentroid(knownRoomCentroid2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getCentroid returned a failure "
                                     "status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }
                    const double knownRoomSide =
                        passageNormal.dot(knownRoomCentroid2) + passageEq(3);
                    knownSideDirection =
                        knownRoomSide < 0.0 ? -passageNormal : passageNormal;
                    if (p_passage_inout->setKnownSideDirection(
                            knownSideDirection) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                    "%s: setKnownSideDirection rejected "
                                    "its input; continuing as before.",
                                    __func__);
                    }
                    if (p_passage_inout->setKnownSideRoom(p_knownRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: setKnownSideRoom returned a "
                                     "failure status although it cannot "
                                     "fail; continuing as before.",
                                     __func__);
                    }
                }
                const Eigen::Vector3d farSideNormal = -knownSideDirection;

                /*!
                 * Anti-churn gate: a passage whose far side already holds a
                 * confirmed room must not create (and then immediately
                 * delete) a prospective placeholder for that same space.
                 * Matches the promotion search exactly, so nothing flips
                 * between created-this-cycle and promoted-next-cycle.
                 */
                bool farSideConfirmedRoomExists = false;
                vs_graphs::core::semantic::Room *p_existingFarSideRoom =
                    nullptr;
                if (findConfirmedFarSideRoom(allRooms_in,
                                             p_passage_inout,
                                             p_knownRoom,
                                             farSideConfirmedRoomExists,
                                             p_existingFarSideRoom) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: findConfirmedFarSideRoom returned a "
                                 "failure status although it "
                                 "cannot fail; continuing as before.",
                                 __func__);
                }

                if (farSideConfirmedRoomExists)
                {
                    if (p_existingFarSideRoom != nullptr)
                    {
                        if (p_existingFarSideRoom->addPassageAssociation(
                                p_passage_inout) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: addPassageAssociation returned a failure "
                                "status "
                                "although it cannot fail; continuing as "
                                "before.",
                                __func__);
                        }
                        if (p_passage_inout->setProspectiveRoom(
                                p_existingFarSideRoom) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            /*!
                             * setProspectiveRoom cannot fail; continue as
                             * before.
                             */
                        }

                        int passageId14{};
                        if (p_passage_inout->getId(passageId14) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getId returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        int existingFarSideRoomId{};
                        if (p_existingFarSideRoom->getId(
                                existingFarSideRoomId) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getId returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        std::cout << "[SemMgr] Passage#" << passageId14
                                  << " resolved directly to confirmed "
                                     "Room#"
                                  << existingFarSideRoomId
                                  << " on the far side." << std::endl;
                    }
                }
                else
                {
                    if (createProspectiveRoom(allPassages_in,
                                              p_passage_inout,
                                              linkState_inout,
                                              passageEq,
                                              passageCentroid,
                                              farSideNormal) !=
                        SemanticsManagerStatus::
                            SEMANTICS_MANAGER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: createProspectiveRoom returned a "
                                     "failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                }
            }
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
