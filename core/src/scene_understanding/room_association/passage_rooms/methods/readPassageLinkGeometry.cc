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
 * @file            readPassageLinkGeometry.cc
 *
 * @brief           Implements SemanticsManager::readPassageLinkGeometry(),
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

SemanticsManagerStatus SemanticsManager::readPassageLinkGeometry(
    semantic::Passage    *p_passage_inout,
    PassageRoomLinkState &linkState_inout)
{
    /* Skip invalid passages */
    bool passageIsBad{};
    if (!(p_passage_inout == nullptr) &&
        p_passage_inout->isBad(passageIsBad) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_passage_inout == nullptr || passageIsBad)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* A recovery proxy carries only stable identity and topology. Its
     * historical coordinates deliberately are not copied into the new
     * map frame. Preserve its reciprocal room edge until map alignment
     * can reconcile it with newly observed passage geometry. */
    bool passageIsRecoveryProxy{};
    if (p_passage_inout->isRecoveryProxy(passageIsRecoveryProxy) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isRecoveryProxy returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (passageIsRecoveryProxy)
    {
        if (p_passage_inout->getKnownSideProvenance(
                linkState_inout.knownSide) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getKnownSideProvenance returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room *p_farSideRoom = nullptr;
        if (p_passage_inout->getProspectiveRoom(p_farSideRoom) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool isBad2{};
        if ((linkState_inout.knownSide.p_room != nullptr) &&
            linkState_inout.knownSide.p_room->isBad(isBad2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (linkState_inout.knownSide.p_room != nullptr && !isBad2)
        {
            if (linkState_inout.knownSide.p_room->addPassageAssociation(
                    p_passage_inout) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addPassageAssociation returned a failure "
                             "status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
        }
        bool farSideRoomIsBad{};
        if ((p_farSideRoom != nullptr) &&
            p_farSideRoom->isBad(farSideRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_farSideRoom != nullptr && !farSideRoomIsBad)
        {
            if (p_farSideRoom->addPassageAssociation(p_passage_inout) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addPassageAssociation returned a failure "
                             "status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
        }
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract the wall or walls supporting the passage */
    if (p_passage_inout->getAssociateWalls(linkState_inout.supportingWalls) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAssociateWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    /* A passage without a supporting wall cannot connect rooms */
    if (linkState_inout.supportingWalls.empty())
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    /* Extract and normalize the passage plane equation */
    g2o::Plane3D passageGlobalEquation{};
    if (p_passage_inout->getGlobalEquation(passageGlobalEquation) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    linkState_inout.passageEquation_world = passageGlobalEquation.coeffs();

    const double passageNormalNorm =
        linkState_inout.passageEquation_world.head<3>().norm();

    if (!std::isfinite(passageNormalNorm) || passageNormalNorm < 1e-8)
    {
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    linkState_inout.passageEquation_world /= passageNormalNorm;

    /* Extract the passage centroid in double precision */
    Eigen::Vector3d passageCentroid2{};
    if (p_passage_inout->getCentroid(passageCentroid2) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCentroid returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    linkState_inout.passageCentroid_world_m = passageCentroid2.cast<double>();

    /*!
     * A wall the passage is not linked to borders it only where that
     * wall's support reaches the opening's own frame: half the opening
     * width plus the jamb tolerance (passage_partition.opening_margin)
     * from the opening centre. Along a corridor, doors share one wall
     * line, so any wider reach lets one room's face claim the doors of
     * its neighbours.
     */
    if (p_passage_inout->getWidth(linkState_inout.passageWidth_m) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getWidth returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    linkState_inout.openingFrameReach_m =
        0.5 * linkState_inout.passageWidth_m +
        static_cast<double>(
            p_sysParams->roomSeg.passagePartition.openingMargin_m);

    if (p_passage_inout->getKnownSideProvenance(linkState_inout.knownSide) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    bool knownSideHasDirection{};
    if (linkState_inout.knownSide.hasDirection(knownSideHasDirection) !=
        semantic::KnownSideProvenanceStatus::
            KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: hasDirection returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!knownSideHasDirection)
    {
        /* Which side the passage was seen from is a property of the
         * observation that produced its supporting wall face, so take it
         * from that face's stamped observation origin
         * (Plane::getObservationOrigin_world()). Deriving it instead from
         * a median over the wall's whole observation history would
         * migrate to the far side once the UAV flew through this very
         * passage -- inverting the passage's own notion of which side it
         * was discovered from. The median remains only as a fallback for
         * faces created before the stamp existed. */
        for (geometric::Plane *p_supportingWall :
             linkState_inout.supportingWalls)
        {
            bool supportingWallIsBad{};
            if (!(p_supportingWall == nullptr) &&
                p_supportingWall->isBad(supportingWallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_supportingWall == nullptr || supportingWallIsBad)
            {
                return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
            }

            std::optional<Eigen::Vector3d> observationOrigin_world_m{};
            if (p_supportingWall->getObservationOrigin_world(
                    observationOrigin_world_m) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                /*!
                 * getObservationOrigin_world cannot fail; continue as
                 * before.
                 */
            }

            std::optional<double> observedSide_m;

            if (observationOrigin_world_m.has_value() &&
                observationOrigin_world_m->allFinite())
            {
                observedSide_m =
                    linkState_inout.passageEquation_world.head<3>().dot(
                        observationOrigin_world_m.value()) +
                    linkState_inout.passageEquation_world(3);
            }
            else
            {
                geometric::Plane::ObservationSideSnapshot sideSnapshot{};
                if (p_supportingWall->getObservationSideSnapshot(
                        linkState_inout.passageEquation_world,
                        sideSnapshot) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    /*!
                     * getObservationSideSnapshot cannot fail; continue as
                     * before.
                     */
                }
                observedSide_m = sideSnapshot.medianSignedDistance_m;
            }

            if (!observedSide_m.has_value() ||
                !std::isfinite(observedSide_m.value()))
            {
                return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
            }

            if (p_passage_inout->setKnownSideDirection(
                    observedSide_m.value() > 0.0
                        ? Eigen::Vector3d(
                              linkState_inout.passageEquation_world.head<3>())
                        : Eigen::Vector3d(-linkState_inout.passageEquation_world
                                               .head<3>())) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: setKnownSideDirection rejected its input; "
                            "continuing as before.",
                            __func__);
            }
            semantic::Passage::KnownSideProvenance passageKnownSideProvenance{};
            if (p_passage_inout->getKnownSideProvenance(
                    passageKnownSideProvenance) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            linkState_inout.knownSide = passageKnownSideProvenance;
            break;
        }
    }

    linkState_inout.isPassageLinkable = true;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
