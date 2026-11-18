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
 * @file            findPromotionFarSideRoom.cc
 *
 * @brief           Implements SemanticsManager::findPromotionFarSideRoom(),
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

SemanticsManagerStatus SemanticsManager::findPromotionFarSideRoom(
    const std::vector<semantic::Room *> &allRooms_in,
    semantic::Passage                   *p_passage_inout,
    semantic::Room                      *p_prospectiveRoom_in,
    semantic::Room                     *&p_farSideConfirmedRoom_out)
{
    p_farSideConfirmedRoom_out = nullptr;

    semantic::Passage::KnownSideProvenance passageKnownSideProvenance3{};
    if (p_passage_inout->getKnownSideProvenance(passageKnownSideProvenance3) !=
        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getKnownSideProvenance returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    vs_graphs::core::semantic::Room *p_knownSideRoom =
        passageKnownSideProvenance3.p_room;

    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    bool groundPlaneIsBad{};
    if ((p_groundPlane != nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (p_groundPlane != nullptr && !groundPlaneIsBad)
    {
        g2o::Plane3D groundPlaneGetGlobalEquation{};
        if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector4d groundEquation_world =
            groundPlaneGetGlobalEquation.coeffs();
        const double groundNormalNorm = groundEquation_world.head<3>().norm();

        if (groundEquation_world.allFinite() && groundNormalNorm > 1e-8)
        {
            const Eigen::Vector3d groundNormal_world =
                groundEquation_world.head<3>() / groundNormalNorm;
            const Eigen::Vector3d groundAxisU_world =
                groundNormal_world.unitOrthogonal().normalized();
            const Eigen::Vector3d groundAxisV_world =
                groundNormal_world.cross(groundAxisU_world).normalized();
            const types::SystemParams::RoomSeg::PassagePartition
                &partitionParameters = p_sysParams->roomSeg.passagePartition;
            const double openingMargin_m =
                static_cast<double>(partitionParameters.openingMargin_m);
            const double minimumSideDistance_m =
                static_cast<double>(partitionParameters.minimumSideDistance_m);
            const types::SystemParams::RoomSeg::BoundaryTopology
                &topologyParameters = p_sysParams->roomSeg.boundaryTopology;
            /* The candidate must lie beyond THIS opening: the
             * line to it starts just inside the known side, the
             * passage centroid pushed twice the side band along
             * the known-side direction (the stand-in
             * enforcePassageApertureBackstop() uses). The
             * prospective room's own centroid sits on the far
             * side, so its line through the opening reached the
             * known side instead: across a corridor into the
             * office opposite. Without a known side there is no
             * start, and the prospective room waits. */
            bool knownSideHasDirection4{};
            if (passageKnownSideProvenance3.hasDirection(
                    knownSideHasDirection4) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasDirection returned a failure "
                             "status although it cannot fail; "
                             "continuing as before.",
                             __func__);
            }
            Eigen::Vector3d passageCentroid4{};
            if (p_passage_inout->getCentroid(passageCentroid4) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure "
                             "status although it cannot fail; "
                             "continuing as before.",
                             __func__);
            }
            const Eigen::Vector3d knownSideStart_world_m =
                passageCentroid4 +
                2.0 * minimumSideDistance_m *
                    passageKnownSideProvenance3.knownSideDirection_world;
            const std::vector<vs_graphs::core::semantic::Room *> excludedRooms =
                {p_prospectiveRoom_in, p_knownSideRoom};

            for (vs_graphs::core::semantic::Room *p_otherRoom :
                 knownSideHasDirection4 ? allRooms_in
                                        : std::vector<semantic::Room *>{})
            {
                bool otherRoomIsBad2{};
                if (!(p_otherRoom == nullptr) &&
                    p_otherRoom->isBad(otherRoomIsBad2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure "
                                 "status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                semantic::Room::RoomVariant otherRoomRoomVariant2{};
                if (!(p_otherRoom == nullptr || otherRoomIsBad2 ||
                      p_otherRoom == p_prospectiveRoom_in ||
                      p_otherRoom == p_knownSideRoom) &&
                    p_otherRoom->getRoomVariant(otherRoomRoomVariant2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    /*!
                     * getRoomVariant cannot fail; continue as
                     * before.
                     */
                }
                if (p_otherRoom == nullptr || otherRoomIsBad2 ||
                    p_otherRoom == p_prospectiveRoom_in ||
                    p_otherRoom == p_knownSideRoom ||
                    otherRoomRoomVariant2 ==
                        vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
                {
                    continue;
                }

                Eigen::Vector3d otherRoomCentroid3{};
                if (p_otherRoom->getCentroid(otherRoomCentroid3) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getCentroid returned a failure status "
                                 "although it cannot fail; continuing as "
                                 "before.",
                                 __func__);
                }
                bool crossesPassageOpening2{};
                if (segmentCrossesPassageOpening(knownSideStart_world_m,
                                                 otherRoomCentroid3,
                                                 p_passage_inout,
                                                 groundNormal_world,
                                                 openingMargin_m,
                                                 minimumSideDistance_m,
                                                 crossesPassageOpening2) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: segmentCrossesPassageOpening returned "
                                 "a failure status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                if (!crossesPassageOpening2)
                {
                    continue;
                }

                /*!
                 * A candidate that threads this passage's own
                 * aperture is still not a legitimate match when
                 * the straight line to it is blocked by another
                 * room's own wall -- see segmentCrossesForeignWall
                 * above. Observed directly: a prospective
                 * placeholder's heuristic position resolved
                 * straight to a confirmed room three doors down
                 * the same corridor, with the true intervening
                 * room's own wall sitting directly on that line.
                 */
                Eigen::Vector3d otherRoomCentroid4{};
                if (p_otherRoom->getCentroid(otherRoomCentroid4) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getCentroid returned a failure status "
                                 "although it cannot fail; continuing as "
                                 "before.",
                                 __func__);
                }
                bool crossesForeignWall2{};
                if (segmentCrossesForeignWall(
                        knownSideStart_world_m,
                        otherRoomCentroid4,
                        excludedRooms,
                        allRooms_in,
                        groundAxisU_world,
                        groundAxisV_world,
                        groundNormal_world,
                        topologyParameters.endpointTrimRatio,
                        topologyParameters.minimumWallLength_m,
                        crossesForeignWall2) !=
                    SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: segmentCrossesForeignWall returned a "
                                 "failure status although it cannot fail; "
                                 "continuing as before.",
                                 __func__);
                }
                if (crossesForeignWall2)
                {
                    continue;
                }

                p_farSideConfirmedRoom_out = p_otherRoom;
                break;
            }
        }
    }
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
