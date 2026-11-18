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
 * @file            findFarSideProspectiveRoom.cc
 *
 * @brief           Implements SemanticsManager::findFarSideProspectiveRoom(),
 * declared in semantics_manager/objects/SemanticsManagerClass.h.
 */

#include "scene_understanding/semantics_manager/objects/SemanticsManagerClass.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/logging.hpp>

#include "execution/profiling/profiling.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "mapping/geometric/Plane/Plane.h"
#include "mapping/geometric/Plane/objects/PlaneStatusEnum.h"
#include "mapping/geometry_helpers/objects/UtilsClass.h"
#include "mapping/geometry_helpers/objects/UtilsStatusEnum.h"
#include "mapping/geometry_helpers/utils.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/passage/objects/PassageStatusEnum.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "scene_understanding/scene_graph/room/objects/RoomStatusEnum.h"
#include "scene_understanding/semantic/value_order/value_order.h"
#include "scene_understanding/semantic_constraints/visibility_rules/visibility_rules.h"
#include "scene_understanding/semantics_manager/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::findFarSideProspectiveRoom(
    const std::vector<geometric::Plane *> &allPlanes_in,
    const Eigen::Vector3d                 &clusterCentroid_in,
    const std::vector<geometric::Plane *> &closestWalls_in,
    const std::unordered_set<int>         &matchedRoomIds_in,
    FarSideProspectiveMatch               &prospectiveMatch_out)
{
    /*
     * Match external far-side evidence before admitting any new walls.
     * Taking the wall snapshot here prevents admission by this cluster
     * from manufacturing its own promotion evidence.
     */
    vs_graphs::core::semantic::Room    *p_clusterProspective = nullptr;
    vs_graphs::core::semantic::Passage *p_clusterPassage     = nullptr;
    std::vector<vs_graphs::core::geometric::Plane *>
           prospectiveWallsBeforeCluster;
    double bestProspectiveDistance_m = std::numeric_limits<double>::infinity();

    std::vector<vs_graphs::core::semantic::Passage *> atlasAllPassages{};
    if (getAllPassagesInIdOrder(atlasAllPassages) !=
        SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassagesInIdOrder returned a failure "
                     "status; continuing as before.",
                     __func__);
    }
    for (vs_graphs::core::semantic::Passage *p_passage : atlasAllPassages)
    {
        if (p_passage == nullptr)
        {
            continue;
        }

        vs_graphs::core::semantic::Room *p_prospective = nullptr;
        if (p_passage->getProspectiveRoom(p_prospective) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        bool prospectiveIsBad{};
        if (!(p_prospective == nullptr) &&
            p_prospective->isBad(prospectiveIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant prospectiveRoomVariant{};
        if (!(p_prospective == nullptr || prospectiveIsBad) &&
            p_prospective->getRoomVariant(prospectiveRoomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        int prospectiveId{};
        if (!(p_prospective == nullptr || prospectiveIsBad ||
              prospectiveRoomVariant !=
                  vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED) &&
            p_prospective->getId(prospectiveId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_prospective == nullptr || prospectiveIsBad ||
            prospectiveRoomVariant !=
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED ||
            matchedRoomIds_in.count(prospectiveId) > 0U)
        {
            continue;
        }

        g2o::Plane3D passageGlobalEquation{};
        if (p_passage->getGlobalEquation(passageGlobalEquation) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d passageEquation_world = passageGlobalEquation.coeffs();
        const double passageNormalNorm = passageEquation_world.head<3>().norm();

        if (!passageEquation_world.allFinite() || passageNormalNorm < 1e-8)
        {
            continue;
        }

        passageEquation_world /= passageNormalNorm;

        Eigen::Vector3d prospectiveCentroid_world_m{};
        if (p_prospective->getCentroid(prospectiveCentroid_world_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const double prospectiveSide_m =
            passageEquation_world.head<3>().dot(prospectiveCentroid_world_m) +
            passageEquation_world(3);
        const double clusterSide_m =
            passageEquation_world.head<3>().dot(clusterCentroid_in) +
            passageEquation_world(3);
        const double centroidDistance_m =
            (prospectiveCentroid_world_m - clusterCentroid_in).norm();

        /*!
         * The far side is the one opposite the passage's persisted
         * known-side direction, the same direction the prospective room
         * was created from. The seed's own position cannot decide it: it
         * is placed only 0.15 m beyond the passage, inside the 0.20 m
         * band that does not resolve a side. Only a passage without a
         * known-side direction falls back to the prospective centroid.
         */
        semantic::Passage::KnownSideProvenance knownSide{};
        if (p_passage->getKnownSideProvenance(knownSide) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getKnownSideProvenance returned a failure "
                         "status although it cannot fail; continuing as "
                         "before.",
                         __func__);
        }
        bool hasKnownSideDirection{};
        if (knownSide.hasDirection(hasKnownSideDirection) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const double knownSideAlongNormal =
            hasKnownSideDirection ? passageEquation_world.head<3>().dot(
                                        knownSide.knownSideDirection_world)
                                  : 0.0;
        double farSideSign = 0.0;
        if (std::abs(knownSideAlongNormal) > 1e-3)
        {
            farSideSign = knownSideAlongNormal < 0.0 ? 1.0 : -1.0;
        }
        else if (std::abs(prospectiveSide_m) > 0.20)
        {
            farSideSign = prospectiveSide_m > 0.0 ? 1.0 : -1.0;
        }

        if (!prospectiveCentroid_world_m.allFinite() || farSideSign == 0.0 ||
            farSideSign * clusterSide_m <= 0.20 ||
            centroidDistance_m > kProspectiveDedupDistance_m)
        {
            continue;
        }

        std::vector<vs_graphs::core::geometric::Plane *> prospectiveWalls{};
        if (p_prospective->getWalls(prospectiveWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getWalls returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        bool sharesWallEvidence = false;
        for (vs_graphs::core::geometric::Plane *p_prospectiveWall :
             prospectiveWalls)
        {
            bool prospectiveWallIsBad{};
            if ((p_prospectiveWall != nullptr) &&
                p_prospectiveWall->isBad(prospectiveWallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_prospectiveWall != nullptr && !prospectiveWallIsBad &&
                std::find(closestWalls_in.begin(),
                          closestWalls_in.end(),
                          p_prospectiveWall) != closestWalls_in.end())
            {
                sharesWallEvidence = true;
                break;
            }
        }

        /* A wall-less prospective has no wall to share; the cluster's
         * far-side position above and the absence of a finite wall
         * between seed and cluster (the associateRooms veto) are its
         * whole evidence. */
        bool hasProspectiveWall = false;
        for (vs_graphs::core::geometric::Plane *p_prospectiveWall :
             prospectiveWalls)
        {
            bool prospectiveWallIsBad{};
            if ((p_prospectiveWall != nullptr) &&
                p_prospectiveWall->isBad(prospectiveWallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_prospectiveWall != nullptr && !prospectiveWallIsBad)
            {
                hasProspectiveWall = true;
                break;
            }
        }
        const bool isSeedSeparatedFromCluster =
            !hasProspectiveWall &&
            semantic_constraints::hasSeparatingFiniteWall(
                allPlanes_in,
                prospectiveCentroid_world_m,
                clusterCentroid_in,
                static_cast<double>(
                    p_sysParams->roomSeg.finiteWallBoundsMargin_m),
                false);

        if ((hasProspectiveWall && !sharesWallEvidence) ||
            isSeedSeparatedFromCluster ||
            centroidDistance_m >= bestProspectiveDistance_m)
        {
            continue;
        }

        p_clusterProspective          = p_prospective;
        p_clusterPassage              = p_passage;
        prospectiveWallsBeforeCluster = prospectiveWalls;
        bestProspectiveDistance_m     = centroidDistance_m;
    }

    prospectiveMatch_out.p_prospectiveRoom = p_clusterProspective;
    prospectiveMatch_out.p_passage         = p_clusterPassage;
    prospectiveMatch_out.prospectiveWallsBeforeCluster =
        prospectiveWallsBeforeCluster;

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
