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

#include "SemanticsManager.h"

#include "../private_functions.h"
#include "GeoSemHelpers.h"
#include "GeoSemHelpersStatus.h"
#include "Semantic/ValueOrder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::associatePassagesToRooms(void)
{
    /* Extract all rooms from the current map */
    std::vector<vs_graphs::core::semantic::Room *> allRooms{};
    if (p_atlas->getAllRooms(allRooms) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Extract all passages from the current map */
    std::vector<vs_graphs::core::semantic::Passage *> allPassages{};
    if (p_atlas->getAllPassages(allPassages) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    constexpr double sideEpsilon_m = 0.20;

    constexpr double maximumSupportingPlaneDistance_m = 1.00;
    constexpr double maximumOpeningEdgeDistance_m     = 3.00;
    constexpr double minimumNormalAlignment           = 0.80;

    /* Stable room ordering makes equal-distance passage associations
     * repeatable. */
    std::sort(allRooms.begin(),
              allRooms.end(),
              semantic::isEntityIdLess<semantic::Room>);

    /*!
     * Snapshot the previous topology before rebuilding it. The semantic pass
     * runs periodically, so logging every unchanged edge as newly associated
     * would obscure genuine topology changes.
     */
    std::unordered_map<vs_graphs::core::semantic::Room *,
                       std::unordered_set<int>>
        previousPassageIdsByRoom;

    /* Rebuild the topology so stale associations cannot survive a remerge. */
    for (vs_graphs::core::semantic::Room *p_room : allRooms)
    {
        bool roomIsBad{};
        if ((p_room != nullptr) &&
            p_room->isBad(roomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room != nullptr && !roomIsBad)
        {
            std::unordered_set<int> &previousPassageIds =
                previousPassageIdsByRoom[p_room];

            std::vector<vs_graphs::core::semantic::Passage *> roomPassages2{};
            if (p_room->getPassages(roomPassages2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            for (vs_graphs::core::semantic::Passage *p_previousPassage :
                 roomPassages2)
            {
                if (p_previousPassage != nullptr)
                {
                    int previousPassageId{};
                    if (p_previousPassage->getId(previousPassageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    previousPassageIds.insert(previousPassageId);
                }
            }

            if (p_room->clearPassages() !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: clearPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    /* Iterate through every passage */
    for (vs_graphs::core::semantic::Passage *p_passage : allPassages)
    {
        /* Skip invalid passages */
        bool passageIsBad{};
        if (!(p_passage == nullptr) &&
            p_passage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_passage == nullptr || passageIsBad)
        {
            continue;
        }

        /* A recovery proxy carries only stable identity and topology. Its
         * historical coordinates deliberately are not copied into the new
         * map frame. Preserve its reciprocal room edge until map alignment
         * can reconcile it with newly observed passage geometry. */
        bool passageIsRecoveryProxy{};
        if (p_passage->isRecoveryProxy(passageIsRecoveryProxy) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isRecoveryProxy returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (passageIsRecoveryProxy)
        {
            semantic::Passage::KnownSideProvenance knownSide{};
            if (p_passage->getKnownSideProvenance(knownSide) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getKnownSideProvenance returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            semantic::Room *p_farSideRoom = nullptr;
            if (p_passage->getProspectiveRoom(p_farSideRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool isBad2{};
            if ((knownSide.p_room != nullptr) &&
                knownSide.p_room->isBad(isBad2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (knownSide.p_room != nullptr && !isBad2)
            {
                if (knownSide.p_room->setDoorways(p_passage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setDoorways returned a failure status although it "
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
                if (p_farSideRoom->setDoorways(p_passage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setDoorways returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
            }
            int passageId{};
            if (p_passage->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            passageZeroRoomCycles.erase(passageId);
            continue;
        }

        /* Extract the wall or walls supporting the passage */
        std::vector<vs_graphs::core::geometric::Plane *> supportingWalls{};
        if (p_passage->getAssociateWalls(supportingWalls) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAssociateWalls returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* A passage without a supporting wall cannot connect rooms */
        if (supportingWalls.empty())
        {
            continue;
        }

        /* Extract and normalize the passage plane equation */
        g2o::Plane3D passageGlobalEquation{};
        if (p_passage->getGlobalEquation(passageGlobalEquation) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGlobalEquation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Eigen::Vector4d passageEquation_World = passageGlobalEquation.coeffs();

        const double passageNormalNorm = passageEquation_World.head<3>().norm();

        if (!std::isfinite(passageNormalNorm) || passageNormalNorm < 1e-8)
        {
            continue;
        }

        passageEquation_World /= passageNormalNorm;

        /* Extract the passage centroid in double precision */
        Eigen::Vector3d passageCentroid2{};
        if (p_passage->getCentroid(passageCentroid2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCentroid returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const Eigen::Vector3d passageCentroid_World_m =
            passageCentroid2.cast<double>();

        semantic::Passage::KnownSideProvenance knownSide{};
        if (p_passage->getKnownSideProvenance(knownSide) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getKnownSideProvenance returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool knownSideHasDirection{};
        if (knownSide.hasDirection(knownSideHasDirection) !=
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
             * (Plane::getObservationOrigin_World()). Deriving it instead from
             * a median over the wall's whole observation history would
             * migrate to the far side once the UAV flew through this very
             * passage -- inverting the passage's own notion of which side it
             * was discovered from. The median remains only as a fallback for
             * faces created before the stamp existed. */
            for (geometric::Plane *p_supportingWall : supportingWalls)
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
                    continue;
                }

                std::optional<Eigen::Vector3d> observationOrigin_World_m{};
                if (p_supportingWall->getObservationOrigin_World(
                        observationOrigin_World_m) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getObservationOrigin_World cannot fail; continue as
                    // before.
                }

                std::optional<double> observedSide_m;

                if (observationOrigin_World_m.has_value() &&
                    observationOrigin_World_m->allFinite())
                {
                    observedSide_m = passageEquation_World.head<3>().dot(
                                         observationOrigin_World_m.value()) +
                                     passageEquation_World(3);
                }
                else
                {
                    geometric::Plane::ObservationSideSnapshot sideSnapshot{};
                    if (p_supportingWall->getObservationSideSnapshot(
                            passageEquation_World,
                            sideSnapshot) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        // getObservationSideSnapshot cannot fail; continue as
                        // before.
                    }
                    observedSide_m = sideSnapshot.medianSignedDistance_m;
                }

                if (!observedSide_m.has_value() ||
                    !std::isfinite(observedSide_m.value()))
                {
                    continue;
                }

                if (p_passage->setKnownSideDirection(
                        observedSide_m.value() > 0.0
                            ? Eigen::Vector3d(passageEquation_World.head<3>())
                            : Eigen::Vector3d(
                                  -passageEquation_World.head<3>())) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: setKnownSideDirection rejected its input; "
                                "continuing as before.",
                                __func__);
                }
                semantic::Passage::KnownSideProvenance
                    passageKnownSideProvenance{};
                if (p_passage->getKnownSideProvenance(
                        passageKnownSideProvenance) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKnownSideProvenance returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                knownSide = passageKnownSideProvenance;
                break;
            }
        }

        /* Track the closest room found on each side of the passage */
        vs_graphs::core::semantic::Room *p_negativeSideRoom = nullptr;
        vs_graphs::core::semantic::Room *p_positiveSideRoom = nullptr;
        vs_graphs::core::semantic::Room *p_negativeExactSupportingOwner =
            nullptr;
        vs_graphs::core::semantic::Room *p_positiveExactSupportingOwner =
            nullptr;

        double negativeRoomDistance_m = std::numeric_limits<double>::max();
        double positiveRoomDistance_m = std::numeric_limits<double>::max();

        /* Iterate through all valid rooms */
        for (vs_graphs::core::semantic::Room *p_room : allRooms)
        {
            /* Skip invalid rooms */
            bool roomIsBad2{};
            if (!(p_room == nullptr) &&
                p_room->isBad(roomIsBad2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room == nullptr || roomIsBad2)
            {
                continue;
            }

            /* Extract the walls assigned to the room */
            std::vector<vs_graphs::core::geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            /*
             * Match either the passage's source wall or a separately observed
             * wall surface at the same physical opening. Adjacent rooms must
             * not share one Plane pointer merely to obtain a graph edge.
             */
            const bool hasSupportingWallGeometry = std::any_of(
                roomWalls.begin(),
                roomWalls.end(),
                [&](vs_graphs::core::geometric::Plane *p_roomWall)
                {
                    bool roomWallIsBad{};
                    if (!(p_roomWall == nullptr) &&
                        p_roomWall->isBad(roomWallIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_roomWall == nullptr || roomWallIsBad)
                    {
                        return false;
                    }

                    if (std::find(supportingWalls.begin(),
                                  supportingWalls.end(),
                                  p_roomWall) != supportingWalls.end())
                    {
                        return true;
                    }

                    geometric::Plane::GeometrySnapshot roomWallGeometry{};
                    if (p_roomWall->getGeometrySnapshot(roomWallGeometry) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGeometrySnapshot returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    Eigen::Vector4d roomWallEquation =
                        roomWallGeometry.equation_World;

                    const double roomWallNormalNorm =
                        roomWallEquation.head<3>().norm();

                    if (!roomWallEquation.allFinite() ||
                        roomWallNormalNorm < 1e-8)
                    {
                        return false;
                    }

                    roomWallEquation /= roomWallNormalNorm;

                    const double normalAlignment =
                        std::abs(roomWallEquation.head<3>().dot(
                            passageEquation_World.head<3>()));

                    const double passagePlaneDistance_m =
                        std::abs(roomWallEquation.head<3>().dot(
                                     passageCentroid_World_m) +
                                 roomWallEquation(3));

                    if (normalAlignment < minimumNormalAlignment ||
                        passagePlaneDistance_m >
                            maximumSupportingPlaneDistance_m)
                    {
                        return false;
                    }

                    const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr
                        p_roomWallCloud = roomWallGeometry.supportCloud;

                    if (p_roomWallCloud == nullptr || p_roomWallCloud->empty())
                    {
                        return false;
                    }

                    double nearestOpeningEdgeDistance_m =
                        std::numeric_limits<double>::infinity();

                    for (const pcl::PointXYZRGBA &wallPoint :
                         p_roomWallCloud->points)
                    {
                        if (!pcl::isFinite(wallPoint))
                        {
                            continue;
                        }

                        const Eigen::Vector3d wallPoint_World_m(
                            static_cast<double>(wallPoint.x),
                            static_cast<double>(wallPoint.y),
                            static_cast<double>(wallPoint.z));

                        Eigen::Vector3d openingOffset_World_m =
                            wallPoint_World_m - passageCentroid_World_m;

                        openingOffset_World_m -=
                            openingOffset_World_m.dot(
                                roomWallEquation.head<3>()) *
                            roomWallEquation.head<3>();

                        nearestOpeningEdgeDistance_m =
                            std::min(nearestOpeningEdgeDistance_m,
                                     openingOffset_World_m.norm());
                    }

                    return nearestOpeningEdgeDistance_m <=
                           maximumOpeningEdgeDistance_m;
                });

            /* A passage can only belong to a room when the wall it is linked
             * to (its supporting wall) belongs to that room. A room that owns
             * none of the passage's supporting wall surfaces - even one whose
             * free-space skeleton happens to cross the opening - must not be
             * connected to this passage. The consistency knot of the semantic
             * graph is the wall itself: the passage anchors to walls, and
             * walls anchor to exactly one room. */
            if (!hasSupportingWallGeometry)
            {
                continue;
            }

            const bool ownsExactSupportingWall = std::any_of(
                roomWalls.begin(),
                roomWalls.end(),
                [&supportingWalls](geometric::Plane *p_roomWall)
                {
                    return std::find(supportingWalls.begin(),
                                     supportingWalls.end(),
                                     p_roomWall) != supportingWalls.end();
                });

            /* Association guard: exact ownership of the passage's supporting
             * wall is definitive adjacency evidence (handled above and
             * below). Anything weaker -- geometric proximity of some other
             * wall plus centroid distance -- may only compete for a side
             * when the room has enough boundary substance to make its
             * centroid meaningful. A single-wall room has no 2D extent; its
             * centroid sits on that one wall and wins whatever passage
             * happens to be nearest (typically right after a map reset,
             * latching the fresh room onto the wrong passage and locking it
             * in via wall ownership). Defer the edge until a second wall
             * arrives rather than invent topology. */
            if (!ownsExactSupportingWall)
            {
                constexpr std::size_t minimumWallsForProximityAssociation = 2U;
                std::size_t           validWallCount                      = 0U;
                for (geometric::Plane *p_roomWall : roomWalls)
                {
                    bool roomWallIsBad{};
                    if ((p_roomWall != nullptr) &&
                        p_roomWall->isBad(roomWallIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_roomWall != nullptr && !roomWallIsBad)
                    {
                        ++validWallCount;
                    }
                }
                if (validWallCount < minimumWallsForProximityAssociation)
                {
                    static std::set<std::pair<int, int>> reportedSparseSkips;
                    int                                  passageId2{};
                    if (p_passage->getId(passageId2) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int roomId{};
                    if (p_room->getId(roomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (reportedSparseSkips.emplace(passageId2, roomId).second)
                    {
                        int passageId3{};
                        if (p_passage->getId(passageId3) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        int roomId2{};
                        if (p_room->getId(roomId2) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(
                                rclcpp::get_logger("vs_graphs"),
                                "%s: getId returned a failure status although "
                                "it cannot fail; continuing as before.",
                                __func__);
                        }
                        std::cout
                            << "[SemMgr] semantic::Passage#" << passageId3
                            << " skipping semantic::Room#" << roomId2
                            << " (only " << validWallCount
                            << " valid wall(s); needs "
                            << minimumWallsForProximityAssociation
                            << " without exact supporting-wall ownership)."
                            << std::endl;
                    }
                    continue;
                }
            }

            /* Extract the room centroid */
            Eigen::Vector3d roomCentroid_World_m{};
            if (p_room->getCentroid(roomCentroid_World_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Determine which side of the passage plane contains the room */
            const double roomSide_m =
                passageEquation_World.head<3>().dot(roomCentroid_World_m) +
                passageEquation_World(3);

            /*!
             * A wall-centred provisional SE does not yet provide enough
             * evidence to form a room-to-passage connection -- UNLESS the
             * room owns the passage's own exact supporting wall. Owning
             * that wall is definitive, purely semantic (plane-equation)
             * evidence that the room borders this passage; which side of
             * the (nearly coincident, since the wall IS the passage's own
             * plane) passage plane the room's overall centroid happens to
             * land on is not meaningful evidence and must never veto it.
             * A sparsely-observed room (e.g. one confirmed wall so far)
             * can have its centroid sit within sideEpsilon_m purely
             * because that one known wall is this passage's supporting
             * wall -- silently dropping the room-to-passage edge every
             * cycle even though ownsExactSupportingWall already proves
             * the association.
             */
            if (!ownsExactSupportingWall &&
                std::abs(roomSide_m) <= sideEpsilon_m)
            {
                continue;
            }

            /* Find the distance from the room to the passage */
            const double roomDistance_m =
                (roomCentroid_World_m - passageCentroid_World_m).norm();

            /* The centroid of a sparse room can lie on its only known wall.
             * Exact ownership of the passage's supporting wall is still
             * definitive adjacency evidence; use the stamped observation-side
             * provenance to break the otherwise-zero side test. */
            if (ownsExactSupportingWall &&
                std::abs(roomSide_m) <= sideEpsilon_m)
            {
                bool knownSideHasDirection2{};
                if (knownSide.hasDirection(knownSideHasDirection2) !=
                    semantic::KnownSideProvenanceStatus::
                        KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                const double knownSideSign =
                    knownSideHasDirection2
                        ? knownSide.direction_World.dot(
                              passageEquation_World.head<3>())
                        : 0.0;
                if (knownSideSign >= 0.0 &&
                    roomDistance_m < positiveRoomDistance_m)
                {
                    positiveRoomDistance_m         = roomDistance_m;
                    p_positiveSideRoom             = p_room;
                    p_positiveExactSupportingOwner = p_room;
                }
                else if (knownSideSign < 0.0 &&
                         roomDistance_m < negativeRoomDistance_m)
                {
                    negativeRoomDistance_m         = roomDistance_m;
                    p_negativeSideRoom             = p_room;
                    p_negativeExactSupportingOwner = p_room;
                }
                continue;
            }

            /* Keep the closest room on the negative side */
            if (roomSide_m < 0.0 && roomDistance_m < negativeRoomDistance_m)
            {
                negativeRoomDistance_m = roomDistance_m;
                p_negativeSideRoom     = p_room;
                p_negativeExactSupportingOwner =
                    ownsExactSupportingWall ? p_room : nullptr;
            }

            /* Keep the closest room on the positive side */
            if (roomSide_m > 0.0 && roomDistance_m < positiveRoomDistance_m)
            {
                positiveRoomDistance_m = roomDistance_m;
                p_positiveSideRoom     = p_room;
                p_positiveExactSupportingOwner =
                    ownsExactSupportingWall ? p_room : nullptr;
            }
        }

        /*! Axiom: both rooms a passage links must be on the same floor,
         * except through a vertical passage / staircase (not implemented
         * yet -- see the user's own carve-out). A same-passage,
         * different-floor match is therefore not new information, it is a
         * matching error: since no vertical-passage mechanism exists to
         * produce a genuine one, one of the two sides must be wrong. Keep
         * whichever side is closer to the passage (the stronger match) and
         * drop the farther one back to unresolved for this cycle -- it can
         * still recover in a later cycle, e.g. once its own floor identity
         * is corrected, or a different room wins that side instead. */
        if (p_negativeSideRoom != nullptr && p_positiveSideRoom != nullptr)
        {
            vs_graphs::core::semantic::Floor *p_negativeFloor = nullptr;
            if (p_negativeSideRoom->getFloor(p_negativeFloor) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getFloor returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            vs_graphs::core::semantic::Floor *p_positiveFloor = nullptr;
            if (p_positiveSideRoom->getFloor(p_positiveFloor) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getFloor returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            bool negativeFloorHasPlaneIdentity{};
            if ((p_negativeFloor != nullptr && p_positiveFloor != nullptr) &&
                p_negativeFloor->hasPlaneIdentity(
                    negativeFloorHasPlaneIdentity) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasPlaneIdentity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool positiveFloorHasPlaneIdentity{};
            if ((p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
                 negativeFloorHasPlaneIdentity) &&
                p_positiveFloor->hasPlaneIdentity(
                    positiveFloorHasPlaneIdentity) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasPlaneIdentity returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int negativeFloorId{};
            if ((p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
                 negativeFloorHasPlaneIdentity &&
                 positiveFloorHasPlaneIdentity) &&
                p_negativeFloor->getId(negativeFloorId) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            int positiveFloorId{};
            if ((p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
                 negativeFloorHasPlaneIdentity &&
                 positiveFloorHasPlaneIdentity) &&
                p_positiveFloor->getId(positiveFloorId) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
                negativeFloorHasPlaneIdentity &&
                positiveFloorHasPlaneIdentity &&
                negativeFloorId != positiveFloorId)
            {
                const bool negativeIsFarther =
                    negativeRoomDistance_m >= positiveRoomDistance_m;
                vs_graphs::core::semantic::Room *p_droppedRoom =
                    negativeIsFarther ? p_negativeSideRoom : p_positiveSideRoom;

                int passageId4{};
                if (p_passage->getId(passageId4) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int negativeSideRoomId{};
                if (p_negativeSideRoom->getId(negativeSideRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int negativeFloorId2{};
                if (p_negativeFloor->getId(negativeFloorId2) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int positiveSideRoomId{};
                if (p_positiveSideRoom->getId(positiveSideRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int positiveFloorId2{};
                if (p_positiveFloor->getId(positiveFloorId2) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int droppedRoomId{};
                if (p_droppedRoom->getId(droppedRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] semantic::Passage#" << passageId4
                          << " matched semantic::Room#" << negativeSideRoomId
                          << " (semantic::Floor#" << negativeFloorId2
                          << ") and semantic::Room#" << positiveSideRoomId
                          << " (semantic::Floor#" << positiveFloorId2
                          << ") on different floors -- no vertical passage "
                             "mechanism exists, so this is a matching "
                             "error, not a real staircase; dropping the "
                             "farther match semantic::Room#"
                          << droppedRoomId << " for this cycle." << std::endl;

                if (negativeIsFarther)
                {
                    p_negativeSideRoom             = nullptr;
                    p_negativeExactSupportingOwner = nullptr;
                }
                else
                {
                    p_positiveSideRoom             = nullptr;
                    p_positiveExactSupportingOwner = nullptr;
                }
            }
        }

        /* Helper which adds a passage to a room without duplicates */
        const auto addPassageToRoom =
            [p_passage, &previousPassageIdsByRoom](
                vs_graphs::core::semantic::Room *p_room_inout)
        {
            /* Skip invalid rooms */
            if (p_room_inout == nullptr)
            {
                return;
            }

            /* Extract the passages already assigned to the room */
            std::vector<vs_graphs::core::semantic::Passage *> roomPassages{};
            if (p_room_inout->getPassages(roomPassages) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPassages returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            /* Check whether the relationship already exists */
            const bool alreadyAssociated = std::any_of(
                roomPassages.begin(),
                roomPassages.end(),
                [p_passage](
                    vs_graphs::core::semantic::Passage *p_existingPassage)
                {
                    int existingPassageId{};
                    if ((p_existingPassage != nullptr) &&
                        p_existingPassage->getId(existingPassageId) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int passageId{};
                    if ((p_existingPassage != nullptr) &&
                        p_passage->getId(passageId) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    return p_existingPassage != nullptr &&
                           existingPassageId == passageId;
                });

            /* Add the relationship if required */
            if (!alreadyAssociated)
            {
                if (p_room_inout->setDoorways(p_passage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setDoorways returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                const auto previousPassagesIterator =
                    previousPassageIdsByRoom.find(p_room_inout);

                int passageId{};
                if ((previousPassagesIterator !=
                     previousPassageIdsByRoom.end()) &&
                    p_passage->getId(passageId) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                const bool relationshipAlreadyExisted =
                    previousPassagesIterator !=
                        previousPassageIdsByRoom.end() &&
                    previousPassagesIterator->second.count(passageId) > 0U;

                if (!relationshipAlreadyExisted)
                {
                    int passageId2{};
                    if (p_passage->getId(passageId2) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int room_inoutId{};
                    if (p_room_inout->getId(room_inoutId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] Associated semantic::Passage#"
                              << passageId2 << " with semantic::Room#"
                              << room_inoutId << "." << std::endl;
                }
            }
        };

        /*!
         * Preserve every geometrically supported room-to-passage edge.
         *
         * A free-space ray may confirm an opening before the room on the far
         * side has enough boundary evidence to exist in the semantic graph.
         * Retaining the known-side edge represents that passage as a frontier
         * without inventing a second room. Once distinct rooms are observed on
         * both sides, the same two edges form the complete room-to-room route.
         */
        addPassageToRoom(p_negativeSideRoom);

        if (p_positiveSideRoom != p_negativeSideRoom)
        {
            addPassageToRoom(p_positiveSideRoom);
        }

        /* Enforce the invariant that a passage is linked to AT MOST TWO rooms
         * (the nearest room on each side of its supporting wall). Rooms that
         * no longer win their side - or that are duplicate hypotheses of the
         * winning side - must have this passage association revoked so the
         * semantic graph never shows a passage with three rooms. */
        for (vs_graphs::core::semantic::Room *p_candidateRoom : allRooms)
        {
            bool candidateRoomIsBad{};
            if (!(p_candidateRoom == nullptr) &&
                p_candidateRoom->isBad(candidateRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_candidateRoom == nullptr || candidateRoomIsBad ||
                p_candidateRoom == p_negativeSideRoom ||
                p_candidateRoom == p_positiveSideRoom)
            {
                continue;
            }

            bool candidateRoomWasPassageRemoved{};
            if (p_candidateRoom->removePassageAssociation(
                    p_passage,
                    candidateRoomWasPassageRemoved) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                candidateRoomWasPassageRemoved = false;
                RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                            "%s: removePassageAssociation rejected its input; "
                            "continuing as before.",
                            __func__);
            }
            if (candidateRoomWasPassageRemoved)
            {
                int passageId5{};
                if (p_passage->getId(passageId5) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int candidateRoomId{};
                if (p_candidateRoom->getId(candidateRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] Revoked semantic::Passage#" << passageId5
                          << " from semantic::Room#" << candidateRoomId
                          << " (enforcing max-2-rooms-per-passage)."
                          << std::endl;
            }
        }

        /* Enforce 1-2 room invariant for this passage */
        std::size_t                      associatedRoomCount          = 0;
        std::size_t                      confirmedAssociatedRoomCount = 0;
        vs_graphs::core::semantic::Room *p_confirmedAssociatedRoom    = nullptr;
        vs_graphs::core::semantic::Room *p_undefinedAssociatedRoom    = nullptr;

        const auto classifyAssociatedRoom =
            [&confirmedAssociatedRoomCount,
             &p_confirmedAssociatedRoom,
             &p_undefinedAssociatedRoom](
                vs_graphs::core::semantic::Room *p_room)
        {
            semantic::Room::RoomVariant roomVariant{};
            if (p_room->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (roomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
            {
                p_undefinedAssociatedRoom = p_room;
                return;
            }

            confirmedAssociatedRoomCount++;
            p_confirmedAssociatedRoom = p_room;
        };

        if (p_negativeSideRoom != nullptr)
        {
            associatedRoomCount++;
            classifyAssociatedRoom(p_negativeSideRoom);
        }
        if (p_positiveSideRoom != nullptr &&
            p_positiveSideRoom != p_negativeSideRoom)
        {
            associatedRoomCount++;
            classifyAssociatedRoom(p_positiveSideRoom);
        }

        bool knownSideHasDirection3{};
        if (knownSide.hasDirection(knownSideHasDirection3) !=
            semantic::KnownSideProvenanceStatus::
                KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        const double knownSideSign =
            knownSideHasDirection3
                ? knownSide.direction_World.dot(passageEquation_World.head<3>())
                : 0.0;
        const auto roomIsOnKnownSide = [&passageEquation_World,
                                        &knownSide,
                                        knownSideSign](semantic::Room *p_room)
        {
            bool knownSideHasDirection{};
            if (!(p_room == nullptr) &&
                knownSide.hasDirection(knownSideHasDirection) !=
                    semantic::KnownSideProvenanceStatus::
                        KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasDirection returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room == nullptr || !knownSideHasDirection ||
                std::abs(knownSideSign) < 1e-8)
            {
                return false;
            }
            Eigen::Vector3d roomCentroid{};
            if (p_room->getCentroid(roomCentroid) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getCentroid returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            const double roomSide_m =
                passageEquation_World.head<3>().dot(roomCentroid) +
                passageEquation_World(3);
            return roomSide_m * knownSideSign > 0.0;
        };

        if (knownSide.p_room == nullptr)
        {
            semantic::Room *p_knownSideRoom =
                roomIsOnKnownSide(p_negativeSideRoom)
                    ? p_negativeSideRoom
                    : (roomIsOnKnownSide(p_positiveSideRoom)
                           ? p_positiveSideRoom
                           : nullptr);
            if (p_knownSideRoom != nullptr)
            {
                if (p_passage->setKnownSideRoom(p_knownSideRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setKnownSideRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                semantic::Passage::KnownSideProvenance
                    passageKnownSideProvenance2{};
                if (p_passage->getKnownSideProvenance(
                        passageKnownSideProvenance2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKnownSideProvenance returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                knownSide = passageKnownSideProvenance2;
            }
        }

        if (associatedRoomCount == 0)
        {
            /* A passage linked to no room at all -- real or prospective --
             * is not a valid passage. Give it a short grace period (fresh
             * passages start at 0 rooms for a cycle or two before nearby
             * wall/room evidence catches up) before invalidating it, rather
             * than deleting on the very first zero-room cycle. Passage has
             * no removal from the Atlas, only Plane/Room's isBad()
             * convention (see Passage::setBad()'s own comment). */
            constexpr std::size_t maximumZeroRoomCycles = 5U;
            int                   passageId6{};
            if (p_passage->getId(passageId6) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            const std::size_t zeroRoomCycles =
                ++passageZeroRoomCycles[passageId6];

            if (zeroRoomCycles > maximumZeroRoomCycles)
            {
                if (p_passage->setBad() !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setBad returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                int passageId7{};
                if (p_passage->getId(passageId7) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                passageZeroRoomCycles.erase(passageId7);
                int passageId8{};
                if (p_passage->getId(passageId8) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] semantic::Passage#" << passageId8
                          << " invalidated: 0 associated rooms for "
                          << zeroRoomCycles << " consecutive cycles."
                          << std::endl;
            }
            else
            {
                int passageId9{};
                if (p_passage->getId(passageId9) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                bool knownSideHasDirection4{};
                if (knownSide.hasDirection(knownSideHasDirection4) !=
                    semantic::KnownSideProvenanceStatus::
                        KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: hasDirection returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                std::cout << "[SemMgr] semantic::Passage#" << passageId9
                          << " has 0 associated rooms (" << zeroRoomCycles
                          << "/" << maximumZeroRoomCycles
                          << " grace cycles); camera-side provenance="
                          << (knownSideHasDirection4 ? "known" : "missing")
                          << "." << std::endl;
            }
        }
        else
        {
            int passageId10{};
            if (p_passage->getId(passageId10) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            passageZeroRoomCycles.erase(passageId10);
        }

        if (associatedRoomCount > 2)
        {
            int passageId11{};
            if (p_passage->getId(passageId11) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] WARNING: semantic::Passage#" << passageId11
                      << " has " << associatedRoomCount
                      << " associated rooms; expected max 2." << std::endl;
        }

        /* ----------------------------------------------------------------------
         * * PROSPECTIVE ROOM CREATION
         * ----------------------------------------------------------------------
         * * When a passage has exactly 1 associated room, create a PROVISIONAL
         * (UNDEFINED variant) room on the far side. This represents the spatial
         * hypothesis that traversable space continues beyond the opening.
         *
         * The prospective room centroid is estimated as:
         *   passage_centroid + passage_normal * estimated_room_depth
         *
         * where passage_normal points from the known room toward the far side.
         * The depth heuristic (0.15 m) deliberately stays close to the
         * passage rather than guessing a typical room depth -- it is a
         * placeholder handle, not a position estimate, and gets corrected
         * the moment real far-side evidence (a wall, a cluster) arrives.
         *
         * Constraints:
         *   - Spatial deduplication: reuse existing prospective room
         * within 2.0m
         *   - Max 12 prospective rooms total (matches office_clean's 12 rooms)
         *   - Max 2 rooms per passage (near + far side)
         *   - Passage pointer is the persistent primary handle
         *   - Wall ownership alone never promotes the prospective
         *   - Validated far-side cluster evidence may promote it in place
         */
        vs_graphs::core::semantic::Room *p_existingProspective = nullptr;
        if (p_passage->getProspectiveRoom(p_existingProspective) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool hadProspectiveHandle = p_existingProspective != nullptr;

        bool existingProspectiveIsBad{};
        if ((p_existingProspective != nullptr) &&
            p_existingProspective->isBad(existingProspectiveIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant existingProspectiveRoomVariant{};
        if ((p_existingProspective != nullptr && !existingProspectiveIsBad) &&
            p_existingProspective->getRoomVariant(
                existingProspectiveRoomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_existingProspective != nullptr && existingProspectiveIsBad)
        {
            if (p_passage->setProspectiveRoom(nullptr) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
        else if (p_existingProspective != nullptr &&
                 existingProspectiveRoomVariant !=
                     vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            /* Promotion/replacement keeps the same far-side resolution. */
            if (p_existingProspective->setDoorways(p_passage) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setDoorways returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            int existingProspectiveId{};
            if (p_existingProspective->getId(existingProspectiveId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            prospectiveRoomCycles.erase(existingProspectiveId);
        }

        semantic::Room *p_currentFarSideHandle = nullptr;
        if (p_passage->getProspectiveRoom(p_currentFarSideHandle) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if ((confirmedAssociatedRoomCount == 1U &&
             p_currentFarSideHandle == p_confirmedAssociatedRoom) ||
            (confirmedAssociatedRoomCount == 2U &&
             p_currentFarSideHandle != nullptr &&
             p_currentFarSideHandle != p_negativeSideRoom &&
             p_currentFarSideHandle != p_positiveSideRoom))
        {
            if (p_passage->setProspectiveRoom(nullptr) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        if (confirmedAssociatedRoomCount == 2U)
        {
            semantic::Room *p_farSideConfirmedRoom = nullptr;
            bool            knownSideHasDirection11{};
            if (knownSide.hasDirection(knownSideHasDirection11) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: hasDirection returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (knownSide.p_room == p_negativeSideRoom)
            {
                p_farSideConfirmedRoom = p_positiveSideRoom;
            }
            else if (knownSide.p_room == p_positiveSideRoom)
            {
                p_farSideConfirmedRoom = p_negativeSideRoom;
            }
            else if (knownSideHasDirection11)
            {
                p_farSideConfirmedRoom = roomIsOnKnownSide(p_negativeSideRoom)
                                             ? p_positiveSideRoom
                                             : p_negativeSideRoom;
            }
            else if (p_negativeExactSupportingOwner != nullptr)
            {
                if (p_passage->setKnownSideRoom(
                        p_negativeExactSupportingOwner) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setKnownSideRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_passage->setKnownSideDirection(
                        -passageEquation_World.head<3>()) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: setKnownSideDirection rejected its input; "
                                "continuing as before.",
                                __func__);
                }
                p_farSideConfirmedRoom = p_positiveSideRoom;
            }
            else if (p_positiveExactSupportingOwner != nullptr)
            {
                if (p_passage->setKnownSideRoom(
                        p_positiveExactSupportingOwner) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setKnownSideRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_passage->setKnownSideDirection(
                        passageEquation_World.head<3>()) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                "%s: setKnownSideDirection rejected its input; "
                                "continuing as before.",
                                __func__);
                }
                p_farSideConfirmedRoom = p_negativeSideRoom;
            }
            if (p_farSideConfirmedRoom != nullptr)
            {
                semantic::Room *p_previousFarSideHandle = nullptr;
                if (p_passage->getProspectiveRoom(p_previousFarSideHandle) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_passage->setProspectiveRoom(p_farSideConfirmedRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setProspectiveRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                if (p_farSideConfirmedRoom->setDoorways(p_passage) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setDoorways returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                if (p_previousFarSideHandle != p_farSideConfirmedRoom)
                {
                    int passageId12{};
                    if (p_passage->getId(passageId12) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int farSideConfirmedRoomId{};
                    if (p_farSideConfirmedRoom->getId(farSideConfirmedRoomId) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout
                        << "[SemMgr] semantic::Passage#" << passageId12
                        << " resolved to opposite confirmed semantic::Room#"
                        << farSideConfirmedRoomId
                        << " with both sides observed." << std::endl;
                }
            }
        }

        bool passageHasProspectiveRoom{};
        if (p_passage->hasProspectiveRoom(passageHasProspectiveRoom) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!passageHasProspectiveRoom && confirmedAssociatedRoomCount == 1 &&
            p_undefinedAssociatedRoom != nullptr)
        {
            if (p_passage->setProspectiveRoom(p_undefinedAssociatedRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        bool passageHasProspectiveRoom2{};
        bool knownSideHasDirection5{};
        if (!(confirmedAssociatedRoomCount == 1) &&
            (confirmedAssociatedRoomCount == 0) &&
            knownSide.hasDirection(knownSideHasDirection5) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (((confirmedAssociatedRoomCount == 1 ||
              (confirmedAssociatedRoomCount == 0 && knownSideHasDirection5))) &&
            p_passage->hasProspectiveRoom(passageHasProspectiveRoom2) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool knownSideHasDirection6{};
        if (!(confirmedAssociatedRoomCount == 1) &&
            (confirmedAssociatedRoomCount == 0) &&
            knownSide.hasDirection(knownSideHasDirection6) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if ((confirmedAssociatedRoomCount == 1 ||
             (confirmedAssociatedRoomCount == 0 && knownSideHasDirection6)) &&
            !passageHasProspectiveRoom2)
        {
            /* Passage limit: don't create prospective if passage already has 2
             * rooms */
            if (associatedRoomCount >= 2)
            {
                int passageId13{};
                if (p_passage->getId(passageId13) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout << "[SemMgr] semantic::Passage#" << passageId13
                          << " already has 2 associated rooms; skipping "
                             "prospective creation."
                          << std::endl;
            }
            else
            {
                g2o::Plane3D passageGlobalEquation2{};
                if (p_passage->getGlobalEquation(passageGlobalEquation2) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
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
                    if (p_passage->getCentroid(passageCentroid3) !=
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
                        p_confirmedAssociatedRoom;
                    bool knownSideHasDirection7{};
                    if (knownSide.hasDirection(knownSideHasDirection7) !=
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
                        knownSideHasDirection7 ? knownSide.direction_World
                                               : Eigen::Vector3d::Zero();
                    bool knownSideHasDirection8{};
                    if (knownSide.hasDirection(knownSideHasDirection8) !=
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
                            passageNormal.dot(knownRoomCentroid2) +
                            passageEq(3);
                        knownSideDirection = knownRoomSide < 0.0
                                                 ? -passageNormal
                                                 : passageNormal;
                        if (p_passage->setKnownSideDirection(
                                knownSideDirection) !=
                            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                        "%s: setKnownSideDirection rejected "
                                        "its input; continuing as before.",
                                        __func__);
                        }
                        if (p_passage->setKnownSideRoom(p_knownRoom) !=
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

                    /*!
                     * Same class of flaw as the promotion search further
                     * below (and the same fix): a bare same-side-of-the-
                     * infinite-passage-plane sign test, with only a 0.05m
                     * epsilon, is satisfied by any room past this passage
                     * on the far side -- including a room several doors
                     * down the same corridor that is nowhere near this
                     * specific opening. This gate runs BEFORE any
                     * prospective placeholder exists, so it must carry the
                     * same rigor itself rather than relying on the
                     * promotion search to catch it later: the bounded
                     * aperture test (segmentCrossesPassageOpening) plus the
                     * intervening-wall test (segmentCrossesForeignWall).
                     */
                    geometric::Plane *p_anteChurnGroundPlane = nullptr;
                    if (p_atlas->getBiggestGroundPlane(
                            p_anteChurnGroundPlane) !=
                        AtlasStatus::ATLAS_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: getBiggestGroundPlane returned a "
                                     "failure status although it cannot fail; "
                                     "continuing as before.",
                                     __func__);
                    }

                    bool anteChurnGroundPlaneIsBad{};
                    if ((p_knownRoom != nullptr &&
                         p_anteChurnGroundPlane != nullptr) &&
                        p_anteChurnGroundPlane->isBad(
                            anteChurnGroundPlaneIsBad) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_knownRoom != nullptr &&
                        p_anteChurnGroundPlane != nullptr &&
                        !anteChurnGroundPlaneIsBad)
                    {
                        g2o::Plane3D anteChurnGroundPlaneGetGlobalEquation{};
                        if (p_anteChurnGroundPlane->getGlobalEquation(
                                anteChurnGroundPlaneGetGlobalEquation) !=
                            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                        {
                            // getGlobalEquation cannot fail; continue as
                            // before.
                        }
                        const Eigen::Vector4d anteChurnGroundEq =
                            anteChurnGroundPlaneGetGlobalEquation.coeffs();
                        const double anteChurnGroundNorm =
                            anteChurnGroundEq.head<3>().norm();

                        if (anteChurnGroundEq.allFinite() &&
                            anteChurnGroundNorm > 1e-8)
                        {
                            const Eigen::Vector3d anteChurnGroundNormal_World =
                                anteChurnGroundEq.head<3>() /
                                anteChurnGroundNorm;
                            const Eigen::Vector3d anteChurnGroundAxisU_World =
                                anteChurnGroundNormal_World.unitOrthogonal()
                                    .normalized();
                            const Eigen::Vector3d anteChurnGroundAxisV_World =
                                anteChurnGroundNormal_World
                                    .cross(anteChurnGroundAxisU_World)
                                    .normalized();
                            const types::SystemParams::RoomSeg::PassagePartition
                                &anteChurnPartitionParameters =
                                    p_sysParams->roomSeg.passagePartition;
                            const double anteChurnOpeningMargin_m =
                                static_cast<double>(anteChurnPartitionParameters
                                                        .openingMargin_m);
                            const double anteChurnMinimumSideDistance_m =
                                static_cast<double>(anteChurnPartitionParameters
                                                        .minimumSideDistance_m);
                            const types::SystemParams::RoomSeg::BoundaryTopology
                                &anteChurnTopologyParameters =
                                    p_sysParams->roomSeg.boundaryTopology;
                            Eigen::Vector3d knownRoomCentroid{};
                            if (p_knownRoom->getCentroid(knownRoomCentroid) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getCentroid returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                            const std::vector<vs_graphs::core::semantic::Room *>
                                anteChurnExcludedRooms = {p_knownRoom};

                            for (vs_graphs::core::semantic::Room *p_otherRoom :
                                 allRooms)
                            {
                                bool otherRoomIsBad{};
                                if (!(p_otherRoom == nullptr) &&
                                    p_otherRoom->isBad(otherRoomIsBad) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: isBad returned a failure status "
                                        "although it cannot fail; continuing "
                                        "as before.",
                                        __func__);
                                }
                                semantic::Room::RoomVariant
                                    otherRoomRoomVariant{};
                                if (!(p_otherRoom == nullptr ||
                                      otherRoomIsBad ||
                                      p_otherRoom == p_knownRoom) &&
                                    p_otherRoom->getRoomVariant(
                                        otherRoomRoomVariant) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                {
                                    // getRoomVariant cannot fail; continue as
                                    // before.
                                }
                                if (p_otherRoom == nullptr || otherRoomIsBad ||
                                    p_otherRoom == p_knownRoom ||
                                    otherRoomRoomVariant ==
                                        vs_graphs::core::semantic::Room::
                                            RoomVariant::UNDEFINED)
                                {
                                    continue;
                                }

                                Eigen::Vector3d otherRoomCentroid{};
                                if (p_otherRoom->getCentroid(
                                        otherRoomCentroid) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                                {
                                    // getCentroid cannot fail; continue as
                                    // before.
                                }
                                bool crossesPassageOpening{};
                                if (segmentCrossesPassageOpening(
                                        knownRoomCentroid,
                                        otherRoomCentroid,
                                        p_passage,
                                        anteChurnGroundNormal_World,
                                        anteChurnOpeningMargin_m,
                                        anteChurnMinimumSideDistance_m,
                                        crossesPassageOpening) !=
                                    SemanticsManagerStatus::
                                        SEMANTICS_MANAGER_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: segmentCrossesPassageOpening "
                                        "returned a failure status although it "
                                        "cannot fail; continuing as before.",
                                        __func__);
                                }
                                if (!crossesPassageOpening)
                                {
                                    continue;
                                }

                                Eigen::Vector3d otherRoomCentroid2{};
                                if (p_otherRoom->getCentroid(
                                        otherRoomCentroid2) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                                {
                                    // getCentroid cannot fail; continue as
                                    // before.
                                }
                                bool crossesForeignWall{};
                                if (segmentCrossesForeignWall(
                                        knownRoomCentroid,
                                        otherRoomCentroid2,
                                        anteChurnExcludedRooms,
                                        allRooms,
                                        anteChurnGroundAxisU_World,
                                        anteChurnGroundAxisV_World,
                                        anteChurnGroundNormal_World,
                                        anteChurnTopologyParameters
                                            .endpointTrimRatio,
                                        anteChurnTopologyParameters
                                            .minimumWallLength_m,
                                        crossesForeignWall) !=
                                    SemanticsManagerStatus::
                                        SEMANTICS_MANAGER_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: segmentCrossesForeignWall "
                                        "returned a failure status although it "
                                        "cannot fail; continuing as before.",
                                        __func__);
                                }
                                if (crossesForeignWall)
                                {
                                    continue;
                                }

                                farSideConfirmedRoomExists = true;
                                p_existingFarSideRoom      = p_otherRoom;
                                break;
                            }
                        }
                    }

                    if (farSideConfirmedRoomExists)
                    {
                        if (p_existingFarSideRoom != nullptr)
                        {
                            if (p_existingFarSideRoom->setDoorways(p_passage) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: setDoorways returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                            if (p_passage->setProspectiveRoom(
                                    p_existingFarSideRoom) !=
                                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                            {
                                // setProspectiveRoom cannot fail; continue as
                                // before.
                            }

                            int passageId14{};
                            if (p_passage->getId(passageId14) !=
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
                            std::cout
                                << "[SemMgr] semantic::Passage#" << passageId14
                                << " resolved directly to confirmed "
                                   "semantic::Room#"
                                << existingFarSideRoomId << " on the far side."
                                << std::endl;
                        }
                    }
                    else
                    {
                        /* Placeholder handle for "some room exists on the
                         * far side of this doorway," not a real position
                         * estimate -- it gets corrected the moment any real
                         * far-side evidence (a wall, a cluster) arrives. Kept
                         * close to the passage rather than out at a typical
                         * room's centre depth so it doesn't visually or
                         * spatially masquerade as a real room position in
                         * the meantime. */
                        constexpr double      estimatedRoomDepth_m = 0.15;
                        const Eigen::Vector3d prospectiveCentroid =
                            passageCentroid +
                            farSideNormal * estimatedRoomDepth_m;

                        /* SPATIAL DEDUPLICATION: Check if a
                         * candidate/prospective room already exists near this
                         * location (within 2.0m) across ALL passages. */
                        bool prospectiveExists = false;
                        std::vector<vs_graphs::core::semantic::Room *>
                            candidateRooms{};
                        if (p_atlas->getAllCandidateMapRooms(candidateRooms) !=
                            AtlasStatus::ATLAS_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getAllCandidateMapRooms returned "
                                         "a failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }

                        /* Count current prospective rooms (UNDEFINED variant
                         * candidates) */
                        int prospectiveRoomCount = 0;
                        for (vs_graphs::core::semantic::Room *p_candidate :
                             candidateRooms)
                        {
                            bool candidateIsBad{};
                            if ((p_candidate != nullptr) &&
                                p_candidate->isBad(candidateIsBad) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: isBad returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            semantic::Room::RoomVariant candidateRoomVariant{};
                            if ((p_candidate != nullptr && !candidateIsBad) &&
                                p_candidate->getRoomVariant(
                                    candidateRoomVariant) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                            {
                                // getRoomVariant cannot fail; continue as
                                // before.
                            }
                            if (p_candidate != nullptr && !candidateIsBad &&
                                candidateRoomVariant ==
                                    vs_graphs::core::semantic::Room::
                                        RoomVariant::UNDEFINED)
                            {
                                prospectiveRoomCount++;
                            }
                        }

                        /* Enforce max prospective rooms cap */
                        bool passageIsPassable{};
                        if ((prospectiveRoomCount >= kMaxProspectiveRooms) &&
                            p_passage->isPassable(passageIsPassable) !=
                                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: isPassable returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        bool passageTraversalEvidence{};
                        if ((prospectiveRoomCount >= kMaxProspectiveRooms &&
                             !passageIsPassable) &&
                            p_passage->getTraversalEvidence(
                                passageTraversalEvidence) !=
                                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                        {
                            // getTraversalEvidence cannot fail; continue as
                            // before.
                        }
                        if (prospectiveRoomCount >= kMaxProspectiveRooms &&
                            !passageIsPassable && !passageTraversalEvidence &&
                            !hadProspectiveHandle)
                        {
                            int passageId15{};
                            if (p_passage->getId(passageId15) !=
                                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                             "%s: getId returned a failure "
                                             "status although it cannot fail; "
                                             "continuing as before.",
                                             __func__);
                            }
                            std::cout << "[SemMgr] Max prospective rooms ("
                                      << kMaxProspectiveRooms
                                      << ") reached; skipping creation for "
                                         "semantic::Passage#"
                                      << passageId15 << std::endl;
                        }
                        else
                        {
                            for (vs_graphs::core::semantic::Room *p_candidate :
                                 candidateRooms)
                            {
                                bool candidateIsBad2{};
                                if (!(p_candidate == nullptr) &&
                                    p_candidate->isBad(candidateIsBad2) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: isBad returned a failure status "
                                        "although it cannot fail; continuing "
                                        "as before.",
                                        __func__);
                                }
                                semantic::Room::RoomVariant
                                    candidateRoomVariant2{};
                                if (!(p_candidate == nullptr ||
                                      candidateIsBad2) &&
                                    p_candidate->getRoomVariant(
                                        candidateRoomVariant2) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                {
                                    // getRoomVariant cannot fail; continue as
                                    // before.
                                }
                                if (p_candidate == nullptr || candidateIsBad2 ||
                                    candidateRoomVariant2 !=
                                        vs_graphs::core::semantic::Room::
                                            RoomVariant::UNDEFINED)
                                {
                                    continue;
                                }
                                Eigen::Vector3d candidateCentroid{};
                                if (p_candidate->getCentroid(
                                        candidateCentroid) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                                {
                                    // getCentroid cannot fail; continue as
                                    // before.
                                }
                                const double distance =
                                    (candidateCentroid - prospectiveCentroid)
                                        .norm();

                                bool passageIdentityMatches = false;
                                if (distance <= kProspectiveDedupDistance_m)
                                {
                                    for (semantic::Passage *p_candidatePassage :
                                         allPassages)
                                    {
                                        vs_graphs::core::semantic::Room
                                            *p_candidatePassageProspectiveRoom =
                                                nullptr;
                                        if (!(p_candidatePassage == nullptr ||
                                              p_candidatePassage ==
                                                  p_passage) &&
                                            p_candidatePassage->getProspectiveRoom(
                                                p_candidatePassageProspectiveRoom) !=
                                                semantic::PassageStatus::
                                                    PASSAGE_STATUS_SUCCESS)
                                        {
                                            RCLCPP_ERROR(
                                                rclcpp::get_logger("vs_graphs"),
                                                "%s: getProspectiveRoom "
                                                "returned a failure status "
                                                "although it cannot fail; "
                                                "continuing as before.",
                                                __func__);
                                        }
                                        if (p_candidatePassage == nullptr ||
                                            p_candidatePassage == p_passage ||
                                            p_candidatePassageProspectiveRoom !=
                                                p_candidate)
                                        {
                                            continue;
                                        }

                                        g2o::Plane3D
                                            candidatePassageGlobalEquation{};
                                        if (p_candidatePassage->getGlobalEquation(
                                                candidatePassageGlobalEquation) !=
                                            semantic::PassageStatus::
                                                PASSAGE_STATUS_SUCCESS)
                                        {
                                            RCLCPP_ERROR(
                                                rclcpp::get_logger("vs_graphs"),
                                                "%s: getGlobalEquation "
                                                "returned a failure status "
                                                "although it cannot fail; "
                                                "continuing as before.",
                                                __func__);
                                        }
                                        Eigen::Vector4d candidatePassageEq =
                                            candidatePassageGlobalEquation
                                                .coeffs();
                                        const double candidatePassageNorm =
                                            candidatePassageEq.head<3>().norm();
                                        if (!candidatePassageEq.allFinite() ||
                                            candidatePassageNorm < 1e-8)
                                        {
                                            continue;
                                        }
                                        candidatePassageEq /=
                                            candidatePassageNorm;

                                        Eigen::Vector3d
                                            candidatePassageCentroid{};
                                        if (p_candidatePassage->getCentroid(
                                                candidatePassageCentroid) !=
                                            semantic::PassageStatus::
                                                PASSAGE_STATUS_SUCCESS)
                                        {
                                            // getCentroid cannot fail; continue
                                            // as before.
                                        }
                                        const double openingDistance_m =
                                            (candidatePassageCentroid -
                                             passageCentroid)
                                                .norm();
                                        const double normalAlignment = std::abs(
                                            candidatePassageEq.head<3>().dot(
                                                passageEq.head<3>()));
                                        Eigen::Vector3d
                                            candidatePassageCentroid2{};
                                        if (p_candidatePassage->getCentroid(
                                                candidatePassageCentroid2) !=
                                            semantic::PassageStatus::
                                                PASSAGE_STATUS_SUCCESS)
                                        {
                                            // getCentroid cannot fail; continue
                                            // as before.
                                        }
                                        const double planeResidual_m = std::abs(
                                            passageEq.head<3>().dot(
                                                candidatePassageCentroid2) +
                                            passageEq(3));

                                        if (openingDistance_m <=
                                                p_sysParams->semSeg
                                                    .passageDetection
                                                    .duplicatePassageDistance_m &&
                                            normalAlignment >=
                                                p_sysParams->semSeg
                                                    .passageDetection
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
                                    if (p_passage->setProspectiveRoom(
                                            p_candidate) !=
                                        semantic::PassageStatus::
                                            PASSAGE_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: setProspectiveRoom returned a "
                                            "failure status although it cannot "
                                            "fail; continuing as before.",
                                            __func__);
                                    }
                                    if (p_candidate->setDoorways(p_passage) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                    {
                                        // setDoorways cannot fail; continue as
                                        // before.
                                    }
                                    prospectiveExists = true;
                                    int candidateId{};
                                    if (p_candidate->getId(candidateId) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                    {
                                        // getId cannot fail; continue as
                                        // before.
                                    }
                                    int passageId16{};
                                    if (p_passage->getId(passageId16) !=
                                        semantic::PassageStatus::
                                            PASSAGE_STATUS_SUCCESS)
                                    {
                                        // getId cannot fail; continue as
                                        // before.
                                    }
                                    std::cout << "[SemMgr] Reusing existing "
                                                 "prospective semantic::Room#"
                                              << candidateId
                                              << " (dist=" << distance
                                              << "m) for semantic::Passage#"
                                              << passageId16 << std::endl;
                                    break;
                                }
                            }

                            if (!prospectiveExists)
                            {
                                /* Create the prospective room */
                                vs_graphs::core::semantic::Room
                                    *p_prospectiveRoom = nullptr;
                                if (GeoSemHelpers::createBlankRoomCandidate(
                                        p_atlas,
                                        p_prospectiveRoom,
                                        prospectiveCentroid) !=
                                    GeoSemHelpersStatus::
                                        GEO_SEM_HELPERS_STATUS_SUCCESS)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("vs_graphs"),
                                        "%s: createBlankRoomCandidate returned "
                                        "a failure status although it cannot "
                                        "fail; continuing as before.",
                                        __func__);
                                }

                                if (p_prospectiveRoom != nullptr)
                                {
                                    /* Mark as provisional - will be promoted
                                     * when walls are observed */
                                    if (p_prospectiveRoom->setRoomVariant(
                                            vs_graphs::core::semantic::Room::
                                                RoomVariant::UNDEFINED) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                    {
                                        // setRoomVariant cannot fail; continue
                                        // as before.
                                    }
                                    int prospectiveRoomId{};
                                    if (p_prospectiveRoom->getId(
                                            prospectiveRoomId) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                    {
                                        // getId cannot fail; continue as
                                        // before.
                                    }
                                    if (p_prospectiveRoom->setName(
                                            "Prospective#" +
                                            std::to_string(
                                                prospectiveRoomId)) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                    {
                                        // setName cannot fail; continue as
                                        // before.
                                    }

                                    /* Add to atlas as a candidate (not yet a
                                     * confirmed room) */
                                    if (p_atlas->addCandidateMapRoom(
                                            p_prospectiveRoom) !=
                                        AtlasStatus::ATLAS_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: addCandidateMapRoom returned "
                                            "a failure status although it "
                                            "cannot fail; continuing as "
                                            "before.",
                                            __func__);
                                    }

                                    /* Link passage <-> prospective room */
                                    if (p_passage->setProspectiveRoom(
                                            p_prospectiveRoom) !=
                                        semantic::PassageStatus::
                                            PASSAGE_STATUS_SUCCESS)
                                    {
                                        RCLCPP_ERROR(
                                            rclcpp::get_logger("vs_graphs"),
                                            "%s: setProspectiveRoom returned a "
                                            "failure status although it cannot "
                                            "fail; continuing as before.",
                                            __func__);
                                    }
                                    if (p_prospectiveRoom->setDoorways(
                                            p_passage) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                    {
                                        // setDoorways cannot fail; continue as
                                        // before.
                                    }

                                    /* Register the live passage-created handle.
                                     */
                                    int prospectiveRoomId2{};
                                    if (p_prospectiveRoom->getId(
                                            prospectiveRoomId2) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                    {
                                        // getId cannot fail; continue as
                                        // before.
                                    }
                                    prospectiveRoomCycles[prospectiveRoomId2] =
                                        0;

                                    int prospectiveRoomId3{};
                                    if (p_prospectiveRoom->getId(
                                            prospectiveRoomId3) !=
                                        semantic::RoomStatus::
                                            ROOM_STATUS_SUCCESS)
                                    {
                                        // getId cannot fail; continue as
                                        // before.
                                    }
                                    int passageId17{};
                                    if (p_passage->getId(passageId17) !=
                                        semantic::PassageStatus::
                                            PASSAGE_STATUS_SUCCESS)
                                    {
                                        // getId cannot fail; continue as
                                        // before.
                                    }
                                    std::cout << "[SemMgr] Created prospective "
                                                 "semantic::Room#"
                                              << prospectiveRoomId3 << " at "
                                              << prospectiveCentroid.transpose()
                                              << " for semantic::Passage#"
                                              << passageId17
                                              << " (total prospective: "
                                              << prospectiveRoomCount + 1 << ")"
                                              << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }

        bool passageIsPassable2{};
        if (p_passage->isPassable(passageIsPassable2) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isPassable returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool passageTraversalEvidence2{};
        if (!(passageIsPassable2) &&
            p_passage->getTraversalEvidence(passageTraversalEvidence2) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTraversalEvidence returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool passageHasProspectiveRoom3{};
        bool knownSideHasDirection9{};
        if (((passageIsPassable2 || passageTraversalEvidence2)) &&
            !(confirmedAssociatedRoomCount > 0U) &&
            knownSide.hasDirection(knownSideHasDirection9) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (((passageIsPassable2 || passageTraversalEvidence2) &&
             (confirmedAssociatedRoomCount > 0U || knownSideHasDirection9)) &&
            p_passage->hasProspectiveRoom(passageHasProspectiveRoom3) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        bool knownSideHasDirection10{};
        if (((passageIsPassable2 || passageTraversalEvidence2)) &&
            !(confirmedAssociatedRoomCount > 0U) &&
            knownSide.hasDirection(knownSideHasDirection10) !=
                semantic::KnownSideProvenanceStatus::
                    KNOWN_SIDE_PROVENANCE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasDirection returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if ((passageIsPassable2 || passageTraversalEvidence2) &&
            (confirmedAssociatedRoomCount > 0U || knownSideHasDirection10) &&
            !passageHasProspectiveRoom3)
        {
            int passageId18{};
            if (p_passage->getId(passageId18) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cerr << "[SemMgr] WARNING: semantic::Passage#" << passageId18
                      << " has a confirmed side but no stable far-side handle; "
                         "it is not routable this cycle."
                      << std::endl;
        }

        /* ----------------------------------------------------------------------
         * * PROSPECTIVE ROOM PROMOTION
         * ----------------------------------------------------------------------
         * * Wall ownership alone cannot promote a prospective. Promotion is
         * performed only by detectRoom_FreeSpaceCluster() after an independent
         * far-side cluster matches the prospective's existing walls. If a
         * distinct confirmed room already resolves the far side, retire the
         * placeholder and preserve that room as the passage's stable handle.
         */
        bool passageHasProspectiveRoom4{};
        if (p_passage->hasProspectiveRoom(passageHasProspectiveRoom4) !=
            semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: hasProspectiveRoom returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (passageHasProspectiveRoom4)
        {
            vs_graphs::core::semantic::Room *p_prospectiveRoom = nullptr;
            if (p_passage->getProspectiveRoom(p_prospectiveRoom) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getProspectiveRoom returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            bool prospectiveRoomIsBad{};
            if ((p_prospectiveRoom != nullptr) &&
                p_prospectiveRoom->isBad(prospectiveRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Room::RoomVariant prospectiveRoomRoomVariant{};
            if ((p_prospectiveRoom != nullptr && !prospectiveRoomIsBad) &&
                p_prospectiveRoom->getRoomVariant(prospectiveRoomRoomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRoomVariant returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            std::vector<geometric::Plane *> prospectiveRoomWalls{};
            if ((p_prospectiveRoom != nullptr && !prospectiveRoomIsBad &&
                 prospectiveRoomRoomVariant ==
                     vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED) &&
                p_prospectiveRoom->getWalls(prospectiveRoomWalls) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_prospectiveRoom != nullptr && !prospectiveRoomIsBad &&
                prospectiveRoomRoomVariant ==
                    vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED &&
                !prospectiveRoomWalls.empty())
            {
                /* Find a confirmed (non-prospective) room on the FAR side of
                 * the passage - i.e. on the same side as the prospective room.
                 * Use ANY such room, not just the strictly associated side
                 * rooms, so that a passage opening onto a corridor or another
                 * already-mapped room stops being prospective as soon as that
                 * room exists. When one is found, the prospective placeholder
                 * is deleted - it must not linger as a loose provisional room.
                 */
                vs_graphs::core::semantic::Room *p_farSideConfirmedRoom =
                    nullptr;

                /*!
                 * Per WP8-B's own driving principle: "a wall observed on the
                 * far side of ANY confirmed passage aperture belongs to that
                 * passage's prospective room... decide which side of every
                 * passage it lies on" -- using segmentCrossesPassageOpening,
                 * the same purely semantic (passage width/height aperture,
                 * not voxblox cluster geometry) test already used for wall
                 * admission (e.g. line ~3759). A same-side-of-the-infinite-
                 * plane sign test is NOT proof of adjacency: a room several
                 * metres past this passage (only reachable through an
                 * intervening, not-yet-confirmed room) satisfies "same
                 * side" just as well as a genuinely bordering room does.
                 * Observed directly: Passage#1 sitting between Room#4 and a
                 * distant Room#5 kept resolving straight to Room#5,
                 * destroying the middle prospective room meant to sit
                 * between them, every cycle. Testing whether the segment
                 * between the two room centroids actually threads through
                 * THIS passage's own bounded opening (not just crosses its
                 * infinite plane somewhere) rejects that distant match
                 * without any distance threshold borrowed from an unrelated
                 * (voxblox free-space cluster) subsystem.
                 *
                 * This still isn't sufficient on its own when the
                 * prospective room is nothing but its creation-time
                 * heuristic position (passage_centroid + normal * an
                 * assumed depth, before any real wall has been admitted to
                 * it): in a straight corridor with several doors in a row,
                 * that guessed point and a genuinely distant, unrelated
                 * room can both sit close enough to the corridor centreline
                 * for the straight segment between them to thread THIS
                 * passage's aperture too, even though a different room and
                 * passage lie directly between them. Observed directly:
                 * a freshly created prospective room, still with zero
                 * walls, resolved straight to a confirmed room three doors
                 * down the same corridor on the very cycle it was created.
                 * Requiring at least one real, admitted wall first (the
                 * same evidence bar promotion already applies below: "wall
                 * ownership alone cannot promote... validated far-side
                 * cluster evidence may promote") anchors the near endpoint
                 * of the crossing test to an actually observed position
                 * instead of an unvalidated depth guess.
                 *
                 * Also excludes the passage's own near-side/known room from
                 * candidacy, mirroring the creation-time anti-churn guard
                 * above (§7342-7374) -- a room that owns the passage's own
                 * near side must never be matched as its far side.
                 */
                semantic::Passage::KnownSideProvenance
                    passageKnownSideProvenance3{};
                if (p_passage->getKnownSideProvenance(
                        passageKnownSideProvenance3) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getKnownSideProvenance returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                vs_graphs::core::semantic::Room *p_knownSideRoom =
                    passageKnownSideProvenance3.p_room;

                geometric::Plane *p_groundPlane = nullptr;
                if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
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
                    if (p_groundPlane->getGlobalEquation(
                            groundPlaneGetGlobalEquation) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getGlobalEquation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    const Eigen::Vector4d groundEquation_World =
                        groundPlaneGetGlobalEquation.coeffs();
                    const double groundNormalNorm =
                        groundEquation_World.head<3>().norm();

                    if (groundEquation_World.allFinite() &&
                        groundNormalNorm > 1e-8)
                    {
                        const Eigen::Vector3d groundNormal_World =
                            groundEquation_World.head<3>() / groundNormalNorm;
                        const Eigen::Vector3d groundAxisU_World =
                            groundNormal_World.unitOrthogonal().normalized();
                        const Eigen::Vector3d groundAxisV_World =
                            groundNormal_World.cross(groundAxisU_World)
                                .normalized();
                        const types::SystemParams::RoomSeg::PassagePartition
                            &partitionParameters =
                                p_sysParams->roomSeg.passagePartition;
                        const double openingMargin_m = static_cast<double>(
                            partitionParameters.openingMargin_m);
                        const double minimumSideDistance_m =
                            static_cast<double>(
                                partitionParameters.minimumSideDistance_m);
                        const types::SystemParams::RoomSeg::BoundaryTopology
                            &topologyParameters =
                                p_sysParams->roomSeg.boundaryTopology;
                        Eigen::Vector3d prospectiveCentroid{};
                        if (p_prospectiveRoom->getCentroid(
                                prospectiveCentroid) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: getCentroid returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                        const std::vector<vs_graphs::core::semantic::Room *>
                            excludedRooms = {p_prospectiveRoom,
                                             p_knownSideRoom};

                        for (vs_graphs::core::semantic::Room *p_otherRoom :
                             allRooms)
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
                                  p_otherRoom == p_prospectiveRoom ||
                                  p_otherRoom == p_knownSideRoom) &&
                                p_otherRoom->getRoomVariant(
                                    otherRoomRoomVariant2) !=
                                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                            {
                                // getRoomVariant cannot fail; continue as
                                // before.
                            }
                            if (p_otherRoom == nullptr || otherRoomIsBad2 ||
                                p_otherRoom == p_prospectiveRoom ||
                                p_otherRoom == p_knownSideRoom ||
                                otherRoomRoomVariant2 ==
                                    vs_graphs::core::semantic::Room::
                                        RoomVariant::UNDEFINED)
                            {
                                continue;
                            }

                            Eigen::Vector3d otherRoomCentroid3{};
                            if (p_otherRoom->getCentroid(otherRoomCentroid3) !=
                                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getCentroid returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                            bool crossesPassageOpening2{};
                            if (segmentCrossesPassageOpening(
                                    prospectiveCentroid,
                                    otherRoomCentroid3,
                                    p_passage,
                                    groundNormal_World,
                                    openingMargin_m,
                                    minimumSideDistance_m,
                                    crossesPassageOpening2) !=
                                SemanticsManagerStatus::
                                    SEMANTICS_MANAGER_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
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
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: getCentroid returned a failure status "
                                    "although it cannot fail; continuing as "
                                    "before.",
                                    __func__);
                            }
                            bool crossesForeignWall2{};
                            if (segmentCrossesForeignWall(
                                    prospectiveCentroid,
                                    otherRoomCentroid4,
                                    excludedRooms,
                                    allRooms,
                                    groundAxisU_World,
                                    groundAxisV_World,
                                    groundNormal_World,
                                    topologyParameters.endpointTrimRatio,
                                    topologyParameters.minimumWallLength_m,
                                    crossesForeignWall2) !=
                                SemanticsManagerStatus::
                                    SEMANTICS_MANAGER_STATUS_SUCCESS)
                            {
                                RCLCPP_ERROR(
                                    rclcpp::get_logger("vs_graphs"),
                                    "%s: segmentCrossesForeignWall returned a "
                                    "failure status although it cannot fail; "
                                    "continuing as before.",
                                    __func__);
                            }
                            if (crossesForeignWall2)
                            {
                                continue;
                            }

                            p_farSideConfirmedRoom = p_otherRoom;
                            break;
                        }
                    }
                }

                if (p_farSideConfirmedRoom != nullptr)
                {
                    /* Transfer uniquely held evidence before retiring the
                     * distinct placeholder. Failed admissions remain unowned
                     * for the normal wall-association pass below. */
                    if (p_passage->setProspectiveRoom(p_farSideConfirmedRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setProspectiveRoom returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    std::vector<geometric::Plane *> prospectiveRoomWalls2{};
                    if (p_prospectiveRoom->getWalls(prospectiveRoomWalls2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getWalls returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }
                    for (vs_graphs::core::geometric::Plane *p_wall :
                         prospectiveRoomWalls2)
                    {
                        bool wasAdmitted{};
                        if (admitWallToRoom(p_farSideConfirmedRoom,
                                            p_wall,
                                            wasAdmitted) !=
                            SemanticsManagerStatus::
                                SEMANTICS_MANAGER_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: admitWallToRoom returned a "
                                         "failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                        bool prospectiveRoomWasWallRemoved{};
                        if (p_prospectiveRoom->removeWall(
                                p_wall,
                                prospectiveRoomWasWallRemoved) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                        {
                            prospectiveRoomWasWallRemoved = false;
                            RCLCPP_WARN(rclcpp::get_logger("vs_graphs"),
                                        "%s: removeWall rejected its input; "
                                        "continuing as before.",
                                        __func__);
                        }
                    }

                    int prospectiveRoomId4{};
                    if (p_prospectiveRoom->getId(prospectiveRoomId4) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    prospectiveRoomCycles.erase(prospectiveRoomId4);

                    Map *p_roomMap = nullptr;
                    if (p_prospectiveRoom->getMap(p_roomMap) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getMap returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_roomMap != nullptr)
                    {
                        if (p_roomMap->eraseMarkerBasedMapRoom(
                                p_prospectiveRoom) !=
                            MapStatus::MAP_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: eraseMarkerBasedMapRoom returned "
                                         "a failure status although it cannot "
                                         "fail; continuing as before.",
                                         __func__);
                        }
                    }

                    if (p_prospectiveRoom->clearPassages() !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: clearPassages returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_prospectiveRoom->setBad() !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }

                    if (p_farSideConfirmedRoom->setDoorways(p_passage) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setDoorways returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }

                    int prospectiveRoomId5{};
                    if (p_prospectiveRoom->getId(prospectiveRoomId5) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int farSideConfirmedRoomId2{};
                    if (p_farSideConfirmedRoom->getId(
                            farSideConfirmedRoomId2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    int passageId19{};
                    if (p_passage->getId(passageId19) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getId returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    std::cout << "[SemMgr] Resolved prospective semantic::Room#"
                              << prospectiveRoomId5
                              << " with confirmed semantic::Room#"
                              << farSideConfirmedRoomId2
                              << " for semantic::Passage#" << passageId19 << "."
                              << std::endl;
                }
            }
        }
    }

    /* Report, but never fabricate, missing passage connectivity. */
    std::unordered_set<int> computedDisconnectedRoomIds;

    for (vs_graphs::core::semantic::Room *p_room : allRooms)
    {
        bool roomIsBad3{};
        if (!(p_room == nullptr) &&
            p_room->isBad(roomIsBad3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        semantic::Room::RoomVariant roomVariant{};
        if (!(p_room == nullptr || roomIsBad3) &&
            p_room->getRoomVariant(roomVariant) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRoomVariant returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<vs_graphs::core::semantic::Passage *> roomPassages3{};
        if (!(p_room == nullptr || roomIsBad3 ||
              roomVariant ==
                  vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED) &&
            p_room->getPassages(roomPassages3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPassages returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || roomIsBad3 ||
            roomVariant ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED ||
            !roomPassages3.empty())
        {
            continue;
        }

        int roomId3{};
        if (p_room->getId(roomId3) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        computedDisconnectedRoomIds.insert(roomId3);

        int roomId4{};
        if (p_room->getId(roomId4) != semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (disconnectedRoomIds.count(roomId4) == 0U)
        {
            int roomId5{};
            if (p_room->getId(roomId5) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cout << "[SemMgr] semantic::Room#" << roomId5
                      << " is not yet connected by a confirmed passage; "
                         "semantic routing will treat it as disconnected."
                      << std::endl;
        }
    }

    disconnectedRoomIds = std::move(computedDisconnectedRoomIds);

    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
