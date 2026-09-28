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

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{

void SemanticsManager::associatePassagesToRooms(void)
{
    /* Extract all rooms from the current map */
    std::vector<vs_graphs::core::semantic::Room *> allRooms =
        p_atlas->getAllRooms();

    /* Extract all passages from the current map */
    const std::vector<vs_graphs::core::semantic::Passage *> allPassages =
        p_atlas->getAllPassages();

    constexpr double sideEpsilon_m = 0.20;

    constexpr double maximumSupportingPlaneDistance_m = 1.00;
    constexpr double maximumOpeningEdgeDistance_m     = 3.00;
    constexpr double minimumNormalAlignment           = 0.80;

    /* Stable room ordering makes equal-distance passage associations
     * repeatable. */
    std::sort(allRooms.begin(),
              allRooms.end(),
              [](const semantic::Room *p_firstRoom,
                 const semantic::Room *p_secondRoom)
              {
                  if (p_firstRoom == nullptr)
                  {
                      return false;
                  }

                  if (p_secondRoom == nullptr)
                  {
                      return true;
                  }

                  return p_firstRoom->getId() < p_secondRoom->getId();
              });

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
        if (p_room != nullptr && !p_room->isBad())
        {
            std::unordered_set<int> &previousPassageIds =
                previousPassageIdsByRoom[p_room];

            for (vs_graphs::core::semantic::Passage *p_previousPassage :
                 p_room->getPassages())
            {
                if (p_previousPassage != nullptr)
                {
                    previousPassageIds.insert(p_previousPassage->getId());
                }
            }

            p_room->clearPassages();
        }
    }

    /* Iterate through every passage */
    for (vs_graphs::core::semantic::Passage *p_passage : allPassages)
    {
        /* Skip invalid passages */
        if (p_passage == nullptr || p_passage->isBad())
        {
            continue;
        }

        /* A recovery proxy carries only stable identity and topology. Its
         * historical coordinates deliberately are not copied into the new
         * map frame. Preserve its reciprocal room edge until map alignment
         * can reconcile it with newly observed passage geometry. */
        if (p_passage->isRecoveryProxy())
        {
            const semantic::Passage::KnownSideProvenance knownSide =
                p_passage->getKnownSideProvenance();
            semantic::Room *p_farSideRoom = p_passage->getProspectiveRoom();
            if (knownSide.p_room != nullptr && !knownSide.p_room->isBad())
            {
                knownSide.p_room->setDoorways(p_passage);
            }
            if (p_farSideRoom != nullptr && !p_farSideRoom->isBad())
            {
                p_farSideRoom->setDoorways(p_passage);
            }
            passageZeroRoomCycles.erase(p_passage->getId());
            continue;
        }

        /* Extract the wall or walls supporting the passage */
        const std::vector<vs_graphs::core::geometric::Plane *> supportingWalls =
            p_passage->getAssociateWalls();

        /* A passage without a supporting wall cannot connect rooms */
        if (supportingWalls.empty())
        {
            continue;
        }

        /* Extract and normalize the passage plane equation */
        Eigen::Vector4d passageEquation_World =
            p_passage->getGlobalEquation().coeffs();

        const double passageNormalNorm = passageEquation_World.head<3>().norm();

        if (!std::isfinite(passageNormalNorm) || passageNormalNorm < 1e-8)
        {
            continue;
        }

        passageEquation_World /= passageNormalNorm;

        /* Extract the passage centroid in double precision */
        const Eigen::Vector3d passageCentroid_World_m =
            p_passage->getCentroid().cast<double>();

        semantic::Passage::KnownSideProvenance knownSide =
            p_passage->getKnownSideProvenance();
        if (!knownSide.hasDirection())
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
                if (p_supportingWall == nullptr || p_supportingWall->isBad())
                {
                    continue;
                }

                const std::optional<Eigen::Vector3d> observationOrigin_World_m =
                    p_supportingWall->getObservationOrigin_World();

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
                    const geometric::Plane::ObservationSideSnapshot
                        sideSnapshot =
                            p_supportingWall->getObservationSideSnapshot(
                                passageEquation_World);
                    observedSide_m = sideSnapshot.medianSignedDistance_m;
                }

                if (!observedSide_m.has_value() ||
                    !std::isfinite(observedSide_m.value()))
                {
                    continue;
                }

                p_passage->setKnownSideDirection(
                    observedSide_m.value() > 0.0
                        ? Eigen::Vector3d(passageEquation_World.head<3>())
                        : Eigen::Vector3d(-passageEquation_World.head<3>()));
                knownSide = p_passage->getKnownSideProvenance();
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
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            /* Extract the walls assigned to the room */
            const std::vector<vs_graphs::core::geometric::Plane *> roomWalls =
                p_room->getWalls();

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
                    if (p_roomWall == nullptr || p_roomWall->isBad())
                    {
                        return false;
                    }

                    if (std::find(supportingWalls.begin(),
                                  supportingWalls.end(),
                                  p_roomWall) != supportingWalls.end())
                    {
                        return true;
                    }

                    const geometric::Plane::GeometrySnapshot roomWallGeometry =
                        p_roomWall->getGeometrySnapshot();
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
                    if (p_roomWall != nullptr && !p_roomWall->isBad())
                    {
                        ++validWallCount;
                    }
                }
                if (validWallCount < minimumWallsForProximityAssociation)
                {
                    static std::set<std::pair<int, int>> reportedSparseSkips;
                    if (reportedSparseSkips
                            .emplace(p_passage->getId(), p_room->getId())
                            .second)
                    {
                        std::cout
                            << "[SemMgr] semantic::Passage#"
                            << p_passage->getId() << " skipping semantic::Room#"
                            << p_room->getId() << " (only " << validWallCount
                            << " valid wall(s); needs "
                            << minimumWallsForProximityAssociation
                            << " without exact supporting-wall ownership)."
                            << std::endl;
                    }
                    continue;
                }
            }

            /* Extract the room centroid */
            const Eigen::Vector3d roomCentroid_World_m = p_room->getCentroid();

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
                const double knownSideSign =
                    knownSide.hasDirection()
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
            vs_graphs::core::semantic::Floor *p_negativeFloor =
                p_negativeSideRoom->getFloor();
            vs_graphs::core::semantic::Floor *p_positiveFloor =
                p_positiveSideRoom->getFloor();

            if (p_negativeFloor != nullptr && p_positiveFloor != nullptr &&
                p_negativeFloor->hasPlaneIdentity() &&
                p_positiveFloor->hasPlaneIdentity() &&
                p_negativeFloor->getId() != p_positiveFloor->getId())
            {
                const bool negativeIsFarther =
                    negativeRoomDistance_m >= positiveRoomDistance_m;
                vs_graphs::core::semantic::Room *p_droppedRoom =
                    negativeIsFarther ? p_negativeSideRoom : p_positiveSideRoom;

                std::cout << "[SemMgr] semantic::Passage#" << p_passage->getId()
                          << " matched semantic::Room#"
                          << p_negativeSideRoom->getId() << " (semantic::Floor#"
                          << p_negativeFloor->getId() << ") and semantic::Room#"
                          << p_positiveSideRoom->getId() << " (semantic::Floor#"
                          << p_positiveFloor->getId()
                          << ") on different floors -- no vertical passage "
                             "mechanism exists, so this is a matching "
                             "error, not a real staircase; dropping the "
                             "farther match semantic::Room#"
                          << p_droppedRoom->getId() << " for this cycle."
                          << std::endl;

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
            const std::vector<vs_graphs::core::semantic::Passage *>
                roomPassages = p_room_inout->getPassages();

            /* Check whether the relationship already exists */
            const bool alreadyAssociated = std::any_of(
                roomPassages.begin(),
                roomPassages.end(),
                [p_passage](
                    vs_graphs::core::semantic::Passage *p_existingPassage)
                {
                    return p_existingPassage != nullptr &&
                           p_existingPassage->getId() == p_passage->getId();
                });

            /* Add the relationship if required */
            if (!alreadyAssociated)
            {
                p_room_inout->setDoorways(p_passage);

                const auto previousPassagesIterator =
                    previousPassageIdsByRoom.find(p_room_inout);

                const bool relationshipAlreadyExisted =
                    previousPassagesIterator !=
                        previousPassageIdsByRoom.end() &&
                    previousPassagesIterator->second.count(p_passage->getId()) >
                        0U;

                if (!relationshipAlreadyExisted)
                {
                    std::cout << "[SemMgr] Associated semantic::Passage#"
                              << p_passage->getId() << " with semantic::Room#"
                              << p_room_inout->getId() << "." << std::endl;
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
            if (p_candidateRoom == nullptr || p_candidateRoom->isBad() ||
                p_candidateRoom == p_negativeSideRoom ||
                p_candidateRoom == p_positiveSideRoom)
            {
                continue;
            }

            if (p_candidateRoom->removePassageAssociation(p_passage))
            {
                std::cout << "[SemMgr] Revoked semantic::Passage#"
                          << p_passage->getId() << " from semantic::Room#"
                          << p_candidateRoom->getId()
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
            if (p_room->getRoomVariant() ==
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

        const double knownSideSign =
            knownSide.hasDirection()
                ? knownSide.direction_World.dot(passageEquation_World.head<3>())
                : 0.0;
        const auto roomIsOnKnownSide = [&passageEquation_World,
                                        &knownSide,
                                        knownSideSign](semantic::Room *p_room)
        {
            if (p_room == nullptr || !knownSide.hasDirection() ||
                std::abs(knownSideSign) < 1e-8)
            {
                return false;
            }
            const double roomSide_m =
                passageEquation_World.head<3>().dot(p_room->getCentroid()) +
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
                p_passage->setKnownSideRoom(p_knownSideRoom);
                knownSide = p_passage->getKnownSideProvenance();
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
            const std::size_t     zeroRoomCycles =
                ++passageZeroRoomCycles[p_passage->getId()];

            if (zeroRoomCycles > maximumZeroRoomCycles)
            {
                p_passage->setBad();
                passageZeroRoomCycles.erase(p_passage->getId());
                std::cout << "[SemMgr] semantic::Passage#" << p_passage->getId()
                          << " invalidated: 0 associated rooms for "
                          << zeroRoomCycles << " consecutive cycles."
                          << std::endl;
            }
            else
            {
                std::cout << "[SemMgr] semantic::Passage#" << p_passage->getId()
                          << " has 0 associated rooms (" << zeroRoomCycles
                          << "/" << maximumZeroRoomCycles
                          << " grace cycles); camera-side provenance="
                          << (knownSide.hasDirection() ? "known" : "missing")
                          << "." << std::endl;
            }
        }
        else
        {
            passageZeroRoomCycles.erase(p_passage->getId());
        }

        if (associatedRoomCount > 2)
        {
            std::cout << "[SemMgr] WARNING: semantic::Passage#"
                      << p_passage->getId() << " has " << associatedRoomCount
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
        vs_graphs::core::semantic::Room *p_existingProspective =
            p_passage->getProspectiveRoom();
        const bool hadProspectiveHandle = p_existingProspective != nullptr;

        if (p_existingProspective != nullptr && p_existingProspective->isBad())
        {
            p_passage->setProspectiveRoom(nullptr);
        }
        else if (p_existingProspective != nullptr &&
                 p_existingProspective->getRoomVariant() !=
                     vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED)
        {
            /* Promotion/replacement keeps the same far-side resolution. */
            p_existingProspective->setDoorways(p_passage);
            prospectiveRoomCycles.erase(p_existingProspective->getId());
        }

        semantic::Room *p_currentFarSideHandle =
            p_passage->getProspectiveRoom();
        if ((confirmedAssociatedRoomCount == 1U &&
             p_currentFarSideHandle == p_confirmedAssociatedRoom) ||
            (confirmedAssociatedRoomCount == 2U &&
             p_currentFarSideHandle != nullptr &&
             p_currentFarSideHandle != p_negativeSideRoom &&
             p_currentFarSideHandle != p_positiveSideRoom))
        {
            p_passage->setProspectiveRoom(nullptr);
        }

        if (confirmedAssociatedRoomCount == 2U)
        {
            semantic::Room *p_farSideConfirmedRoom = nullptr;
            if (knownSide.p_room == p_negativeSideRoom)
            {
                p_farSideConfirmedRoom = p_positiveSideRoom;
            }
            else if (knownSide.p_room == p_positiveSideRoom)
            {
                p_farSideConfirmedRoom = p_negativeSideRoom;
            }
            else if (knownSide.hasDirection())
            {
                p_farSideConfirmedRoom = roomIsOnKnownSide(p_negativeSideRoom)
                                             ? p_positiveSideRoom
                                             : p_negativeSideRoom;
            }
            else if (p_negativeExactSupportingOwner != nullptr)
            {
                p_passage->setKnownSideRoom(p_negativeExactSupportingOwner);
                p_passage->setKnownSideDirection(
                    -passageEquation_World.head<3>());
                p_farSideConfirmedRoom = p_positiveSideRoom;
            }
            else if (p_positiveExactSupportingOwner != nullptr)
            {
                p_passage->setKnownSideRoom(p_positiveExactSupportingOwner);
                p_passage->setKnownSideDirection(
                    passageEquation_World.head<3>());
                p_farSideConfirmedRoom = p_negativeSideRoom;
            }
            if (p_farSideConfirmedRoom != nullptr)
            {
                semantic::Room *p_previousFarSideHandle =
                    p_passage->getProspectiveRoom();
                p_passage->setProspectiveRoom(p_farSideConfirmedRoom);
                p_farSideConfirmedRoom->setDoorways(p_passage);
                if (p_previousFarSideHandle != p_farSideConfirmedRoom)
                {
                    std::cout
                        << "[SemMgr] semantic::Passage#" << p_passage->getId()
                        << " resolved to opposite confirmed semantic::Room#"
                        << p_farSideConfirmedRoom->getId()
                        << " with both sides observed." << std::endl;
                }
            }
        }

        if (!p_passage->hasProspectiveRoom() &&
            confirmedAssociatedRoomCount == 1 &&
            p_undefinedAssociatedRoom != nullptr)
        {
            p_passage->setProspectiveRoom(p_undefinedAssociatedRoom);
        }

        if ((confirmedAssociatedRoomCount == 1 ||
             (confirmedAssociatedRoomCount == 0 && knownSide.hasDirection())) &&
            !p_passage->hasProspectiveRoom())
        {
            /* Passage limit: don't create prospective if passage already has 2
             * rooms */
            if (associatedRoomCount >= 2)
            {
                std::cout << "[SemMgr] semantic::Passage#" << p_passage->getId()
                          << " already has 2 associated rooms; skipping "
                             "prospective creation."
                          << std::endl;
            }
            else
            {
                Eigen::Vector4d passageEq =
                    p_passage->getGlobalEquation().coeffs();
                const double normalNorm = passageEq.head<3>().norm();

                if (std::isfinite(normalNorm) && normalNorm > 1e-8)
                {
                    passageEq /= normalNorm;
                    const Eigen::Vector3d passageNormal = passageEq.head<3>();
                    const Eigen::Vector3d passageCentroid =
                        p_passage->getCentroid().cast<double>();

                    /* Persisted provenance, not the current camera pose,
                     * defines the side opposite which the stable handle is
                     * created. */
                    vs_graphs::core::semantic::Room *p_knownRoom =
                        p_confirmedAssociatedRoom;
                    Eigen::Vector3d knownSideDirection =
                        knownSide.hasDirection() ? knownSide.direction_World
                                                 : Eigen::Vector3d::Zero();
                    if (!knownSide.hasDirection() && p_knownRoom != nullptr)
                    {
                        const double knownRoomSide =
                            passageNormal.dot(p_knownRoom->getCentroid()) +
                            passageEq(3);
                        knownSideDirection = knownRoomSide < 0.0
                                                 ? -passageNormal
                                                 : passageNormal;
                        p_passage->setKnownSideDirection(knownSideDirection);
                        p_passage->setKnownSideRoom(p_knownRoom);
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
                    geometric::Plane *p_anteChurnGroundPlane =
                        p_atlas->getBiggestGroundPlane();

                    if (p_knownRoom != nullptr &&
                        p_anteChurnGroundPlane != nullptr &&
                        !p_anteChurnGroundPlane->isBad())
                    {
                        const Eigen::Vector4d anteChurnGroundEq =
                            p_anteChurnGroundPlane->getGlobalEquation()
                                .coeffs();
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
                            const Eigen::Vector3d knownRoomCentroid =
                                p_knownRoom->getCentroid();
                            const std::vector<vs_graphs::core::semantic::Room *>
                                anteChurnExcludedRooms = {p_knownRoom};

                            for (vs_graphs::core::semantic::Room *p_otherRoom :
                                 allRooms)
                            {
                                if (p_otherRoom == nullptr ||
                                    p_otherRoom->isBad() ||
                                    p_otherRoom == p_knownRoom ||
                                    p_otherRoom->getRoomVariant() ==
                                        vs_graphs::core::semantic::Room::
                                            RoomVariant::UNDEFINED)
                                {
                                    continue;
                                }

                                if (!segmentCrossesPassageOpening(
                                        knownRoomCentroid,
                                        p_otherRoom->getCentroid(),
                                        p_passage,
                                        anteChurnGroundNormal_World,
                                        anteChurnOpeningMargin_m,
                                        anteChurnMinimumSideDistance_m))
                                {
                                    continue;
                                }

                                if (segmentCrossesForeignWall(
                                        knownRoomCentroid,
                                        p_otherRoom->getCentroid(),
                                        anteChurnExcludedRooms,
                                        allRooms,
                                        anteChurnGroundAxisU_World,
                                        anteChurnGroundAxisV_World,
                                        anteChurnGroundNormal_World,
                                        anteChurnTopologyParameters
                                            .endpointTrimRatio,
                                        anteChurnTopologyParameters
                                            .minimumWallLength_m))
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
                            p_existingFarSideRoom->setDoorways(p_passage);
                            p_passage->setProspectiveRoom(
                                p_existingFarSideRoom);

                            std::cout << "[SemMgr] semantic::Passage#"
                                      << p_passage->getId()
                                      << " resolved directly to confirmed "
                                         "semantic::Room#"
                                      << p_existingFarSideRoom->getId()
                                      << " on the far side." << std::endl;
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
                        const std::vector<vs_graphs::core::semantic::Room *>
                            candidateRooms = p_atlas->getAllCandidateMapRooms();

                        /* Count current prospective rooms (UNDEFINED variant
                         * candidates) */
                        int prospectiveRoomCount = 0;
                        for (vs_graphs::core::semantic::Room *p_candidate :
                             candidateRooms)
                        {
                            if (p_candidate != nullptr &&
                                !p_candidate->isBad() &&
                                p_candidate->getRoomVariant() ==
                                    vs_graphs::core::semantic::Room::
                                        RoomVariant::UNDEFINED)
                            {
                                prospectiveRoomCount++;
                            }
                        }

                        /* Enforce max prospective rooms cap */
                        if (prospectiveRoomCount >= kMaxProspectiveRooms &&
                            !p_passage->isPassable() &&
                            !p_passage->getTraversalEvidence() &&
                            !hadProspectiveHandle)
                        {
                            std::cout << "[SemMgr] Max prospective rooms ("
                                      << kMaxProspectiveRooms
                                      << ") reached; skipping creation for "
                                         "semantic::Passage#"
                                      << p_passage->getId() << std::endl;
                        }
                        else
                        {
                            for (vs_graphs::core::semantic::Room *p_candidate :
                                 candidateRooms)
                            {
                                if (p_candidate == nullptr ||
                                    p_candidate->isBad() ||
                                    p_candidate->getRoomVariant() !=
                                        vs_graphs::core::semantic::Room::
                                            RoomVariant::UNDEFINED)
                                {
                                    continue;
                                }
                                const Eigen::Vector3d candidateCentroid =
                                    p_candidate->getCentroid();
                                const double distance =
                                    (candidateCentroid - prospectiveCentroid)
                                        .norm();

                                bool passageIdentityMatches = false;
                                if (distance <= kProspectiveDedupDistance_m)
                                {
                                    for (semantic::Passage *p_candidatePassage :
                                         allPassages)
                                    {
                                        if (p_candidatePassage == nullptr ||
                                            p_candidatePassage == p_passage ||
                                            p_candidatePassage
                                                    ->getProspectiveRoom() !=
                                                p_candidate)
                                        {
                                            continue;
                                        }

                                        Eigen::Vector4d candidatePassageEq =
                                            p_candidatePassage
                                                ->getGlobalEquation()
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

                                        const double openingDistance_m =
                                            (p_candidatePassage->getCentroid() -
                                             passageCentroid)
                                                .norm();
                                        const double normalAlignment = std::abs(
                                            candidatePassageEq.head<3>().dot(
                                                passageEq.head<3>()));
                                        const double planeResidual_m =
                                            std::abs(passageEq.head<3>().dot(
                                                         p_candidatePassage
                                                             ->getCentroid()) +
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
                                    p_passage->setProspectiveRoom(p_candidate);
                                    p_candidate->setDoorways(p_passage);
                                    prospectiveExists = true;
                                    std::cout << "[SemMgr] Reusing existing "
                                                 "prospective semantic::Room#"
                                              << p_candidate->getId()
                                              << " (dist=" << distance
                                              << "m) for semantic::Passage#"
                                              << p_passage->getId()
                                              << std::endl;
                                    break;
                                }
                            }

                            if (!prospectiveExists)
                            {
                                /* Create the prospective room */
                                vs_graphs::core::semantic::Room
                                    *p_prospectiveRoom =
                                        GeoSemHelpers::createBlankRoomCandidate(
                                            p_atlas,
                                            prospectiveCentroid);

                                if (p_prospectiveRoom != nullptr)
                                {
                                    /* Mark as provisional - will be promoted
                                     * when walls are observed */
                                    p_prospectiveRoom->setRoomVariant(
                                        vs_graphs::core::semantic::Room::
                                            RoomVariant::UNDEFINED);
                                    p_prospectiveRoom->setName(
                                        "Prospective#" +
                                        std::to_string(
                                            p_prospectiveRoom->getId()));

                                    /* Add to atlas as a candidate (not yet a
                                     * confirmed room) */
                                    p_atlas->addCandidateMapRoom(
                                        p_prospectiveRoom);

                                    /* Link passage <-> prospective room */
                                    p_passage->setProspectiveRoom(
                                        p_prospectiveRoom);
                                    p_prospectiveRoom->setDoorways(p_passage);

                                    /* Register the live passage-created handle.
                                     */
                                    prospectiveRoomCycles[p_prospectiveRoom
                                                              ->getId()] = 0;

                                    std::cout << "[SemMgr] Created prospective "
                                                 "semantic::Room#"
                                              << p_prospectiveRoom->getId()
                                              << " at "
                                              << prospectiveCentroid.transpose()
                                              << " for semantic::Passage#"
                                              << p_passage->getId()
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

        if ((p_passage->isPassable() || p_passage->getTraversalEvidence()) &&
            (confirmedAssociatedRoomCount > 0U || knownSide.hasDirection()) &&
            !p_passage->hasProspectiveRoom())
        {
            std::cerr << "[SemMgr] WARNING: semantic::Passage#"
                      << p_passage->getId()
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
        if (p_passage->hasProspectiveRoom())
        {
            vs_graphs::core::semantic::Room *p_prospectiveRoom =
                p_passage->getProspectiveRoom();

            if (p_prospectiveRoom != nullptr && !p_prospectiveRoom->isBad() &&
                p_prospectiveRoom->getRoomVariant() ==
                    vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED &&
                !p_prospectiveRoom->getWalls().empty())
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
                vs_graphs::core::semantic::Room *p_knownSideRoom =
                    p_passage->getKnownSideProvenance().p_room;

                geometric::Plane *p_groundPlane =
                    p_atlas->getBiggestGroundPlane();

                if (p_groundPlane != nullptr && !p_groundPlane->isBad())
                {
                    const Eigen::Vector4d groundEquation_World =
                        p_groundPlane->getGlobalEquation().coeffs();
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
                        const Eigen::Vector3d prospectiveCentroid =
                            p_prospectiveRoom->getCentroid();
                        const std::vector<vs_graphs::core::semantic::Room *>
                            excludedRooms = {p_prospectiveRoom,
                                             p_knownSideRoom};

                        for (vs_graphs::core::semantic::Room *p_otherRoom :
                             allRooms)
                        {
                            if (p_otherRoom == nullptr ||
                                p_otherRoom->isBad() ||
                                p_otherRoom == p_prospectiveRoom ||
                                p_otherRoom == p_knownSideRoom ||
                                p_otherRoom->getRoomVariant() ==
                                    vs_graphs::core::semantic::Room::
                                        RoomVariant::UNDEFINED)
                            {
                                continue;
                            }

                            if (!segmentCrossesPassageOpening(
                                    prospectiveCentroid,
                                    p_otherRoom->getCentroid(),
                                    p_passage,
                                    groundNormal_World,
                                    openingMargin_m,
                                    minimumSideDistance_m))
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
                            if (segmentCrossesForeignWall(
                                    prospectiveCentroid,
                                    p_otherRoom->getCentroid(),
                                    excludedRooms,
                                    allRooms,
                                    groundAxisU_World,
                                    groundAxisV_World,
                                    groundNormal_World,
                                    topologyParameters.endpointTrimRatio,
                                    topologyParameters.minimumWallLength_m))
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
                    p_passage->setProspectiveRoom(p_farSideConfirmedRoom);
                    for (vs_graphs::core::geometric::Plane *p_wall :
                         p_prospectiveRoom->getWalls())
                    {
                        admitWallToRoom(p_farSideConfirmedRoom, p_wall);
                        p_prospectiveRoom->removeWall(p_wall);
                    }

                    prospectiveRoomCycles.erase(p_prospectiveRoom->getId());

                    Map *p_roomMap = p_prospectiveRoom->getMap();
                    if (p_roomMap != nullptr)
                    {
                        p_roomMap->eraseMarkerBasedMapRoom(p_prospectiveRoom);
                    }

                    p_prospectiveRoom->clearPassages();
                    p_prospectiveRoom->setBad();

                    p_farSideConfirmedRoom->setDoorways(p_passage);

                    std::cout << "[SemMgr] Resolved prospective semantic::Room#"
                              << p_prospectiveRoom->getId()
                              << " with confirmed semantic::Room#"
                              << p_farSideConfirmedRoom->getId()
                              << " for semantic::Passage#" << p_passage->getId()
                              << "." << std::endl;
                }
            }
        }
    }

    /* Report, but never fabricate, missing passage connectivity. */
    std::unordered_set<int> computedDisconnectedRoomIds;

    for (vs_graphs::core::semantic::Room *p_room : allRooms)
    {
        if (p_room == nullptr || p_room->isBad() ||
            p_room->getRoomVariant() ==
                vs_graphs::core::semantic::Room::RoomVariant::UNDEFINED ||
            !p_room->getPassages().empty())
        {
            continue;
        }

        computedDisconnectedRoomIds.insert(p_room->getId());

        if (disconnectedRoomIds.count(p_room->getId()) == 0U)
        {
            std::cout << "[SemMgr] semantic::Room#" << p_room->getId()
                      << " is not yet connected by a confirmed passage; "
                         "semantic routing will treat it as disconnected."
                      << std::endl;
        }
    }

    disconnectedRoomIds = std::move(computedDisconnectedRoomIds);
}

} // namespace core
} // namespace vs_graphs
