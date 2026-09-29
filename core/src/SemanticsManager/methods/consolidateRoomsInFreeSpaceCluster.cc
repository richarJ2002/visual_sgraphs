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

namespace vs_graphs
{
namespace core
{

void SemanticsManager::consolidateRoomsInFreeSpaceCluster(
    vs_graphs::core::semantic::Room    *p_retainedRoom_inout,
    const std::vector<Eigen::Vector3d> &freeSpaceCluster_World_m_in,
    const std::vector<vs_graphs::core::geometric::Plane *> &wallList_World_in)
{
    Map *p_currentMap = p_atlas->getCurrentMap();

    bool retainedRoom_inoutIsBad{};
    if (!(p_retainedRoom_inout == nullptr) &&
        p_retainedRoom_inout->isBad(retainedRoom_inoutIsBad) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // isBad cannot fail; continue as before.
    }
    if (p_retainedRoom_inout == nullptr || retainedRoom_inoutIsBad ||
        p_currentMap == nullptr || freeSpaceCluster_World_m_in.empty())
    {
        return;
    }

    const double maximumClusterSupportDistance_m =
        static_cast<double>(p_sysParams->roomSeg.centerDistanceThresh);
    const double finiteWallBoundsMargin_m =
        static_cast<double>(p_sysParams->roomSeg.finiteWallBoundsMargin_m);
    Eigen::Vector3d retainedCentroid_World_m{};
    if (p_retainedRoom_inout->getCentroid(retainedCentroid_World_m) !=
        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getCentroid cannot fail; continue as before.
    }

    const auto distanceToCluster_m =
        [&freeSpaceCluster_World_m_in](const Eigen::Vector3d &point_World_m_in)
    {
        double minimumDistance_m = std::numeric_limits<double>::infinity();

        for (const Eigen::Vector3d &clusterPoint_World_m :
             freeSpaceCluster_World_m_in)
        {
            if (clusterPoint_World_m.allFinite())
            {
                minimumDistance_m =
                    std::min(minimumDistance_m,
                             (point_World_m_in - clusterPoint_World_m).norm());
            }
        }

        return minimumDistance_m;
    };

    for (vs_graphs::core::semantic::Room *p_duplicateRoom :
         p_currentMap->getAllRooms())
    {
        bool duplicateRoomIsBad{};
        if (!(p_duplicateRoom == nullptr ||
              p_duplicateRoom == p_retainedRoom_inout) &&
            p_duplicateRoom->isBad(duplicateRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_duplicateRoom == nullptr ||
            p_duplicateRoom == p_retainedRoom_inout || duplicateRoomIsBad)
        {
            continue;
        }

        const std::vector<semantic::Passage *> activePassages =
            p_atlas->getAllPassages();
        const bool duplicateIsLiveProspective = std::any_of(
            activePassages.begin(),
            activePassages.end(),
            [p_duplicateRoom](semantic::Passage *p_passage)
            {
                vs_graphs::core::semantic::Room *p_passageProspectiveRoom =
                    nullptr;
                if ((p_passage != nullptr) &&
                    p_passage->getProspectiveRoom(p_passageProspectiveRoom) !=
                        semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getProspectiveRoom cannot fail; continue as before.
                }
                return p_passage != nullptr &&
                       p_passageProspectiveRoom == p_duplicateRoom;
            });

        if (duplicateIsLiveProspective)
        {
            continue;
        }

        semantic::Room::RoomVariant retainedType{};
        if (p_retainedRoom_inout->getRoomVariant(retainedType) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomVariant cannot fail; continue as before.
        }
        semantic::Room::RoomVariant duplicateType{};
        if (p_duplicateRoom->getRoomVariant(duplicateType) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomVariant cannot fail; continue as before.
        }

        if (retainedType != semantic::Room::RoomVariant::UNDEFINED &&
            duplicateType != semantic::Room::RoomVariant::UNDEFINED &&
            retainedType != duplicateType)
        {
            continue;
        }

        bool retainedRoom_inoutHasKnownLabel{};
        if (p_retainedRoom_inout->getHasKnownLabel(
                retainedRoom_inoutHasKnownLabel) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getHasKnownLabel cannot fail; continue as before.
        }
        bool duplicateRoomHasKnownLabel{};
        if ((retainedRoom_inoutHasKnownLabel) &&
            p_duplicateRoom->getHasKnownLabel(duplicateRoomHasKnownLabel) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getHasKnownLabel cannot fail; continue as before.
        }
        int retainedRoom_inoutMetaMarkerId{};
        if ((retainedRoom_inoutHasKnownLabel && duplicateRoomHasKnownLabel) &&
            p_retainedRoom_inout->getMetaMarkerId(
                retainedRoom_inoutMetaMarkerId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getMetaMarkerId cannot fail; continue as before.
        }
        int duplicateRoomMetaMarkerId{};
        if ((retainedRoom_inoutHasKnownLabel && duplicateRoomHasKnownLabel) &&
            p_duplicateRoom->getMetaMarkerId(duplicateRoomMetaMarkerId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getMetaMarkerId cannot fail; continue as before.
        }
        if (retainedRoom_inoutHasKnownLabel && duplicateRoomHasKnownLabel &&
            retainedRoom_inoutMetaMarkerId != duplicateRoomMetaMarkerId)
        {
            continue;
        }

        Eigen::Vector3d duplicateCentroid_World_m{};
        if (p_duplicateRoom->getCentroid(duplicateCentroid_World_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }
        const double centroidDistance_m =
            (duplicateCentroid_World_m - retainedCentroid_World_m).norm();

        /*
         * Do not impose a room-centroid separation limit here. One connected
         * free-space component can legitimately span a large office, and its
         * centroid moves as exploration expands. Membership in that component
         * plus the finite-wall veto is the relevant topological evidence.
         */
        if (!duplicateCentroid_World_m.allFinite() ||
            !std::isfinite(centroidDistance_m) ||
            distanceToCluster_m(duplicateCentroid_World_m) >
                maximumClusterSupportDistance_m ||
            hasSeparatingFiniteWall(wallList_World_in,
                                    retainedCentroid_World_m,
                                    duplicateCentroid_World_m,
                                    finiteWallBoundsMargin_m))
        {
            continue;
        }

        std::vector<semantic::Passage *> retainedPassages{};
        if (p_retainedRoom_inout->getPassages(retainedPassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getPassages cannot fail; continue as before.
        }
        std::vector<semantic::Passage *> duplicatePassages{};
        if (p_duplicateRoom->getPassages(duplicatePassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getPassages cannot fail; continue as before.
        }

        bool              roomsSeparatedByConfirmedPassage = false;
        geometric::Plane *p_groundPlane = p_atlas->getBiggestGroundPlane();

        bool groundPlaneIsBad{};
        if ((p_groundPlane != nullptr) &&
            p_groundPlane->isBad(groundPlaneIsBad) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_groundPlane != nullptr && !groundPlaneIsBad)
        {
            g2o::Plane3D groundPlaneGetGlobalEquation{};
            if (p_groundPlane->getGlobalEquation(
                    groundPlaneGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            Eigen::Vector4d groundEquation_World =
                groundPlaneGetGlobalEquation.coeffs();
            const double groundNormalNorm =
                groundEquation_World.head<3>().norm();

            if (groundEquation_World.allFinite() && groundNormalNorm > 1e-8)
            {
                const Eigen::Vector3d groundNormal_World =
                    groundEquation_World.head<3>() / groundNormalNorm;
                const std::vector<semantic::Passage *> confirmedPassages =
                    p_atlas->getAllPassages();

                roomsSeparatedByConfirmedPassage =
                    std::any_of(confirmedPassages.begin(),
                                confirmedPassages.end(),
                                [&retainedCentroid_World_m,
                                 &duplicateCentroid_World_m,
                                 &groundNormal_World,
                                 this](semantic::Passage *p_passage)
                                {
                                    return segmentCrossesPassageOpening(
                                        retainedCentroid_World_m,
                                        duplicateCentroid_World_m,
                                        p_passage,
                                        groundNormal_World,
                                        p_sysParams->roomSeg.passagePartition
                                            .openingMargin_m,
                                        p_sysParams->roomSeg.passagePartition
                                            .minimumSideDistance_m);
                                });
            }
        }

        if (roomsSeparatedByConfirmedPassage)
        {
            continue;
        }

        /*
         * A shared passage only proves that two hypotheses represent adjacent
         * rooms when their centroids lie on opposite sides of the passage
         * plane. During incremental mapping, the same opening may temporarily
         * be associated with two duplicate hypotheses on the same side. Using
         * passage identity alone would then preserve the duplicate forever.
         */
        constexpr double minimumPassageSideDistance_m = 0.20;

        const bool roomsSeparatedBySharedPassage = std::any_of(
            retainedPassages.begin(),
            retainedPassages.end(),
            [&duplicatePassages,
             &retainedCentroid_World_m,
             &duplicateCentroid_World_m](semantic::Passage *p_sharedPassage_in)
            {
                if (p_sharedPassage_in == nullptr ||
                    std::find(duplicatePassages.begin(),
                              duplicatePassages.end(),
                              p_sharedPassage_in) == duplicatePassages.end())
                {
                    return false;
                }

                g2o::Plane3D sharedPassage_inGlobalEquation{};
                if (p_sharedPassage_in->getGlobalEquation(
                        sharedPassage_inGlobalEquation) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getGlobalEquation cannot fail; continue as before.
                }
                Eigen::Vector4d passageEquation_World =
                    sharedPassage_inGlobalEquation.coeffs();

                const double passageNormalNorm =
                    passageEquation_World.head<3>().norm();

                if (!passageEquation_World.allFinite() ||
                    passageNormalNorm < 1e-8)
                {
                    return false;
                }

                passageEquation_World /= passageNormalNorm;

                const double retainedSide_m =
                    passageEquation_World.head<3>().dot(
                        retainedCentroid_World_m) +
                    passageEquation_World(3);

                const double duplicateSide_m =
                    passageEquation_World.head<3>().dot(
                        duplicateCentroid_World_m) +
                    passageEquation_World(3);

                return retainedSide_m * duplicateSide_m < 0.0 &&
                       std::abs(retainedSide_m) >=
                           minimumPassageSideDistance_m &&
                       std::abs(duplicateSide_m) >=
                           minimumPassageSideDistance_m;
            });

        if (roomsSeparatedBySharedPassage)
        {
            continue;
        }

        std::vector<
            std::pair<semantic::Room *, std::vector<geometric::Plane *>>>
            wallSnapshots;
        for (semantic::Room *p_snapshotRoom : p_currentMap->getAllRooms())
        {
            bool snapshotRoomIsBad{};
            if ((p_snapshotRoom != nullptr) &&
                p_snapshotRoom->isBad(snapshotRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_snapshotRoom != nullptr && !snapshotRoomIsBad)
            {
                std::vector<geometric::Plane *> snapshotRoomWalls{};
                if (p_snapshotRoom->getWalls(snapshotRoomWalls) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getWalls cannot fail; continue as before.
                }
                wallSnapshots.emplace_back(p_snapshotRoom, snapshotRoomWalls);
            }
        }

        bool allDuplicateWallsWereAdmitted = true;

        std::vector<geometric::Plane *> duplicateRoomWalls{};
        if (p_duplicateRoom->getWalls(duplicateRoomWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }
        for (geometric::Plane *p_wall : duplicateRoomWalls)
        {
            if (!admitWallToRoom(p_retainedRoom_inout, p_wall))
            {
                allDuplicateWallsWereAdmitted = false;
                break;
            }
        }

        /*
         * A topology-rejected provisional wall is not a duplicate room. Keep
         * that structural element alive so another free-space component can
         * claim it instead of repeatedly deleting and recreating it.
         */
        if (!allDuplicateWallsWereAdmitted)
        {
            for (auto &[p_snapshotRoom, snapshotWalls] : wallSnapshots)
            {
                if (p_snapshotRoom->clearWalls() !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // clearWalls cannot fail; continue as before.
                }
                for (geometric::Plane *p_snapshotWall : snapshotWalls)
                {
                    if (p_snapshotRoom->setWalls(p_snapshotWall) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        // setWalls cannot fail; continue as before.
                    }
                }
            }
            continue;
        }

        for (semantic::Passage *p_passage : duplicatePassages)
        {
            if (p_retainedRoom_inout->setDoorways(p_passage) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setDoorways cannot fail; continue as before.
            }
        }

        geometric::Plane *p_retainedRoom_inoutGroundPlane = nullptr;
        if (p_retainedRoom_inout->getGroundPlane(
                p_retainedRoom_inoutGroundPlane) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getGroundPlane cannot fail; continue as before.
        }
        if (p_retainedRoom_inoutGroundPlane == nullptr)
        {
            geometric::Plane *p_duplicateRoomGroundPlane = nullptr;
            if (p_duplicateRoom->getGroundPlane(p_duplicateRoomGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getGroundPlane cannot fail; continue as before.
            }
            if (p_retainedRoom_inout->setGroundPlane(
                    p_duplicateRoomGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setGroundPlane cannot fail; continue as before.
            }
        }

        bool retainedRoom_inoutHasKnownLabel2{};
        if (p_retainedRoom_inout->getHasKnownLabel(
                retainedRoom_inoutHasKnownLabel2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getHasKnownLabel cannot fail; continue as before.
        }
        bool duplicateRoomHasKnownLabel2{};
        if ((!retainedRoom_inoutHasKnownLabel2) &&
            p_duplicateRoom->getHasKnownLabel(duplicateRoomHasKnownLabel2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getHasKnownLabel cannot fail; continue as before.
        }
        if (!retainedRoom_inoutHasKnownLabel2 && duplicateRoomHasKnownLabel2)
        {
            if (p_retainedRoom_inout->setHasKnownLabel(true) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setHasKnownLabel cannot fail; continue as before.
            }
            semantic::Marker *p_duplicateRoomMetaMarker = nullptr;
            if (p_duplicateRoom->getMetaMarker(p_duplicateRoomMetaMarker) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getMetaMarker cannot fail; continue as before.
            }
            if (p_retainedRoom_inout->setMetaMarker(
                    p_duplicateRoomMetaMarker) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setMetaMarker cannot fail; continue as before.
            }
            int duplicateRoomMetaMarkerId2{};
            if (p_duplicateRoom->getMetaMarkerId(duplicateRoomMetaMarkerId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getMetaMarkerId cannot fail; continue as before.
            }
            if (p_retainedRoom_inout->setMetaMarkerId(
                    duplicateRoomMetaMarkerId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setMetaMarkerId cannot fail; continue as before.
            }
            std::string duplicateRoomName{};
            if (p_duplicateRoom->getName(duplicateRoomName) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getName cannot fail; continue as before.
            }
            if (p_retainedRoom_inout->setName(duplicateRoomName) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setName cannot fail; continue as before.
            }
        }

        if (retainedType == semantic::Room::RoomVariant::UNDEFINED &&
            duplicateType != semantic::Room::RoomVariant::UNDEFINED)
        {
            if (p_retainedRoom_inout->setRoomVariant(duplicateType) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setRoomVariant cannot fail; continue as before.
            }
        }

        for (semantic::Floor *p_floor : p_currentMap->getAllFloors())
        {
            if (p_floor != nullptr)
            {
                bool floorWasRoomReplaced{};
                if (p_floor->replaceRoom(p_duplicateRoom,
                                         p_retainedRoom_inout,
                                         floorWasRoomReplaced) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    floorWasRoomReplaced =
                        false; // rejected input reads as before
                }
            }
        }

        p_currentMap->eraseDetectedMapRoom(p_duplicateRoom);
        p_currentMap->eraseMarkerBasedMapRoom(p_duplicateRoom);
        if (p_duplicateRoom->clearWalls() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // clearWalls cannot fail; continue as before.
        }
        if (p_duplicateRoom->clearPassages() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // clearPassages cannot fail; continue as before.
        }
        if (p_duplicateRoom->setBad() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setBad cannot fail; continue as before.
        }

        int duplicateRoomId{};
        if (p_duplicateRoom->getId(duplicateRoomId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        int retainedRoom_inoutId{};
        if (p_retainedRoom_inout->getId(retainedRoom_inoutId) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        std::cout << "[SemMgr] Fused semantic::Room#" << duplicateRoomId
                  << " into semantic::Room#" << retainedRoom_inoutId
                  << " using connected free-space evidence (centroid distance "
                  << centroidDistance_m << " m)." << std::endl;
    }
}

} // namespace core
} // namespace vs_graphs
