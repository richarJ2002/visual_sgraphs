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

    if (p_retainedRoom_inout == nullptr || p_retainedRoom_inout->isBad() ||
        p_currentMap == nullptr || freeSpaceCluster_World_m_in.empty())
    {
        return;
    }

    const double maximumClusterSupportDistance_m =
        static_cast<double>(p_sysParams->roomSeg.centerDistanceThresh);
    const double finiteWallBoundsMargin_m =
        static_cast<double>(p_sysParams->roomSeg.finiteWallBoundsMargin_m);
    const Eigen::Vector3d retainedCentroid_World_m =
        p_retainedRoom_inout->getCentroid();

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
        if (p_duplicateRoom == nullptr ||
            p_duplicateRoom == p_retainedRoom_inout || p_duplicateRoom->isBad())
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
                return p_passage != nullptr &&
                       p_passage->getProspectiveRoom() == p_duplicateRoom;
            });

        if (duplicateIsLiveProspective)
        {
            continue;
        }

        const semantic::Room::RoomVariant retainedType =
            p_retainedRoom_inout->getRoomVariant();
        const semantic::Room::RoomVariant duplicateType =
            p_duplicateRoom->getRoomVariant();

        if (retainedType != semantic::Room::RoomVariant::UNDEFINED &&
            duplicateType != semantic::Room::RoomVariant::UNDEFINED &&
            retainedType != duplicateType)
        {
            continue;
        }

        if (p_retainedRoom_inout->getHasKnownLabel() &&
            p_duplicateRoom->getHasKnownLabel() &&
            p_retainedRoom_inout->getMetaMarkerId() !=
                p_duplicateRoom->getMetaMarkerId())
        {
            continue;
        }

        const Eigen::Vector3d duplicateCentroid_World_m =
            p_duplicateRoom->getCentroid();
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

        const std::vector<semantic::Passage *> retainedPassages =
            p_retainedRoom_inout->getPassages();
        const std::vector<semantic::Passage *> duplicatePassages =
            p_duplicateRoom->getPassages();

        bool              roomsSeparatedByConfirmedPassage = false;
        geometric::Plane *p_groundPlane = p_atlas->getBiggestGroundPlane();

        if (p_groundPlane != nullptr && !p_groundPlane->isBad())
        {
            Eigen::Vector4d groundEquation_World =
                p_groundPlane->getGlobalEquation().coeffs();
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

                Eigen::Vector4d passageEquation_World =
                    p_sharedPassage_in->getGlobalEquation().coeffs();

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
            if (p_snapshotRoom != nullptr && !p_snapshotRoom->isBad())
            {
                wallSnapshots.emplace_back(p_snapshotRoom,
                                           p_snapshotRoom->getWalls());
            }
        }

        bool allDuplicateWallsWereAdmitted = true;

        for (geometric::Plane *p_wall : p_duplicateRoom->getWalls())
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
                p_snapshotRoom->clearWalls();
                for (geometric::Plane *p_snapshotWall : snapshotWalls)
                {
                    p_snapshotRoom->setWalls(p_snapshotWall);
                }
            }
            continue;
        }

        for (semantic::Passage *p_passage : duplicatePassages)
        {
            p_retainedRoom_inout->setDoorways(p_passage);
        }

        if (p_retainedRoom_inout->getGroundPlane() == nullptr)
        {
            p_retainedRoom_inout->setGroundPlane(
                p_duplicateRoom->getGroundPlane());
        }

        if (!p_retainedRoom_inout->getHasKnownLabel() &&
            p_duplicateRoom->getHasKnownLabel())
        {
            p_retainedRoom_inout->setHasKnownLabel(true);
            p_retainedRoom_inout->setMetaMarker(
                p_duplicateRoom->getMetaMarker());
            p_retainedRoom_inout->setMetaMarkerId(
                p_duplicateRoom->getMetaMarkerId());
            p_retainedRoom_inout->setName(p_duplicateRoom->getName());
        }

        if (retainedType == semantic::Room::RoomVariant::UNDEFINED &&
            duplicateType != semantic::Room::RoomVariant::UNDEFINED)
        {
            p_retainedRoom_inout->setRoomVariant(duplicateType);
        }

        for (semantic::Floor *p_floor : p_currentMap->getAllFloors())
        {
            if (p_floor != nullptr)
            {
                p_floor->replaceRoom(p_duplicateRoom, p_retainedRoom_inout);
            }
        }

        p_currentMap->eraseDetectedMapRoom(p_duplicateRoom);
        p_currentMap->eraseMarkerBasedMapRoom(p_duplicateRoom);
        p_duplicateRoom->clearWalls();
        p_duplicateRoom->clearPassages();
        p_duplicateRoom->setBad();

        std::cout << "[SemMgr] Fused semantic::Room#"
                  << p_duplicateRoom->getId() << " into semantic::Room#"
                  << p_retainedRoom_inout->getId()
                  << " using connected free-space evidence (centroid distance "
                  << centroidDistance_m << " m)." << std::endl;
    }
}

} // namespace core
} // namespace vs_graphs
