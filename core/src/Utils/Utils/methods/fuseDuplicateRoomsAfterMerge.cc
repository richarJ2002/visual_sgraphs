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
 * @file            fuseDuplicateRoomsAfterMerge.cc
 *
 * @brief           Implements
 *                  Utils::fuseDuplicateRoomsAfterMerge(), declared in
 *                  Utils/Utils/objects/Utils.h.
 */

#include "Utils/Utils/objects/Utils.h"
#include "Utils/Utils/private_functions.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_set>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace utils
{

void Utils::fuseDuplicateRoomsAfterMerge(
    Map                                 *p_map_inout,
    const std::vector<semantic::Room *> &importedRooms_in)
{
    /* Reject an invalid lifecycle request. */
    if (p_map_inout == nullptr || importedRooms_in.empty())
    {
        return;
    }

    const types::SystemParams *p_systemParameters =
        types::SystemParams::getParams();

    const double maximumRoomCentroidDistance_m =
        p_systemParameters != nullptr
            ? static_cast<double>(
                  p_systemParameters->roomSeg.centerDistanceThresh)
            : 2.0;

    constexpr double minimumRoomSideDistance_m = 0.20;
    constexpr double finiteWallBoundsMargin_m  = 0.30;

    const std::unordered_set<semantic::Room *> importedRoomSet(
        importedRooms_in.begin(),
        importedRooms_in.end());

    /*
     * Report whether a finite mapped wall separates two room centres. An
     * infinite plane alone is insufficient because unrelated coplanar wall
     * segments are common in office environments.
     */
    const auto hasSeparatingFiniteWall =
        [p_map_inout](const Eigen::Vector3d &firstCentroid_World_m_in,
                      const Eigen::Vector3d &secondCentroid_World_m_in)
    {
        for (geometric::Plane *p_wall : p_map_inout->getAllPlanes())
        {
            if (p_wall == nullptr || p_wall->isBad() ||
                p_wall->getPlaneType() != geometric::Plane::PlaneVariant::WALL)
            {
                continue;
            }

            const geometric::Plane::GeometrySnapshot wallGeometry =
                p_wall->getGeometrySnapshot();
            Eigen::Vector4d wallEquation_World = wallGeometry.equation_World;

            const double wallNormalNorm = wallEquation_World.head<3>().norm();

            if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
            {
                continue;
            }

            wallEquation_World /= wallNormalNorm;

            const double firstSignedDistance_m =
                wallEquation_World.head<3>().dot(firstCentroid_World_m_in) +
                wallEquation_World(3);

            const double secondSignedDistance_m =
                wallEquation_World.head<3>().dot(secondCentroid_World_m_in) +
                wallEquation_World(3);

            if (firstSignedDistance_m * secondSignedDistance_m >= 0.0 ||
                std::abs(firstSignedDistance_m) < minimumRoomSideDistance_m ||
                std::abs(secondSignedDistance_m) < minimumRoomSideDistance_m)
            {
                continue;
            }

            const double interpolation =
                firstSignedDistance_m /
                (firstSignedDistance_m - secondSignedDistance_m);

            if (interpolation <= 0.0 || interpolation >= 1.0)
            {
                continue;
            }

            const Eigen::Vector3d intersection_World_m =
                firstCentroid_World_m_in +
                interpolation *
                    (secondCentroid_World_m_in - firstCentroid_World_m_in);

            const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr p_wallCloud =
                wallGeometry.supportCloud;

            if (p_wallCloud == nullptr || p_wallCloud->empty())
            {
                continue;
            }

            const Eigen::Vector3d wallCentroid_World_m =
                wallGeometry.centroid_World_m;

            const Eigen::Vector3d wallAxisU_World =
                wallEquation_World.head<3>().unitOrthogonal().normalized();

            const Eigen::Vector3d wallAxisV_World = wallEquation_World.head<3>()
                                                        .cross(wallAxisU_World)
                                                        .normalized();

            double      minimumWallU_m = std::numeric_limits<double>::max();
            double      maximumWallU_m = std::numeric_limits<double>::lowest();
            double      minimumWallV_m = std::numeric_limits<double>::max();
            double      maximumWallV_m = std::numeric_limits<double>::lowest();
            std::size_t validWallPointCount = 0U;

            for (const pcl::PointXYZRGBA &wallPoint : p_wallCloud->points)
            {
                if (!pcl::isFinite(wallPoint))
                {
                    continue;
                }

                const Eigen::Vector3d wallPoint_World_m(
                    static_cast<double>(wallPoint.x),
                    static_cast<double>(wallPoint.y),
                    static_cast<double>(wallPoint.z));

                const Eigen::Vector3d wallPointRelToCentroid_World_m =
                    wallPoint_World_m - wallCentroid_World_m;

                const double wallPointU_m =
                    wallPointRelToCentroid_World_m.dot(wallAxisU_World);

                const double wallPointV_m =
                    wallPointRelToCentroid_World_m.dot(wallAxisV_World);

                minimumWallU_m = std::min(minimumWallU_m, wallPointU_m);
                maximumWallU_m = std::max(maximumWallU_m, wallPointU_m);
                minimumWallV_m = std::min(minimumWallV_m, wallPointV_m);
                maximumWallV_m = std::max(maximumWallV_m, wallPointV_m);
                validWallPointCount++;
            }

            if (validWallPointCount == 0U)
            {
                continue;
            }

            const Eigen::Vector3d intersectionRelToWallCentroid_World_m =
                intersection_World_m - wallCentroid_World_m;

            const double intersectionU_m =
                intersectionRelToWallCentroid_World_m.dot(wallAxisU_World);

            const double intersectionV_m =
                intersectionRelToWallCentroid_World_m.dot(wallAxisV_World);

            const bool intersectionInsideFiniteWall =
                intersectionU_m >= minimumWallU_m - finiteWallBoundsMargin_m &&
                intersectionU_m <= maximumWallU_m + finiteWallBoundsMargin_m &&
                intersectionV_m >= minimumWallV_m - finiteWallBoundsMargin_m &&
                intersectionV_m <= maximumWallV_m + finiteWallBoundsMargin_m;

            if (intersectionInsideFiniteWall)
            {
                return true;
            }
        }

        return false;
    };

    /* Evaluate each imported hypothesis against rooms that already existed. */
    for (semantic::Room *p_importedRoom : importedRooms_in)
    {
        if (p_importedRoom == nullptr || p_importedRoom->isBad())
        {
            continue;
        }

        const semantic::Room::RoomVariant importedRoomType =
            p_importedRoom->getRoomVariant();

        if (importedRoomType == semantic::Room::RoomVariant::UNDEFINED)
        {
            continue;
        }

        const Eigen::Vector3d importedCentroid_World_m =
            p_importedRoom->getCentroid();

        if (!importedCentroid_World_m.allFinite())
        {
            continue;
        }

        const std::vector<geometric::Plane *> importedWalls =
            p_importedRoom->getWalls();

        semantic::Room *p_bestRetainedRoom = nullptr;
        double bestCentroidDistance_m = std::numeric_limits<double>::infinity();

        for (semantic::Room *p_candidateRoom : p_map_inout->getAllRooms())
        {
            if (p_candidateRoom == nullptr ||
                p_candidateRoom == p_importedRoom || p_candidateRoom->isBad() ||
                importedRoomSet.count(p_candidateRoom) > 0U ||
                p_candidateRoom->getRoomVariant() != importedRoomType)
            {
                continue;
            }

            const std::string importedIdentity  = p_importedRoom->getRoomTag();
            const std::string candidateIdentity = p_candidateRoom->getRoomTag();
            const bool        identitiesMatch =
                !importedIdentity.empty() && !candidateIdentity.empty()
                           ? importedIdentity == candidateIdentity
                           : p_importedRoom->getId() == p_candidateRoom->getId();
            if (identitiesMatch)
            {
                p_bestRetainedRoom = p_candidateRoom;
                bestCentroidDistance_m =
                    (p_candidateRoom->getCentroid() - importedCentroid_World_m)
                        .norm();
                break;
            }

            /* Geometry-only fusion remains a compatibility fallback solely
             * for legacy unnumbered rooms. Mission rooms always carry a
             * non-negative stable ID and must never cross identities. */
            if (p_importedRoom->getId() >= 0 || p_candidateRoom->getId() >= 0)
            {
                continue;
            }

            if (p_importedRoom->getHasKnownLabel() &&
                p_candidateRoom->getHasKnownLabel() &&
                p_importedRoom->getMetaMarkerId() !=
                    p_candidateRoom->getMetaMarkerId())
            {
                continue;
            }

            const Eigen::Vector3d candidateCentroid_World_m =
                p_candidateRoom->getCentroid();

            const double centroidDistance_m =
                (candidateCentroid_World_m - importedCentroid_World_m).norm();

            if (!candidateCentroid_World_m.allFinite() ||
                !std::isfinite(centroidDistance_m) ||
                centroidDistance_m > maximumRoomCentroidDistance_m ||
                centroidDistance_m >= bestCentroidDistance_m)
            {
                continue;
            }

            const std::vector<geometric::Plane *> candidateWalls =
                p_candidateRoom->getWalls();

            std::size_t sameSideSharedWallCount   = 0U;
            bool        hasOppositeSideSharedWall = false;

            for (geometric::Plane *p_importedWall : importedWalls)
            {
                if (p_importedWall == nullptr || p_importedWall->isBad())
                {
                    continue;
                }

                const bool wallIsShared =
                    std::find(candidateWalls.begin(),
                              candidateWalls.end(),
                              p_importedWall) != candidateWalls.end();

                if (!wallIsShared)
                {
                    continue;
                }

                Eigen::Vector4d wallEquation_World =
                    p_importedWall->getGlobalEquation().coeffs();

                const double wallNormalNorm =
                    wallEquation_World.head<3>().norm();

                if (!wallEquation_World.allFinite() || wallNormalNorm < 1e-8)
                {
                    continue;
                }

                wallEquation_World /= wallNormalNorm;

                const double importedSide_m =
                    wallEquation_World.head<3>().dot(importedCentroid_World_m) +
                    wallEquation_World(3);

                const double candidateSide_m = wallEquation_World.head<3>().dot(
                                                   candidateCentroid_World_m) +
                                               wallEquation_World(3);

                if (importedSide_m * candidateSide_m < 0.0 &&
                    std::abs(importedSide_m) >= minimumRoomSideDistance_m &&
                    std::abs(candidateSide_m) >= minimumRoomSideDistance_m)
                {
                    hasOppositeSideSharedWall = true;
                    break;
                }

                sameSideSharedWallCount++;
            }

            if (hasOppositeSideSharedWall || sameSideSharedWallCount == 0U ||
                hasSeparatingFiniteWall(importedCentroid_World_m,
                                        candidateCentroid_World_m))
            {
                continue;
            }

            p_bestRetainedRoom     = p_candidateRoom;
            bestCentroidDistance_m = centroidDistance_m;
        }

        if (p_bestRetainedRoom == nullptr)
        {
            continue;
        }

        const std::vector<geometric::Plane *> retainedWalls =
            p_bestRetainedRoom->getWalls();

        /*! A wall can bound the retained (near) room only when no passable
         * passage aperture separates its centroid from the retained room
         * centre. Otherwise it belongs to the far-side room (the passage's
         * prospective, or a confirmed room that already resolved that
         * prospective). Copying such a wall into the retained room would both
         * corrupt the near boundary and hand the far room's evidence to the
         * near room, so the far-side consultation happens here. */
        geometric::Plane *p_mergeGroundPlane = nullptr;

        for (geometric::Plane *p_plane : p_map_inout->getAllPlanes())
        {
            if (p_plane != nullptr && !p_plane->isBad() &&
                p_plane->getPlaneType() ==
                    geometric::Plane::PlaneVariant::GROUND)
            {
                p_mergeGroundPlane = p_plane;
                break;
            }
        }

        Eigen::Vector3d mergeGroundNormal_World = Eigen::Vector3d::Zero();
        if (p_mergeGroundPlane != nullptr)
        {
            const Eigen::Vector4d groundEq =
                p_mergeGroundPlane->getGlobalEquation().coeffs();
            const double groundNormalNorm = groundEq.head<3>().norm();
            if (groundEq.allFinite() && groundNormalNorm > 1e-8)
            {
                mergeGroundNormal_World = groundEq.head<3>() / groundNormalNorm;
            }
        }

        const double mergeOpeningMargin_m =
            p_systemParameters != nullptr
                ? static_cast<double>(p_systemParameters->roomSeg
                                          .passagePartition.openingMargin_m)
                : 0.20;

        const double mergeMinimumSideDistance_m =
            p_systemParameters != nullptr
                ? static_cast<double>(
                      p_systemParameters->roomSeg.passagePartition
                          .minimumSideDistance_m)
                : 0.30;

        const Eigen::Vector3d retainedCentroid_World_m =
            p_bestRetainedRoom->getCentroid();

        const std::vector<semantic::Passage *> mergePassages =
            p_map_inout->getAllPassages();
        const bool roomsAreSeparatedByPassage = std::any_of(
            mergePassages.begin(),
            mergePassages.end(),
            [&retainedCentroid_World_m,
             &importedCentroid_World_m,
             &mergeGroundNormal_World,
             mergeOpeningMargin_m,
             mergeMinimumSideDistance_m](semantic::Passage *p_passage)
            {
                return crossesPassablePassageOpening(
                    retainedCentroid_World_m,
                    importedCentroid_World_m,
                    p_passage,
                    mergeGroundNormal_World,
                    mergeOpeningMargin_m,
                    mergeMinimumSideDistance_m);
            });

        if (roomsAreSeparatedByPassage)
        {
            std::cout << "[SemanticMerge] Preserved semantic::Room#"
                      << p_importedRoom->getId() << " and semantic::Room#"
                      << p_bestRetainedRoom->getId()
                      << "; a passable passage separates their centroids."
                      << std::endl;
            continue;
        }

        struct WallTransfer
        {
            geometric::Plane  *p_wall;
            semantic::Room    *p_targetRoom;
            semantic::Passage *p_separatingPassage;
        };

        std::vector<WallTransfer> wallTransfers;
        wallTransfers.reserve(importedWalls.size());
        std::vector<semantic::Room *> mapRooms = p_map_inout->getAllRooms();
        std::sort(
            mapRooms.begin(),
            mapRooms.end(),
            [](const semantic::Room *p_first, const semantic::Room *p_second)
            {
                if (p_first == nullptr)
                {
                    return false;
                }
                if (p_second == nullptr)
                {
                    return true;
                }
                return p_first->getId() < p_second->getId();
            });

        for (geometric::Plane *p_importedWall : importedWalls)
        {
            if (p_importedWall == nullptr || p_importedWall->isBad())
            {
                continue;
            }

            const Eigen::Vector3d importedWallCentroid_World_m =
                p_importedWall->getCentroid().cast<double>();

            vs_graphs::core::semantic::Passage *p_separatingPassage = nullptr;

            for (vs_graphs::core::semantic::Passage *p_passage : mergePassages)
            {
                if (p_passage != nullptr &&
                    crossesPassablePassageOpening(retainedCentroid_World_m,
                                                  importedWallCentroid_World_m,
                                                  p_passage,
                                                  mergeGroundNormal_World,
                                                  mergeOpeningMargin_m,
                                                  mergeMinimumSideDistance_m))
                {
                    p_separatingPassage = p_passage;
                    break;
                }
            }

            semantic::Room *p_targetRoom = p_bestRetainedRoom;

            /* Never introduce another owner when a distinct confirmed room
             * already owns this wall. The imported duplicate is retired below,
             * leaving that confirmed owner unchanged. */
            for (semantic::Room *p_existingOwner : mapRooms)
            {
                if (p_existingOwner == nullptr || p_existingOwner->isBad() ||
                    p_existingOwner == p_importedRoom ||
                    p_existingOwner == p_bestRetainedRoom ||
                    p_existingOwner->getRoomVariant() !=
                        semantic::Room::RoomVariant::ROOM)
                {
                    continue;
                }

                const std::vector<geometric::Plane *> ownerWalls =
                    p_existingOwner->getWalls();
                if (std::find(ownerWalls.begin(),
                              ownerWalls.end(),
                              p_importedWall) != ownerWalls.end())
                {
                    p_targetRoom = p_existingOwner;
                    break;
                }
            }

            if (p_separatingPassage != nullptr &&
                p_targetRoom == p_bestRetainedRoom)
            {
                vs_graphs::core::semantic::Room *p_farSideRoom =
                    p_separatingPassage->getProspectiveRoom();

                if (p_farSideRoom == nullptr || p_farSideRoom->isBad() ||
                    p_farSideRoom == p_importedRoom)
                {
                    p_farSideRoom = nullptr;
                }

                p_targetRoom = p_farSideRoom;
            }

            wallTransfers.push_back(
                {p_importedWall, p_targetRoom, p_separatingPassage});
        }

        /* Commit the precomputed ownership transaction. Removing the duplicate
         * edge before adding its destination prevents transient double
         * ownership inside the authorized room fusion. */
        for (const WallTransfer &transfer : wallTransfers)
        {
            p_importedRoom->removeWall(transfer.p_wall);

            if (transfer.p_targetRoom == nullptr)
            {
                std::cout << "[SemanticMerge] Far-side Wall#"
                          << transfer.p_wall->getId()
                          << " at semantic::Passage#"
                          << transfer.p_separatingPassage->getId()
                          << " has no prospective; left unbound." << std::endl;
                continue;
            }

            transfer.p_targetRoom->setWalls(transfer.p_wall);

            if (transfer.p_separatingPassage != nullptr &&
                transfer.p_targetRoom != p_bestRetainedRoom)
            {
                std::cout << "[SemanticMerge] Redirected far-side Wall#"
                          << transfer.p_wall->getId()
                          << " to stable semantic::Room#"
                          << transfer.p_targetRoom->getId() << "." << std::endl;
            }
        }

        for (vs_graphs::core::semantic::Passage *p_importedPassage :
             p_importedRoom->getPassages())
        {
            p_bestRetainedRoom->setDoorways(p_importedPassage);
        }

        for (semantic::Passage *p_passage : p_map_inout->getAllPassages())
        {
            if (p_passage != nullptr)
            {
                p_passage->replaceProspectiveRoom(p_importedRoom,
                                                  p_bestRetainedRoom);
            }
        }

        const double retainedWeight = static_cast<double>(
            std::max<std::size_t>(retainedWalls.size(), 1U));

        const double importedWeight = static_cast<double>(
            std::max<std::size_t>(importedWalls.size(), 1U));

        const Eigen::Vector3d fusedCentroid_World_m =
            (retainedWeight * p_bestRetainedRoom->getCentroid() +
             importedWeight * importedCentroid_World_m) /
            (retainedWeight + importedWeight);

        p_bestRetainedRoom->setCentroid(fusedCentroid_World_m);

        if (p_bestRetainedRoom->getGroundPlane() == nullptr)
        {
            p_bestRetainedRoom->setGroundPlane(
                p_importedRoom->getGroundPlane());
        }

        if (!p_bestRetainedRoom->getHasKnownLabel() &&
            p_importedRoom->getHasKnownLabel())
        {
            p_bestRetainedRoom->setHasKnownLabel(true);
            p_bestRetainedRoom->setMetaMarker(p_importedRoom->getMetaMarker());
            p_bestRetainedRoom->setMetaMarkerId(
                p_importedRoom->getMetaMarkerId());
            p_bestRetainedRoom->setName(p_importedRoom->getName());
        }

        /* Presence on either side proves the UAV has been there: a room fused
         * from a visited duplicate stays visited. */
        if (p_importedRoom->hasPreviouslyVisited())
        {
            p_bestRetainedRoom->setPreviouslyVisited(true);
        }

        for (semantic::Floor *p_floor : p_map_inout->getAllFloors())
        {
            if (p_floor != nullptr)
            {
                p_floor->replaceRoom(p_importedRoom, p_bestRetainedRoom);
            }
        }

        p_map_inout->eraseDetectedMapRoom(p_importedRoom);
        p_map_inout->eraseMarkerBasedMapRoom(p_importedRoom);
        p_importedRoom->clearWalls();
        p_importedRoom->clearPassages();
        p_importedRoom->setBad();

        std::cout << "[SemanticMerge] Fused duplicate semantic::Room#"
                  << p_importedRoom->getId() << " into semantic::Room#"
                  << p_bestRetainedRoom->getId() << " (centroid distance "
                  << bestCentroidDistance_m << " m)." << std::endl;
    }
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
