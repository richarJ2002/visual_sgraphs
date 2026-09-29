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

UtilsStatus Utils::fuseDuplicateRoomsAfterMerge(
    Map                                 *p_map_inout,
    const std::vector<semantic::Room *> &importedRooms_in)
{
    /* Reject an invalid lifecycle request. */
    if (p_map_inout == nullptr || importedRooms_in.empty())
    {
        return UtilsStatus::UTILS_STATUS_SUCCESS;
    }

    types::SystemParams *p_systemParameters = nullptr;
    if (types::SystemParams::getParams(p_systemParameters) !=
        types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS)
    {
        // getParams cannot fail; continue as before.
    }

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
            bool wallIsBad{};
            if (!(p_wall == nullptr) &&
                p_wall->isBad(wallIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            geometric::Plane::PlaneVariant wallPlaneType{};
            if (!(p_wall == nullptr || wallIsBad) &&
                p_wall->getPlaneType(wallPlaneType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getPlaneType cannot fail; continue as before.
            }
            if (p_wall == nullptr || wallIsBad ||
                wallPlaneType != geometric::Plane::PlaneVariant::WALL)
            {
                continue;
            }

            geometric::Plane::GeometrySnapshot wallGeometry{};
            if (p_wall->getGeometrySnapshot(wallGeometry) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getGeometrySnapshot cannot fail; continue as before.
            }
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
        bool importedRoomIsBad{};
        if (!(p_importedRoom == nullptr) &&
            p_importedRoom->isBad(importedRoomIsBad) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // isBad cannot fail; continue as before.
        }
        if (p_importedRoom == nullptr || importedRoomIsBad)
        {
            continue;
        }

        semantic::Room::RoomVariant importedRoomType{};
        if (p_importedRoom->getRoomVariant(importedRoomType) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getRoomVariant cannot fail; continue as before.
        }

        if (importedRoomType == semantic::Room::RoomVariant::UNDEFINED)
        {
            continue;
        }

        Eigen::Vector3d importedCentroid_World_m{};
        if (p_importedRoom->getCentroid(importedCentroid_World_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }

        if (!importedCentroid_World_m.allFinite())
        {
            continue;
        }

        std::vector<geometric::Plane *> importedWalls{};
        if (p_importedRoom->getWalls(importedWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }

        semantic::Room *p_bestRetainedRoom = nullptr;
        double bestCentroidDistance_m = std::numeric_limits<double>::infinity();

        for (semantic::Room *p_candidateRoom : p_map_inout->getAllRooms())
        {
            bool candidateRoomIsBad{};
            if (!(p_candidateRoom == nullptr ||
                  p_candidateRoom == p_importedRoom) &&
                p_candidateRoom->isBad(candidateRoomIsBad) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            semantic::Room::RoomVariant candidateRoomRoomVariant{};
            if (!(p_candidateRoom == nullptr ||
                  p_candidateRoom == p_importedRoom || candidateRoomIsBad ||
                  importedRoomSet.count(p_candidateRoom) > 0U) &&
                p_candidateRoom->getRoomVariant(candidateRoomRoomVariant) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getRoomVariant cannot fail; continue as before.
            }
            if (p_candidateRoom == nullptr ||
                p_candidateRoom == p_importedRoom || candidateRoomIsBad ||
                importedRoomSet.count(p_candidateRoom) > 0U ||
                candidateRoomRoomVariant != importedRoomType)
            {
                continue;
            }

            std::string importedIdentity{};
            if (p_importedRoom->getRoomTag(importedIdentity) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getRoomTag cannot fail; continue as before.
            }
            std::string candidateIdentity{};
            if (p_candidateRoom->getRoomTag(candidateIdentity) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getRoomTag cannot fail; continue as before.
            }
            int importedRoomId{};
            if (!(!importedIdentity.empty() && !candidateIdentity.empty()) &&
                p_importedRoom->getId(importedRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int candidateRoomId{};
            if (!(!importedIdentity.empty() && !candidateIdentity.empty()) &&
                p_candidateRoom->getId(candidateRoomId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            const bool identitiesMatch =
                !importedIdentity.empty() && !candidateIdentity.empty()
                    ? importedIdentity == candidateIdentity
                    : importedRoomId == candidateRoomId;
            if (identitiesMatch)
            {
                p_bestRetainedRoom = p_candidateRoom;
                Eigen::Vector3d candidateRoomCentroid{};
                if (p_candidateRoom->getCentroid(candidateRoomCentroid) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getCentroid cannot fail; continue as before.
                }
                bestCentroidDistance_m =
                    (candidateRoomCentroid - importedCentroid_World_m).norm();
                break;
            }

            /* Geometry-only fusion remains a compatibility fallback solely
             * for legacy unnumbered rooms. Mission rooms always carry a
             * non-negative stable ID and must never cross identities. */
            int importedRoomId2{};
            if (p_importedRoom->getId(importedRoomId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int candidateRoomId2{};
            if (!(importedRoomId2 >= 0) &&
                p_candidateRoom->getId(candidateRoomId2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            if (importedRoomId2 >= 0 || candidateRoomId2 >= 0)
            {
                continue;
            }

            bool importedRoomHasKnownLabel{};
            if (p_importedRoom->getHasKnownLabel(importedRoomHasKnownLabel) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getHasKnownLabel cannot fail; continue as before.
            }
            bool candidateRoomHasKnownLabel{};
            if ((importedRoomHasKnownLabel) &&
                p_candidateRoom->getHasKnownLabel(candidateRoomHasKnownLabel) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getHasKnownLabel cannot fail; continue as before.
            }
            int importedRoomMetaMarkerId{};
            if ((importedRoomHasKnownLabel && candidateRoomHasKnownLabel) &&
                p_importedRoom->getMetaMarkerId(importedRoomMetaMarkerId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getMetaMarkerId cannot fail; continue as before.
            }
            int candidateRoomMetaMarkerId{};
            if ((importedRoomHasKnownLabel && candidateRoomHasKnownLabel) &&
                p_candidateRoom->getMetaMarkerId(candidateRoomMetaMarkerId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getMetaMarkerId cannot fail; continue as before.
            }
            if (importedRoomHasKnownLabel && candidateRoomHasKnownLabel &&
                importedRoomMetaMarkerId != candidateRoomMetaMarkerId)
            {
                continue;
            }

            Eigen::Vector3d candidateCentroid_World_m{};
            if (p_candidateRoom->getCentroid(candidateCentroid_World_m) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }

            const double centroidDistance_m =
                (candidateCentroid_World_m - importedCentroid_World_m).norm();

            if (!candidateCentroid_World_m.allFinite() ||
                !std::isfinite(centroidDistance_m) ||
                centroidDistance_m > maximumRoomCentroidDistance_m ||
                centroidDistance_m >= bestCentroidDistance_m)
            {
                continue;
            }

            std::vector<geometric::Plane *> candidateWalls{};
            if (p_candidateRoom->getWalls(candidateWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getWalls cannot fail; continue as before.
            }

            std::size_t sameSideSharedWallCount   = 0U;
            bool        hasOppositeSideSharedWall = false;

            for (geometric::Plane *p_importedWall : importedWalls)
            {
                bool importedWallIsBad{};
                if (!(p_importedWall == nullptr) &&
                    p_importedWall->isBad(importedWallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (p_importedWall == nullptr || importedWallIsBad)
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

                g2o::Plane3D importedWallGetGlobalEquation{};
                if (p_importedWall->getGlobalEquation(
                        importedWallGetGlobalEquation) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getGlobalEquation cannot fail; continue as before.
                }
                Eigen::Vector4d wallEquation_World =
                    importedWallGetGlobalEquation.coeffs();

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

        std::vector<geometric::Plane *> retainedWalls{};
        if (p_bestRetainedRoom->getWalls(retainedWalls) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getWalls cannot fail; continue as before.
        }

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
            bool planeIsBad{};
            if ((p_plane != nullptr) &&
                p_plane->isBad(planeIsBad) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            geometric::Plane::PlaneVariant planeType{};
            if ((p_plane != nullptr && !planeIsBad) &&
                p_plane->getPlaneType(planeType) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getPlaneType cannot fail; continue as before.
            }
            if (p_plane != nullptr && !planeIsBad &&
                planeType == geometric::Plane::PlaneVariant::GROUND)
            {
                p_mergeGroundPlane = p_plane;
                break;
            }
        }

        Eigen::Vector3d mergeGroundNormal_World = Eigen::Vector3d::Zero();
        if (p_mergeGroundPlane != nullptr)
        {
            g2o::Plane3D mergeGroundPlaneGetGlobalEquation{};
            if (p_mergeGroundPlane->getGlobalEquation(
                    mergeGroundPlaneGetGlobalEquation) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getGlobalEquation cannot fail; continue as before.
            }
            const Eigen::Vector4d groundEq =
                mergeGroundPlaneGetGlobalEquation.coeffs();
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

        Eigen::Vector3d retainedCentroid_World_m{};
        if (p_bestRetainedRoom->getCentroid(retainedCentroid_World_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }

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
                bool crossesOpening{};
                if (crossesPassablePassageOpening(retainedCentroid_World_m,
                                                  importedCentroid_World_m,
                                                  p_passage,
                                                  mergeGroundNormal_World,
                                                  mergeOpeningMargin_m,
                                                  mergeMinimumSideDistance_m,
                                                  crossesOpening) !=
                    UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    // crossesPassablePassageOpening cannot fail; continue as
                    // before.
                }
                return crossesOpening;
            });

        if (roomsAreSeparatedByPassage)
        {
            int importedRoomId3{};
            if (p_importedRoom->getId(importedRoomId3) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            int bestRetainedRoomId{};
            if (p_bestRetainedRoom->getId(bestRetainedRoomId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getId cannot fail; continue as before.
            }
            std::cout << "[SemanticMerge] Preserved semantic::Room#"
                      << importedRoomId3 << " and semantic::Room#"
                      << bestRetainedRoomId
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
                int firstId{};
                if (p_first->getId(firstId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int secondId{};
                if (p_second->getId(secondId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                return firstId < secondId;
            });

        for (geometric::Plane *p_importedWall : importedWalls)
        {
            bool importedWallIsBad2{};
            if (!(p_importedWall == nullptr) &&
                p_importedWall->isBad(importedWallIsBad2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // isBad cannot fail; continue as before.
            }
            if (p_importedWall == nullptr || importedWallIsBad2)
            {
                continue;
            }

            Eigen::Vector3d importedWallGetCentroid{};
            if (p_importedWall->getCentroid(importedWallGetCentroid) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                // getCentroid cannot fail; continue as before.
            }
            const Eigen::Vector3d importedWallCentroid_World_m =
                importedWallGetCentroid.cast<double>();

            vs_graphs::core::semantic::Passage *p_separatingPassage = nullptr;

            for (vs_graphs::core::semantic::Passage *p_passage : mergePassages)
            {
                bool crossesOpening{};
                if ((p_passage != nullptr) &&
                    crossesPassablePassageOpening(retainedCentroid_World_m,
                                                  importedWallCentroid_World_m,
                                                  p_passage,
                                                  mergeGroundNormal_World,
                                                  mergeOpeningMargin_m,
                                                  mergeMinimumSideDistance_m,
                                                  crossesOpening) !=
                        UtilsStatus::UTILS_STATUS_SUCCESS)
                {
                    // crossesPassablePassageOpening cannot fail; continue as
                    // before.
                }
                if (p_passage != nullptr && crossesOpening)
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
                bool existingOwnerIsBad{};
                if (!(p_existingOwner == nullptr) &&
                    p_existingOwner->isBad(existingOwnerIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                semantic::Room::RoomVariant existingOwnerRoomVariant{};
                if (!(p_existingOwner == nullptr || existingOwnerIsBad ||
                      p_existingOwner == p_importedRoom ||
                      p_existingOwner == p_bestRetainedRoom) &&
                    p_existingOwner->getRoomVariant(existingOwnerRoomVariant) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getRoomVariant cannot fail; continue as before.
                }
                if (p_existingOwner == nullptr || existingOwnerIsBad ||
                    p_existingOwner == p_importedRoom ||
                    p_existingOwner == p_bestRetainedRoom ||
                    existingOwnerRoomVariant !=
                        semantic::Room::RoomVariant::ROOM)
                {
                    continue;
                }

                std::vector<geometric::Plane *> ownerWalls{};
                if (p_existingOwner->getWalls(ownerWalls) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getWalls cannot fail; continue as before.
                }
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
                vs_graphs::core::semantic::Room *p_farSideRoom = nullptr;
                if (p_separatingPassage->getProspectiveRoom(p_farSideRoom) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getProspectiveRoom cannot fail; continue as before.
                }

                bool farSideRoomIsBad{};
                if (!(p_farSideRoom == nullptr) &&
                    p_farSideRoom->isBad(farSideRoomIsBad) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // isBad cannot fail; continue as before.
                }
                if (p_farSideRoom == nullptr || farSideRoomIsBad ||
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
            bool importedRoomWasWallRemoved{};
            if (p_importedRoom->removeWall(transfer.p_wall,
                                           importedRoomWasWallRemoved) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                importedRoomWasWallRemoved =
                    false; // rejected input reads as before
            }

            if (transfer.p_targetRoom == nullptr)
            {
                int id{};
                if (transfer.p_separatingPassage->getId(id) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int getId2{};
                if (transfer.p_wall->getId(getId2) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout << "[SemanticMerge] Far-side Wall#" << getId2
                          << " at semantic::Passage#" << id
                          << " has no prospective; left unbound." << std::endl;
                continue;
            }

            if (transfer.p_targetRoom->setWalls(transfer.p_wall) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setWalls cannot fail; continue as before.
            }

            if (transfer.p_separatingPassage != nullptr &&
                transfer.p_targetRoom != p_bestRetainedRoom)
            {
                int id2{};
                if (transfer.p_targetRoom->getId(id2) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int getId3{};
                if (transfer.p_wall->getId(getId3) !=
                    geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                std::cout << "[SemanticMerge] Redirected far-side Wall#"
                          << getId3 << " to stable semantic::Room#" << id2
                          << "." << std::endl;
            }
        }

        std::vector<vs_graphs::core::semantic::Passage *>
            importedRoomPassages{};
        if (p_importedRoom->getPassages(importedRoomPassages) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getPassages cannot fail; continue as before.
        }
        for (vs_graphs::core::semantic::Passage *p_importedPassage :
             importedRoomPassages)
        {
            if (p_bestRetainedRoom->setDoorways(p_importedPassage) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setDoorways cannot fail; continue as before.
            }
        }

        for (semantic::Passage *p_passage : p_map_inout->getAllPassages())
        {
            if (p_passage != nullptr)
            {
                bool passageWasRoomReplaced{};
                if (p_passage->replaceProspectiveRoom(p_importedRoom,
                                                      p_bestRetainedRoom,
                                                      passageWasRoomReplaced) !=
                    semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
                {
                    passageWasRoomReplaced =
                        false; // rejected input reads as before
                }
            }
        }

        const double retainedWeight = static_cast<double>(
            std::max<std::size_t>(retainedWalls.size(), 1U));

        const double importedWeight = static_cast<double>(
            std::max<std::size_t>(importedWalls.size(), 1U));

        Eigen::Vector3d bestRetainedRoomCentroid{};
        if (p_bestRetainedRoom->getCentroid(bestRetainedRoomCentroid) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getCentroid cannot fail; continue as before.
        }
        const Eigen::Vector3d fusedCentroid_World_m =
            (retainedWeight * bestRetainedRoomCentroid +
             importedWeight * importedCentroid_World_m) /
            (retainedWeight + importedWeight);

        if (p_bestRetainedRoom->setCentroid(fusedCentroid_World_m) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setCentroid cannot fail; continue as before.
        }

        geometric::Plane *p_bestRetainedRoomGroundPlane = nullptr;
        if (p_bestRetainedRoom->getGroundPlane(p_bestRetainedRoomGroundPlane) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getGroundPlane cannot fail; continue as before.
        }
        if (p_bestRetainedRoomGroundPlane == nullptr)
        {
            geometric::Plane *p_importedRoomGroundPlane = nullptr;
            if (p_importedRoom->getGroundPlane(p_importedRoomGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getGroundPlane cannot fail; continue as before.
            }
            if (p_bestRetainedRoom->setGroundPlane(p_importedRoomGroundPlane) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setGroundPlane cannot fail; continue as before.
            }
        }

        bool bestRetainedRoomHasKnownLabel{};
        if (p_bestRetainedRoom->getHasKnownLabel(
                bestRetainedRoomHasKnownLabel) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getHasKnownLabel cannot fail; continue as before.
        }
        bool importedRoomHasKnownLabel2{};
        if ((!bestRetainedRoomHasKnownLabel) &&
            p_importedRoom->getHasKnownLabel(importedRoomHasKnownLabel2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getHasKnownLabel cannot fail; continue as before.
        }
        if (!bestRetainedRoomHasKnownLabel && importedRoomHasKnownLabel2)
        {
            if (p_bestRetainedRoom->setHasKnownLabel(true) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setHasKnownLabel cannot fail; continue as before.
            }
            semantic::Marker *p_importedRoomMetaMarker = nullptr;
            if (p_importedRoom->getMetaMarker(p_importedRoomMetaMarker) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getMetaMarker cannot fail; continue as before.
            }
            if (p_bestRetainedRoom->setMetaMarker(p_importedRoomMetaMarker) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setMetaMarker cannot fail; continue as before.
            }
            int importedRoomMetaMarkerId2{};
            if (p_importedRoom->getMetaMarkerId(importedRoomMetaMarkerId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getMetaMarkerId cannot fail; continue as before.
            }
            if (p_bestRetainedRoom->setMetaMarkerId(
                    importedRoomMetaMarkerId2) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setMetaMarkerId cannot fail; continue as before.
            }
            std::string importedRoomName{};
            if (p_importedRoom->getName(importedRoomName) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // getName cannot fail; continue as before.
            }
            if (p_bestRetainedRoom->setName(importedRoomName) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setName cannot fail; continue as before.
            }
        }

        /* Presence on either side proves the UAV has been there: a room fused
         * from a visited duplicate stays visited. */
        bool importedRoomHasPreviouslyVisited{};
        if (p_importedRoom->hasPreviouslyVisited(
                importedRoomHasPreviouslyVisited) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // hasPreviouslyVisited cannot fail; continue as before.
        }
        if (importedRoomHasPreviouslyVisited)
        {
            if (p_bestRetainedRoom->setPreviouslyVisited(true) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                // setPreviouslyVisited cannot fail; continue as before.
            }
        }

        for (semantic::Floor *p_floor : p_map_inout->getAllFloors())
        {
            if (p_floor != nullptr)
            {
                bool floorWasRoomReplaced{};
                if (p_floor->replaceRoom(p_importedRoom,
                                         p_bestRetainedRoom,
                                         floorWasRoomReplaced) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    floorWasRoomReplaced =
                        false; // rejected input reads as before
                }
            }
        }

        p_map_inout->eraseDetectedMapRoom(p_importedRoom);
        p_map_inout->eraseMarkerBasedMapRoom(p_importedRoom);
        if (p_importedRoom->clearWalls() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // clearWalls cannot fail; continue as before.
        }
        if (p_importedRoom->clearPassages() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // clearPassages cannot fail; continue as before.
        }
        if (p_importedRoom->setBad() !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // setBad cannot fail; continue as before.
        }

        int importedRoomId4{};
        if (p_importedRoom->getId(importedRoomId4) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        int bestRetainedRoomId2{};
        if (p_bestRetainedRoom->getId(bestRetainedRoomId2) !=
            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        std::cout << "[SemanticMerge] Fused duplicate semantic::Room#"
                  << importedRoomId4 << " into semantic::Room#"
                  << bestRetainedRoomId2 << " (centroid distance "
                  << bestCentroidDistance_m << " m)." << std::endl;
    }

    return UtilsStatus::UTILS_STATUS_SUCCESS;
}

} // namespace utils
} // namespace utils
} // namespace core
} // namespace vs_graphs
