/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            mergeMapPair.cc
 *
 * @brief           Implements Atlas::mergeMapPair(), declared in Atlas.h.
 */

#include "Atlas.h"

#include "Utils/Utils/objects/Utils.h"

#include "../private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <sophus/se3.hpp>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Merges the semantic graph of the other map into the
 *                  current map.
 *
 *                  Mirrors the semantic-transfer pattern of
 *                  LoopClosing::MergeLocal using Horn's deterministic
 *                  closed-form solution; no g2o types are used. Caller must
 *                  already hold the semantic-update lock.
 */
AtlasStatus Atlas::mergeMapPair(Map *p_currentMap_inout, Map *p_otherMap_inout)
{
    if (p_currentMap_inout == nullptr || p_otherMap_inout == nullptr)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: null map pointer."
                  << std::endl;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    if (p_currentMap_inout == p_otherMap_inout)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: identical maps."
                  << std::endl;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    bool currentMapIsBad{};
    if (p_currentMap_inout->isBad(currentMapIsBad) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    bool otherMapIsBad{};
    if (!(currentMapIsBad) &&
        p_otherMap_inout->isBad(otherMapIsBad) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (currentMapIsBad || otherMapIsBad)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: a map is bad."
                  << std::endl;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    bool isActiveMap2{};
    if (isActiveMap(p_currentMap_inout, isActiveMap2) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool isActiveMap3{};
    if (!(!isActiveMap2) && isActiveMap(p_otherMap_inout, isActiveMap3) !=
                                AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isActiveMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (!isActiveMap2 || !isActiveMap3)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: map not active."
                  << std::endl;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    /* Pair the walls of both maps using their shared room identity tags. */
    std::vector<Eigen::Vector3d> normalsCurrent, centroidsCurrent;
    std::vector<Eigen::Vector3d> normalsOther, centroidsOther;

    bool hasEnoughCorrespondences{};
    if (utils::utils::Utils::collectCorrespondingWalls(
            p_currentMap_inout,
            p_otherMap_inout,
            normalsCurrent,
            centroidsCurrent,
            normalsOther,
            centroidsOther,
            hasEnoughCorrespondences) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: collectCorrespondingWalls returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (!hasEnoughCorrespondences)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: fewer than three "
                     "wall correspondences."
                  << std::endl;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    /* Horn's closed-form transform maps other-frame points into current frame.
     */
    Eigen::Isometry3d T_otherToCurrent{};
    if (utils::utils::Utils::computeMapTransform_Horn(normalsOther,
                                                      centroidsOther,
                                                      normalsCurrent,
                                                      centroidsCurrent,
                                                      T_otherToCurrent) !=
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeMapTransform_Horn returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    if (!T_otherToCurrent.matrix().allFinite())
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: invalid transform."
                  << std::endl;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    /* Compare floor identities in the surviving map frame without mutating
     * either map. A mismatch rejects the merge before any entity is moved. */
    semantic::Floor               *p_currentFloor = nullptr;
    std::vector<semantic::Floor *> currentMapAllFloors{};
    if (p_currentMap_inout->getAllFloors(currentMapAllFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (semantic::Floor::selectBestObservedFloor(currentMapAllFloors,
                                                 p_currentFloor) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Floor               *p_otherFloor = nullptr;
    std::vector<semantic::Floor *> otherMapAllFloors{};
    if (p_otherMap_inout->getAllFloors(otherMapAllFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (semantic::Floor::selectBestObservedFloor(otherMapAllFloors,
                                                 p_otherFloor) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::optional<semantic::Floor::PlaneIdentity> currentFloorPlaneIdentity{};
    if ((p_currentFloor != nullptr) &&
        p_currentFloor->getPlaneIdentity(currentFloorPlaneIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::optional<semantic::Floor::PlaneIdentity> currentFloorIdentity =
        p_currentFloor != nullptr ? currentFloorPlaneIdentity : std::nullopt;
    std::optional<semantic::Floor::PlaneIdentity> otherFloorPlaneIdentity{};
    if ((p_otherFloor != nullptr) &&
        p_otherFloor->getPlaneIdentity(otherFloorPlaneIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::optional<semantic::Floor::PlaneIdentity> otherFloorIdentity =
        p_otherFloor != nullptr ? otherFloorPlaneIdentity : std::nullopt;

    const g2o::Sim3 floorTransform_otherWorldToCurrentWorld(
        T_otherToCurrent.linear(),
        T_otherToCurrent.translation(),
        1.0);

    if (currentFloorIdentity.has_value() && otherFloorIdentity.has_value())
    {
        std::optional<semantic::Floor::PlaneIdentity>
            transformedOtherFloorIdentity{};
        if (semantic::Floor::transformPlaneIdentity(
                *otherFloorIdentity,
                floorTransform_otherWorldToCurrentWorld,
                transformedOtherFloorIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: transformPlaneIdentity returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        double floorNormalAngle_deg = std::numeric_limits<double>::infinity();
        double floorOffset_m        = std::numeric_limits<double>::infinity();

        bool isMatch{};
        if (!(!transformedOtherFloorIdentity.has_value()) &&
            semantic::Floor::planeIdentitiesMatch(
                *currentFloorIdentity,
                transformedOtherFloorIdentity.value_or(*otherFloorIdentity),
                semantic::Floor::kMergeMaxPlaneNormalAngle_deg,
                semantic::Floor::kMergeMaxPlaneOffset_m,
                floorNormalAngle_deg,
                floorOffset_m,
                isMatch) != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: planeIdentitiesMatch returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!transformedOtherFloorIdentity.has_value() || !isMatch)
        {
            unsigned long currentMapId{};
            if (p_currentMap_inout->getId(currentMapId) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            unsigned long otherMapId{};
            if (p_otherMap_inout->getId(otherMapId) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            std::cerr << "[FloorVerify] Rejecting merge: Map#" << currentMapId
                      << " and Map#" << otherMapId
                      << " floor planes mismatch (angle="
                      << floorNormalAngle_deg
                      << " deg, offset=" << floorOffset_m << " m; limits="
                      << semantic::Floor::kMergeMaxPlaneNormalAngle_deg
                      << " deg/" << semantic::Floor::kMergeMaxPlaneOffset_m
                      << " m). result=REJECTED committed=0" << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }

        unsigned long currentMapId2{};
        if (p_currentMap_inout->getId(currentMapId2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long otherMapId2{};
        if (p_otherMap_inout->getId(otherMapId2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[FloorVerify] Map#" << currentMapId2 << " and Map#"
                  << otherMapId2
                  << " floor planes match (angle=" << floorNormalAngle_deg
                  << " deg, offset=" << floorOffset_m
                  << " m). result=ACCEPTED committed=0" << std::endl;
    }
    else
    {
        unsigned long currentMapId3{};
        if (p_currentMap_inout->getId(currentMapId3) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long otherMapId3{};
        if (p_otherMap_inout->getId(otherMapId3) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[FloorVerify] Map#" << currentMapId3 << " and Map#"
                  << otherMapId3 << " floor verification deferred (current="
                  << (currentFloorIdentity.has_value() ? "valid" : "missing")
                  << ", other="
                  << (otherFloorIdentity.has_value() ? "valid" : "missing")
                  << "); result=DEFERRED committed=0" << std::endl;
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    /* Snapshot every source-owned object and preflight destination indexes
     * before changing geometry, ownership, or any externally visible ID. */
    std::vector<KeyFrame *> importedKeyFrames{};
    if (p_otherMap_inout->getAllKeyFrames(importedKeyFrames) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<MapPoint *> importedMapPoints{};
    if (p_otherMap_inout->getAllMapPoints(importedMapPoints) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> importedDetectedRooms{};
    if (p_otherMap_inout->getAllDetectedMapRooms(importedDetectedRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllDetectedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Room *> importedMarkerRooms{};
    if (p_otherMap_inout->getAllMarkerBasedMapRooms(importedMarkerRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkerBasedMapRooms returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<geometric::Plane *> importedPlanes{};
    if (p_otherMap_inout->getAllPlanes(importedPlanes) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<vs_graphs::core::semantic::Passage *> importedPassages{};
    if (p_otherMap_inout->getAllPassages(importedPassages) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<vs_graphs::core::semantic::Floor *> importedFloors{};
    if (p_otherMap_inout->getAllFloors(importedFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Marker *> importedMarkers{};
    if (p_otherMap_inout->getAllMarkers(importedMarkers) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkers returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    const std::set<semantic::Room *> importedDetectedRoomSet(
        importedDetectedRooms.begin(),
        importedDetectedRooms.end());
    importedMarkerRooms.erase(
        std::remove_if(importedMarkerRooms.begin(),
                       importedMarkerRooms.end(),
                       [&importedDetectedRoomSet](semantic::Room *p_room)
                       { return importedDetectedRoomSet.count(p_room) > 0U; }),
        importedMarkerRooms.end());

    std::vector<semantic::Room *> importedRooms = importedDetectedRooms;
    importedRooms.insert(importedRooms.end(),
                         importedMarkerRooms.begin(),
                         importedMarkerRooms.end());

    const auto ownerIsTransferable = [p_otherMap_inout](Map *p_ownerMap)
    { return p_ownerMap == p_otherMap_inout; };

    std::vector<KeyFrame *> destinationKeyFrames{};
    if (p_currentMap_inout->getAllKeyFrames(destinationKeyFrames) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllKeyFrames returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::set<KeyFrame *> destinationKeyFrameSet(
        destinationKeyFrames.begin(),
        destinationKeyFrames.end());
    std::vector<MapPoint *> destinationMapPoints{};
    if (p_currentMap_inout->getAllMapPoints(destinationMapPoints) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::set<MapPoint *> destinationMapPointSet(
        destinationMapPoints.begin(),
        destinationMapPoints.end());

    for (KeyFrame *p_keyFrame : importedKeyFrames)
    {
        Map *p_keyFrameMap = nullptr;
        if (!(p_keyFrame == nullptr) &&
            p_keyFrame->getMap(p_keyFrameMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_keyFrame == nullptr || !ownerIsTransferable(p_keyFrameMap) ||
            destinationKeyFrameSet.count(p_keyFrame) > 0U)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source "
                         "keyframe has inconsistent ownership."
                      << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }

        KeyFrame *p_indexedKeyFrame = nullptr;
        if (p_currentMap_inout->getKeyFrameById(p_keyFrame->id,
                                                p_indexedKeyFrame) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getKeyFrameById returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_indexedKeyFrame != nullptr && p_indexedKeyFrame != p_keyFrame)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: KeyFrame ID "
                      << p_keyFrame->id << " collides in destination map."
                      << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }
    }

    for (MapPoint *p_mapPoint : importedMapPoints)
    {
        Map *p_mapPointMap = nullptr;
        if (!(p_mapPoint == nullptr) &&
            p_mapPoint->getMap(p_mapPointMap) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_mapPoint == nullptr || !ownerIsTransferable(p_mapPointMap) ||
            destinationMapPointSet.count(p_mapPoint) > 0U)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source map "
                         "point has inconsistent ownership."
                      << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }
    }

    for (geometric::Plane *p_plane : importedPlanes)
    {
        core::Map *p_planeMap = nullptr;
        if (!(p_plane == nullptr) &&
            p_plane->getMap(p_planeMap) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_plane == nullptr || !ownerIsTransferable(p_planeMap))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source plane "
                         "has inconsistent ownership."
                      << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }
    }

    for (semantic::Marker *p_marker : importedMarkers)
    {
        core::Map *p_markerMap = nullptr;
        if (!(p_marker == nullptr) &&
            p_marker->getMap(p_markerMap) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_marker == nullptr || !ownerIsTransferable(p_markerMap))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source marker "
                         "has inconsistent ownership."
                      << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }
    }

    for (semantic::Passage *p_passage : importedPassages)
    {
        vs_graphs::core::Map *p_passageMap = nullptr;
        if (!(p_passage == nullptr) &&
            p_passage->getMap(p_passageMap) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_passage == nullptr || !ownerIsTransferable(p_passageMap))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source passage "
                         "has inconsistent ownership."
                      << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }
    }

    for (semantic::Room *p_room : importedRooms)
    {
        core::Map *p_roomMap = nullptr;
        if (!(p_room == nullptr) &&
            p_room->getMap(p_roomMap) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_room == nullptr || !ownerIsTransferable(p_roomMap))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source room "
                         "has inconsistent ownership."
                      << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }
    }

    for (semantic::Floor *p_floor : importedFloors)
    {
        vs_graphs::core::Map *p_floorMap = nullptr;
        if (!(p_floor == nullptr) &&
            p_floor->getMap(p_floorMap) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_floor == nullptr || !ownerIsTransferable(p_floorMap))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source floor "
                         "has inconsistent ownership."
                      << std::endl;
            return AtlasStatus::ATLAS_STATUS_SUCCESS;
        }
    }

    std::vector<std::pair<geometric::Plane *, int>>  planeIdAssignments;
    std::vector<std::pair<semantic::Marker *, int>>  markerIdAssignments;
    std::vector<std::pair<semantic::Passage *, int>> passageIdAssignments;
    std::vector<std::pair<semantic::Room *, int>>    roomIdAssignments;
    std::vector<std::pair<semantic::Floor *, int>>   floorIdAssignments;

    /* Plan the ids of each entity kind in turn; stop at the first failure. */
    std::vector<geometric::Plane *> currentMapAllPlanes{};
    if (p_currentMap_inout->getAllPlanes(currentMapAllPlanes) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPlanes returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool isPlanned{};
    if (planImportedIds(currentMapAllPlanes,
                        importedPlanes,
                        "plane",
                        planeIdAssignments,
                        isPlanned) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: planImportedIds returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isPlanned)
    {
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    std::vector<semantic::Marker *> currentMapAllMarkers{};
    if (p_currentMap_inout->getAllMarkers(currentMapAllMarkers) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMarkers returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool isPlanned2{};
    if (planImportedIds(currentMapAllMarkers,
                        importedMarkers,
                        "marker",
                        markerIdAssignments,
                        isPlanned2) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: planImportedIds returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isPlanned2)
    {
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    std::vector<vs_graphs::core::semantic::Passage *> currentMapAllPassages{};
    if (p_currentMap_inout->getAllPassages(currentMapAllPassages) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool isPlanned3{};
    if (planImportedIds(currentMapAllPassages,
                        importedPassages,
                        "passage",
                        passageIdAssignments,
                        isPlanned3) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: planImportedIds returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isPlanned3)
    {
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    std::vector<semantic::Room *> currentMapAllRooms{};
    if (p_currentMap_inout->getAllRooms(currentMapAllRooms) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllRooms returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool isPlanned4{};
    if (planImportedIds(currentMapAllRooms,
                        importedRooms,
                        "room",
                        roomIdAssignments,
                        isPlanned4) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: planImportedIds returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isPlanned4)
    {
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }
    std::vector<semantic::Floor *> currentMapAllFloors2{};
    if (p_currentMap_inout->getAllFloors(currentMapAllFloors2) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool isPlanned5{};
    if (planImportedIds(currentMapAllFloors2,
                        importedFloors,
                        "floor",
                        floorIdAssignments,
                        isPlanned5) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: planImportedIds returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (!isPlanned5)
    {
        return AtlasStatus::ATLAS_STATUS_SUCCESS;
    }

    std::vector<MapPoint *> importedReferenceMapPoints{};
    if (p_otherMap_inout->getReferenceMapPoints(importedReferenceMapPoints) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getReferenceMapPoints returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const std::vector<KeyFrame *> importedKeyFrameOrigins =
        p_otherMap_inout->keyFrameOrigins;

    const Eigen::Matrix3f R = T_otherToCurrent.linear().cast<float>();
    const Eigen::Vector3f t = T_otherToCurrent.translation().cast<float>();
    const Sophus::SE3f    T_otherToCurrent_SE3f(R, t);

    {
        std::scoped_lock mapUpdateLocks(p_currentMap_inout->mapUpdateMutex,
                                        p_otherMap_inout->mapUpdateMutex);

        if (p_otherMap_inout->applyScaledRotation(T_otherToCurrent_SE3f,
                                                  1.0f,
                                                  false) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: applyScaledRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        for (KeyFrame *p_keyFrame : importedKeyFrames)
        {
            if (p_keyFrame->updateMap(p_currentMap_inout) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateMap returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap_inout->addKeyFrame(p_keyFrame) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_otherMap_inout->eraseKeyFrame(p_keyFrame) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseKeyFrame returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        for (MapPoint *p_mapPoint : importedMapPoints)
        {
            if (p_mapPoint->updateMap(p_currentMap_inout) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: updateMap returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap_inout->addMapPoint(p_mapPoint) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_otherMap_inout->eraseMapPoint(p_mapPoint) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPoint returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        for (const auto &[p_plane, assignedId] : planeIdAssignments)
        {
            if (p_plane->setId(assignedId) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_plane->setMap(p_currentMap_inout) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap_inout->addMapPlane(p_plane) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_otherMap_inout->eraseRoomWallPlane(p_plane) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseRoomWallPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_otherMap_inout->eraseMapPlane(p_plane) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPlane returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        std::unordered_map<int, int> importedMarkerIdRemap;
        for (const auto &[p_marker, assignedId] : markerIdAssignments)
        {
            int markerId{};
            if (p_marker->getId(markerId) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            importedMarkerIdRemap.insert_or_assign(markerId, assignedId);
            if (p_marker->setId(assignedId) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_marker->setMap(p_currentMap_inout) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap_inout->addMapMarker(p_marker) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapMarker returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_otherMap_inout->eraseMapMarker(p_marker) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapMarker returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        for (const auto &[p_passage, assignedId] : passageIdAssignments)
        {
            /* Same stable lineage as a current-map recovery proxy: fold the
             * transferred (now in-frame) state into the proxy instead of
             * duplicating the doorway. The transferred object retires with
             * the absorbed map; it never enters the current map. */
            int passageId{};
            if (p_passage->getId(passageId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            semantic::Passage *p_proxy = nullptr;
            if (p_currentMap_inout->getPassageById(passageId, p_proxy) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPassageById returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            bool wasResurfaced{};
            if ((p_proxy != nullptr) &&
                resurfaceProxyFromTransferred(p_proxy,
                                              p_passage,
                                              wasResurfaced) !=
                    AtlasStatus::ATLAS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: resurfaceProxyFromTransferred returned a failure "
                    "status although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_proxy != nullptr && wasResurfaced)
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
                continue;
            }
            if (p_passage->setId(assignedId) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_passage->setMap(p_currentMap_inout) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap_inout->addMapPassage(p_passage) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_otherMap_inout->eraseMapPassage(p_passage) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapPassage returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        for (semantic::Room *p_room : importedRooms)
        {
            semantic::Marker *p_metaMarker = nullptr;
            if (p_room->getMetaMarker(p_metaMarker) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMetaMarker returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_metaMarker != nullptr)
            {
                int metaMarkerId{};
                if (p_metaMarker->getId(metaMarkerId) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_room->setMetaMarkerId(metaMarkerId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: setMetaMarkerId returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            else
            {
                int roomMetaMarkerId{};
                if (p_room->getMetaMarkerId(roomMetaMarkerId) !=
                    semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMetaMarkerId returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
                const std::unordered_map<int, int>::iterator markerIdIterator =
                    importedMarkerIdRemap.find(roomMetaMarkerId);
                if (markerIdIterator != importedMarkerIdRemap.end())
                {
                    if (p_room->setMetaMarkerId(markerIdIterator->second) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: setMetaMarkerId returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }
        }

        for (const auto &[p_room, assignedId] : roomIdAssignments)
        {
            if (p_room->setId(assignedId) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_room->setMap(p_currentMap_inout) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (importedDetectedRoomSet.count(p_room) > 0U)
            {
                if (p_currentMap_inout->addDetectedMapRoom(p_room) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addDetectedMapRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            else
            {
                if (p_currentMap_inout->addCandidateMapRoom(p_room) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: addCandidateMapRoom returned a failure status "
                        "although it cannot fail; continuing as before.",
                        __func__);
                }
            }
            if (p_otherMap_inout->eraseDetectedMapRoom(p_room) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: eraseDetectedMapRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            if (p_otherMap_inout->eraseMarkerBasedMapRoom(p_room) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: eraseMarkerBasedMapRoom returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
        }

        for (const auto &[p_floor, assignedId] : floorIdAssignments)
        {
            if (p_floor->setId(assignedId) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setId returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_floor->setMap(p_currentMap_inout) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMap_inout->addMapFloor(p_floor) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: addMapFloor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_otherMap_inout->eraseMapFloor(p_floor) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseMapFloor returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }

        std::vector<MapPoint *> mergedReferenceMapPoints{};
        if (p_currentMap_inout->getReferenceMapPoints(
                mergedReferenceMapPoints) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getReferenceMapPoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        for (MapPoint *p_mapPoint : importedReferenceMapPoints)
        {
            Map *p_mapPointMap2 = nullptr;
            if ((p_mapPoint != nullptr) &&
                p_mapPoint->getMap(p_mapPointMap2) !=
                    MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_mapPoint != nullptr && p_mapPointMap2 == p_currentMap_inout &&
                std::find(mergedReferenceMapPoints.begin(),
                          mergedReferenceMapPoints.end(),
                          p_mapPoint) == mergedReferenceMapPoints.end())
            {
                mergedReferenceMapPoints.push_back(p_mapPoint);
            }
        }
        if (p_currentMap_inout->setReferenceMapPoints(
                mergedReferenceMapPoints) != MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setReferenceMapPoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_otherMap_inout->setReferenceMapPoints({}) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setReferenceMapPoints returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        for (KeyFrame *p_originKeyFrame : importedKeyFrameOrigins)
        {
            Map *p_originKeyFrameMap = nullptr;
            if ((p_originKeyFrame != nullptr) &&
                p_originKeyFrame->getMap(p_originKeyFrameMap) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_originKeyFrame != nullptr &&
                p_originKeyFrameMap == p_currentMap_inout &&
                std::find(p_currentMap_inout->keyFrameOrigins.begin(),
                          p_currentMap_inout->keyFrameOrigins.end(),
                          p_originKeyFrame) ==
                    p_currentMap_inout->keyFrameOrigins.end())
            {
                p_currentMap_inout->keyFrameOrigins.push_back(p_originKeyFrame);
            }
        }
        p_otherMap_inout->keyFrameOrigins.clear();

        if (p_currentMap_inout->setSkeletonClusterPoints({}) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setSkeletonClusterPoints returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_currentMap_inout->setSkeletonEdges({}) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setSkeletonEdges returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_otherMap_inout->setSkeletonClusterPoints({}) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: setSkeletonClusterPoints returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_otherMap_inout->setSkeletonEdges({}) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setSkeletonEdges returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (p_otherMap_inout->clearTransferredEntityIndexes() !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: clearTransferredEntityIndexes returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        /* Fuse duplicate floors: keep only one floor per map (system supports
         * single-floor semantics). Reassign rooms from duplicate floors to the
         * primary floor and erase the extras. */
        std::vector<semantic::Floor *> allFloors{};
        if (p_currentMap_inout->getAllFloors(allFloors) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllFloors returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (allFloors.size() > 1)
        {
            semantic::Floor *p_keeperFloor = nullptr;
            if (semantic::Floor::selectBestObservedFloor(allFloors,
                                                         p_keeperFloor) !=
                semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: selectBestObservedFloor returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            for (semantic::Floor *p_duplicateFloor : allFloors)
            {
                if (p_duplicateFloor == nullptr ||
                    p_duplicateFloor == p_keeperFloor)
                {
                    continue;
                }

                std::vector<vs_graphs::core::semantic::Room *>
                    duplicateFloorRooms{};
                if (p_duplicateFloor->getRooms(duplicateFloorRooms) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getRooms returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                for (semantic::Room *p_room : duplicateFloorRooms)
                {
                    bool roomIsBad{};
                    if ((p_room != nullptr) &&
                        p_room->isBad(roomIsBad) !=
                            semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_room != nullptr && !roomIsBad)
                    {
                        if (p_keeperFloor->addRoom(p_room) !=
                            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                         "%s: addRoom returned a failure "
                                         "status although it cannot fail; "
                                         "continuing as before.",
                                         __func__);
                        }
                    }
                }

                if (p_currentMap_inout->eraseMapFloor(p_duplicateFloor) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: eraseMapFloor returned a failure status although "
                        "it cannot fail; continuing as before.",
                        __func__);
                }
                int duplicateFloorId{};
                if (p_duplicateFloor->getId(duplicateFloorId) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int keeperFloorId{};
                if (p_keeperFloor->getId(keeperFloorId) !=
                    semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                std::cout
                    << "[Atlas::MergeMapPair] Fused duplicate semantic::Floor#"
                    << duplicateFloorId << " into semantic::Floor#"
                    << keeperFloorId
                    << " and retained the better-observed plane identity."
                    << std::endl;
            }
        }

        if (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(
                p_currentMap_inout,
                importedRooms) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: fuseDuplicateRoomsAfterMerge returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        semantic::Floor               *p_mergedFloor = nullptr;
        std::vector<semantic::Floor *> currentMapAllFloors3{};
        if (p_currentMap_inout->getAllFloors(currentMapAllFloors3) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllFloors returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        if (semantic::Floor::selectBestObservedFloor(currentMapAllFloors3,
                                                     p_mergedFloor) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: selectBestObservedFloor returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_mergedFloor != nullptr)
        {
            std::vector<semantic::Room *> currentMapAllDetectedMapRooms{};
            if (p_currentMap_inout->getAllDetectedMapRooms(
                    currentMapAllDetectedMapRooms) !=
                MapStatus::MAP_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("vs_graphs"),
                    "%s: getAllDetectedMapRooms returned a failure status "
                    "although it cannot fail; continuing as before.",
                    __func__);
            }
            for (semantic::Room *p_room : currentMapAllDetectedMapRooms)
            {
                bool roomIsBad2{};
                if ((p_room != nullptr) &&
                    p_room->isBad(roomIsBad2) !=
                        semantic::RoomStatus::ROOM_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_room != nullptr && !roomIsBad2)
                {
                    if (p_mergedFloor->addRoom(p_room) !=
                        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addRoom returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }
        }

        std::vector<semantic::Room *> currentMapAllRooms2{};
        if (p_currentMap_inout->getAllRooms(currentMapAllRooms2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAllRooms returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        for (semantic::Room *p_room : currentMapAllRooms2)
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
            if (p_room == nullptr || roomIsBad3)
            {
                continue;
            }

            std::vector<geometric::Plane *> roomWalls{};
            if (p_room->getWalls(roomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getWalls returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }
            for (geometric::Plane *p_wall : roomWalls)
            {
                bool wallIsBad{};
                if ((p_wall != nullptr) &&
                    p_wall->isBad(wallIsBad) !=
                        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: isBad returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (p_wall != nullptr && !wallIsBad)
                {
                    if (p_currentMap_inout->addRoomWallPlane(p_wall) !=
                        MapStatus::MAP_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addRoomWallPlane returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }
        }

        if (utils::utils::Utils::reAssociatePassages(this) !=
            utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: reAssociatePassages returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    /* Retire the absorbed map while keeping the current map active. */
    if (setMapBad(p_otherMap_inout) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMapBad returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (changeMap(p_currentMap_inout) != AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: changeMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    unsigned long otherMapId4{};
    if (p_otherMap_inout->getId(otherMapId4) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    unsigned long currentMapId4{};
    if (p_currentMap_inout->getId(currentMapId4) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "[Atlas::MergeMapPair] Merged map " << otherMapId4
              << " into map " << currentMapId4 << " fused "
              << importedRooms.size() << " rooms into current map."
              << std::endl;
    unsigned long currentMapId5{};
    if (p_currentMap_inout->getId(currentMapId5) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    unsigned long otherMapId5{};
    if (p_otherMap_inout->getId(otherMapId5) != MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "[FloorVerify] Map#" << currentMapId5 << " and Map#"
              << otherMapId5 << " result=ACCEPTED committed=1" << std::endl;

    /* Notify downstream consumers that the current map changed. */
    if (p_currentMap_inout->increaseChangeIndex() !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: increaseChangeIndex returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
