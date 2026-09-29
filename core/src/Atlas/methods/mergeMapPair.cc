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
 * @brief        Merges the semantic graph of the other map into the
 *               current map.
 *
 *               Mirrors the semantic-transfer pattern of
 *               LoopClosing::MergeLocal using Horn's deterministic
 *               closed-form solution; no g2o types are used. Caller must
 *               already hold the semantic-update lock.
 */
void Atlas::mergeMapPair(Map *p_currentMap_inout, Map *p_otherMap_inout)
{
    if (p_currentMap_inout == nullptr || p_otherMap_inout == nullptr)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: null map pointer."
                  << std::endl;
        return;
    }

    if (p_currentMap_inout == p_otherMap_inout)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: identical maps."
                  << std::endl;
        return;
    }

    if (p_currentMap_inout->isBad() || p_otherMap_inout->isBad())
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: a map is bad."
                  << std::endl;
        return;
    }

    if (!isActiveMap(p_currentMap_inout) || !isActiveMap(p_otherMap_inout))
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: map not active."
                  << std::endl;
        return;
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
        return;
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
        return;
    }

    /* Compare floor identities in the surviving map frame without mutating
     * either map. A mismatch rejects the merge before any entity is moved. */
    semantic::Floor *p_currentFloor = nullptr;
    if (semantic::Floor::selectBestObservedFloor(
            p_currentMap_inout->getAllFloors(),
            p_currentFloor) != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Floor *p_otherFloor = nullptr;
    if (semantic::Floor::selectBestObservedFloor(
            p_otherMap_inout->getAllFloors(),
            p_otherFloor) != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
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
            std::cerr << "[FloorVerify] Rejecting merge: Map#"
                      << p_currentMap_inout->getId() << " and Map#"
                      << p_otherMap_inout->getId()
                      << " floor planes mismatch (angle="
                      << floorNormalAngle_deg
                      << " deg, offset=" << floorOffset_m << " m; limits="
                      << semantic::Floor::kMergeMaxPlaneNormalAngle_deg
                      << " deg/" << semantic::Floor::kMergeMaxPlaneOffset_m
                      << " m). result=REJECTED committed=0" << std::endl;
            return;
        }

        std::cout << "[FloorVerify] Map#" << p_currentMap_inout->getId()
                  << " and Map#" << p_otherMap_inout->getId()
                  << " floor planes match (angle=" << floorNormalAngle_deg
                  << " deg, offset=" << floorOffset_m
                  << " m). result=ACCEPTED committed=0" << std::endl;
    }
    else
    {
        std::cout << "[FloorVerify] Map#" << p_currentMap_inout->getId()
                  << " and Map#" << p_otherMap_inout->getId()
                  << " floor verification deferred (current="
                  << (currentFloorIdentity.has_value() ? "valid" : "missing")
                  << ", other="
                  << (otherFloorIdentity.has_value() ? "valid" : "missing")
                  << "); result=DEFERRED committed=0" << std::endl;
        return;
    }

    /* Snapshot every source-owned object and preflight destination indexes
     * before changing geometry, ownership, or any externally visible ID. */
    std::vector<KeyFrame *> importedKeyFrames =
        p_otherMap_inout->getAllKeyFrames();
    std::vector<MapPoint *> importedMapPoints =
        p_otherMap_inout->getAllMapPoints();
    std::vector<semantic::Room *> importedDetectedRooms =
        p_otherMap_inout->getAllDetectedMapRooms();
    std::vector<semantic::Room *> importedMarkerRooms =
        p_otherMap_inout->getAllMarkerBasedMapRooms();
    std::vector<geometric::Plane *> importedPlanes =
        p_otherMap_inout->getAllPlanes();
    std::vector<vs_graphs::core::semantic::Passage *> importedPassages =
        p_otherMap_inout->getAllPassages();
    std::vector<vs_graphs::core::semantic::Floor *> importedFloors =
        p_otherMap_inout->getAllFloors();
    std::vector<semantic::Marker *> importedMarkers =
        p_otherMap_inout->getAllMarkers();

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

    const std::vector<KeyFrame *> destinationKeyFrames =
        p_currentMap_inout->getAllKeyFrames();
    const std::set<KeyFrame *> destinationKeyFrameSet(
        destinationKeyFrames.begin(),
        destinationKeyFrames.end());
    const std::vector<MapPoint *> destinationMapPoints =
        p_currentMap_inout->getAllMapPoints();
    const std::set<MapPoint *> destinationMapPointSet(
        destinationMapPoints.begin(),
        destinationMapPoints.end());

    for (KeyFrame *p_keyFrame : importedKeyFrames)
    {
        if (p_keyFrame == nullptr ||
            !ownerIsTransferable(p_keyFrame->getMap()) ||
            destinationKeyFrameSet.count(p_keyFrame) > 0U)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source "
                         "keyframe has inconsistent ownership."
                      << std::endl;
            return;
        }

        KeyFrame *p_indexedKeyFrame =
            p_currentMap_inout->getKeyFrameById(p_keyFrame->id);
        if (p_indexedKeyFrame != nullptr && p_indexedKeyFrame != p_keyFrame)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: KeyFrame ID "
                      << p_keyFrame->id << " collides in destination map."
                      << std::endl;
            return;
        }
    }

    for (MapPoint *p_mapPoint : importedMapPoints)
    {
        if (p_mapPoint == nullptr ||
            !ownerIsTransferable(p_mapPoint->getMap()) ||
            destinationMapPointSet.count(p_mapPoint) > 0U)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source map "
                         "point has inconsistent ownership."
                      << std::endl;
            return;
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
            return;
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
            return;
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
            return;
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
            return;
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
            return;
        }
    }

    std::vector<std::pair<geometric::Plane *, int>>  planeIdAssignments;
    std::vector<std::pair<semantic::Marker *, int>>  markerIdAssignments;
    std::vector<std::pair<semantic::Passage *, int>> passageIdAssignments;
    std::vector<std::pair<semantic::Room *, int>>    roomIdAssignments;
    std::vector<std::pair<semantic::Floor *, int>>   floorIdAssignments;

    if (!planImportedIds(p_currentMap_inout->getAllPlanes(),
                         importedPlanes,
                         "plane",
                         planeIdAssignments) ||
        !planImportedIds(p_currentMap_inout->getAllMarkers(),
                         importedMarkers,
                         "marker",
                         markerIdAssignments) ||
        !planImportedIds(p_currentMap_inout->getAllPassages(),
                         importedPassages,
                         "passage",
                         passageIdAssignments) ||
        !planImportedIds(p_currentMap_inout->getAllRooms(),
                         importedRooms,
                         "room",
                         roomIdAssignments) ||
        !planImportedIds(p_currentMap_inout->getAllFloors(),
                         importedFloors,
                         "floor",
                         floorIdAssignments))
    {
        return;
    }

    const std::vector<MapPoint *> importedReferenceMapPoints =
        p_otherMap_inout->getReferenceMapPoints();
    const std::vector<KeyFrame *> importedKeyFrameOrigins =
        p_otherMap_inout->keyFrameOrigins;
    KeyFrame *p_importedFirstRegionKeyFrame =
        p_otherMap_inout->p_firstRegionKeyFrame;

    const Eigen::Matrix3f R = T_otherToCurrent.linear().cast<float>();
    const Eigen::Vector3f t = T_otherToCurrent.translation().cast<float>();
    const Sophus::SE3f    T_otherToCurrent_SE3f(R, t);

    {
        std::scoped_lock mapUpdateLocks(p_currentMap_inout->mapUpdateMutex,
                                        p_otherMap_inout->mapUpdateMutex);

        p_otherMap_inout->applyScaledRotation(T_otherToCurrent_SE3f,
                                              1.0f,
                                              false);

        for (KeyFrame *p_keyFrame : importedKeyFrames)
        {
            p_keyFrame->updateMap(p_currentMap_inout);
            p_currentMap_inout->addKeyFrame(p_keyFrame);
            p_otherMap_inout->eraseKeyFrame(p_keyFrame);
        }

        for (MapPoint *p_mapPoint : importedMapPoints)
        {
            p_mapPoint->updateMap(p_currentMap_inout);
            p_currentMap_inout->addMapPoint(p_mapPoint);
            p_otherMap_inout->eraseMapPoint(p_mapPoint);
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
            p_currentMap_inout->addMapPlane(p_plane);
            p_otherMap_inout->eraseRoomWallPlane(p_plane);
            p_otherMap_inout->eraseMapPlane(p_plane);
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
            p_currentMap_inout->addMapMarker(p_marker);
            p_otherMap_inout->eraseMapMarker(p_marker);
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
            semantic::Passage *p_proxy =
                p_currentMap_inout->getPassageById(passageId);
            if (p_proxy != nullptr &&
                resurfaceProxyFromTransferred(p_proxy, p_passage))
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
            p_currentMap_inout->addMapPassage(p_passage);
            p_otherMap_inout->eraseMapPassage(p_passage);
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
                const auto markerIdIterator =
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
                p_currentMap_inout->addDetectedMapRoom(p_room);
            }
            else
            {
                p_currentMap_inout->addCandidateMapRoom(p_room);
            }
            p_otherMap_inout->eraseDetectedMapRoom(p_room);
            p_otherMap_inout->eraseMarkerBasedMapRoom(p_room);
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
            p_currentMap_inout->addMapFloor(p_floor);
            p_otherMap_inout->eraseMapFloor(p_floor);
        }

        std::vector<MapPoint *> mergedReferenceMapPoints =
            p_currentMap_inout->getReferenceMapPoints();
        for (MapPoint *p_mapPoint : importedReferenceMapPoints)
        {
            if (p_mapPoint != nullptr &&
                p_mapPoint->getMap() == p_currentMap_inout &&
                std::find(mergedReferenceMapPoints.begin(),
                          mergedReferenceMapPoints.end(),
                          p_mapPoint) == mergedReferenceMapPoints.end())
            {
                mergedReferenceMapPoints.push_back(p_mapPoint);
            }
        }
        p_currentMap_inout->setReferenceMapPoints(mergedReferenceMapPoints);
        p_otherMap_inout->setReferenceMapPoints({});

        for (KeyFrame *p_originKeyFrame : importedKeyFrameOrigins)
        {
            if (p_originKeyFrame != nullptr &&
                p_originKeyFrame->getMap() == p_currentMap_inout &&
                std::find(p_currentMap_inout->keyFrameOrigins.begin(),
                          p_currentMap_inout->keyFrameOrigins.end(),
                          p_originKeyFrame) ==
                    p_currentMap_inout->keyFrameOrigins.end())
            {
                p_currentMap_inout->keyFrameOrigins.push_back(p_originKeyFrame);
            }
        }
        p_otherMap_inout->keyFrameOrigins.clear();

        if (p_currentMap_inout->p_firstRegionKeyFrame == nullptr &&
            p_importedFirstRegionKeyFrame != nullptr &&
            p_importedFirstRegionKeyFrame->getMap() == p_currentMap_inout)
        {
            p_currentMap_inout->p_firstRegionKeyFrame =
                p_importedFirstRegionKeyFrame;
        }
        p_otherMap_inout->p_firstRegionKeyFrame = nullptr;

        p_currentMap_inout->setSkeletonClusterPoints({});
        p_currentMap_inout->setSkeletonEdges({});
        p_otherMap_inout->setSkeletonClusterPoints({});
        p_otherMap_inout->setSkeletonEdges({});
        p_otherMap_inout->clearTransferredEntityIndexes();

        /* Fuse duplicate floors: keep only one floor per map (system supports
         * single-floor semantics). Reassign rooms from duplicate floors to the
         * primary floor and erase the extras. */
        std::vector<semantic::Floor *> allFloors =
            p_currentMap_inout->getAllFloors();
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

                p_currentMap_inout->eraseMapFloor(p_duplicateFloor);
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

        semantic::Floor *p_mergedFloor = nullptr;
        if (semantic::Floor::selectBestObservedFloor(
                p_currentMap_inout->getAllFloors(),
                p_mergedFloor) != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: selectBestObservedFloor returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        if (p_mergedFloor != nullptr)
        {
            for (semantic::Room *p_room :
                 p_currentMap_inout->getAllDetectedMapRooms())
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

        for (semantic::Room *p_room : p_currentMap_inout->getAllRooms())
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
                    p_currentMap_inout->addRoomWallPlane(p_wall);
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
    setMapBad(p_otherMap_inout);
    changeMap(p_currentMap_inout);

    std::cout << "[Atlas::MergeMapPair] Merged map "
              << p_otherMap_inout->getId() << " into map "
              << p_currentMap_inout->getId() << " fused "
              << importedRooms.size() << " rooms into current map."
              << std::endl;
    std::cout << "[FloorVerify] Map#" << p_currentMap_inout->getId()
              << " and Map#" << p_otherMap_inout->getId()
              << " result=ACCEPTED committed=1" << std::endl;

    /* Notify downstream consumers that the current map changed. */
    p_currentMap_inout->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
