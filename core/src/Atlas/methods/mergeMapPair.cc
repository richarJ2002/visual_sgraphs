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
void Atlas::mergeMapPair(Map *p_currentMap_in, Map *p_otherMap_in)
{
    if (p_currentMap_in == nullptr || p_otherMap_in == nullptr)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: null map pointer."
                  << std::endl;
        return;
    }

    if (p_currentMap_in == p_otherMap_in)
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: identical maps."
                  << std::endl;
        return;
    }

    if (p_currentMap_in->isBad() || p_otherMap_in->isBad())
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: a map is bad."
                  << std::endl;
        return;
    }

    if (!isActiveMap(p_currentMap_in) || !isActiveMap(p_otherMap_in))
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: map not active."
                  << std::endl;
        return;
    }

    /* Pair the walls of both maps using their shared room identity tags. */
    std::vector<Eigen::Vector3d> normalsCurrent, centroidsCurrent;
    std::vector<Eigen::Vector3d> normalsOther, centroidsOther;

    if (!utils::utils::Utils::collectCorrespondingWalls(p_currentMap_in,
                                                        p_otherMap_in,
                                                        normalsCurrent,
                                                        centroidsCurrent,
                                                        normalsOther,
                                                        centroidsOther))
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: fewer than three "
                     "wall correspondences."
                  << std::endl;
        return;
    }

    /* Horn's closed-form transform maps other-frame points into current frame.
     */
    const Eigen::Isometry3d T_otherToCurrent =
        utils::utils::Utils::computeMapTransform_Horn(normalsOther,
                                                      centroidsOther,
                                                      normalsCurrent,
                                                      centroidsCurrent);

    if (!T_otherToCurrent.matrix().allFinite())
    {
        std::cerr << "[Atlas::MergeMapPair] Aborting merge: invalid transform."
                  << std::endl;
        return;
    }

    /* Compare floor identities in the surviving map frame without mutating
     * either map. A mismatch rejects the merge before any entity is moved. */
    semantic::Floor *p_currentFloor = semantic::Floor::selectBestObservedFloor(
        p_currentMap_in->getAllFloors());
    semantic::Floor *p_otherFloor =
        semantic::Floor::selectBestObservedFloor(p_otherMap_in->getAllFloors());

    const std::optional<semantic::Floor::PlaneIdentity> currentFloorIdentity =
        p_currentFloor != nullptr ? p_currentFloor->getPlaneIdentity()
                                  : std::nullopt;
    const std::optional<semantic::Floor::PlaneIdentity> otherFloorIdentity =
        p_otherFloor != nullptr ? p_otherFloor->getPlaneIdentity()
                                : std::nullopt;

    const g2o::Sim3 floorTransform_otherWorldToCurrentWorld(
        T_otherToCurrent.linear(),
        T_otherToCurrent.translation(),
        1.0);

    if (currentFloorIdentity.has_value() && otherFloorIdentity.has_value())
    {
        const std::optional<semantic::Floor::PlaneIdentity>
            transformedOtherFloorIdentity =
                semantic::Floor::transformPlaneIdentity(
                    *otherFloorIdentity,
                    floorTransform_otherWorldToCurrentWorld);

        double floorNormalAngle_deg = std::numeric_limits<double>::infinity();
        double floorOffset_m        = std::numeric_limits<double>::infinity();

        if (!transformedOtherFloorIdentity.has_value() ||
            !semantic::Floor::planeIdentitiesMatch(
                *currentFloorIdentity,
                transformedOtherFloorIdentity.value_or(*otherFloorIdentity),
                semantic::Floor::kMergeMaxPlaneNormalAngle_deg,
                semantic::Floor::kMergeMaxPlaneOffset_m,
                floorNormalAngle_deg,
                floorOffset_m))
        {
            std::cerr << "[FloorVerify] Rejecting merge: Map#"
                      << p_currentMap_in->getId() << " and Map#"
                      << p_otherMap_in->getId()
                      << " floor planes mismatch (angle="
                      << floorNormalAngle_deg
                      << " deg, offset=" << floorOffset_m << " m; limits="
                      << semantic::Floor::kMergeMaxPlaneNormalAngle_deg
                      << " deg/" << semantic::Floor::kMergeMaxPlaneOffset_m
                      << " m). result=REJECTED committed=0" << std::endl;
            return;
        }

        std::cout << "[FloorVerify] Map#" << p_currentMap_in->getId()
                  << " and Map#" << p_otherMap_in->getId()
                  << " floor planes match (angle=" << floorNormalAngle_deg
                  << " deg, offset=" << floorOffset_m
                  << " m). result=ACCEPTED committed=0" << std::endl;
    }
    else
    {
        std::cout << "[FloorVerify] Map#" << p_currentMap_in->getId()
                  << " and Map#" << p_otherMap_in->getId()
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
        p_otherMap_in->getAllKeyFrames();
    std::vector<MapPoint *> importedMapPoints =
        p_otherMap_in->getAllMapPoints();
    std::vector<semantic::Room *> importedDetectedRooms =
        p_otherMap_in->getAllDetectedMapRooms();
    std::vector<semantic::Room *> importedMarkerRooms =
        p_otherMap_in->getAllMarkerBasedMapRooms();
    std::vector<geometric::Plane *> importedPlanes =
        p_otherMap_in->getAllPlanes();
    std::vector<vs_graphs::core::semantic::Passage *> importedPassages =
        p_otherMap_in->getAllPassages();
    std::vector<vs_graphs::core::semantic::Floor *> importedFloors =
        p_otherMap_in->getAllFloors();
    std::vector<semantic::Marker *> importedMarkers =
        p_otherMap_in->getAllMarkers();

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

    const auto ownerIsTransferable = [p_otherMap_in](Map *p_ownerMap)
    { return p_ownerMap == p_otherMap_in; };

    const std::vector<KeyFrame *> destinationKeyFrames =
        p_currentMap_in->getAllKeyFrames();
    const std::set<KeyFrame *> destinationKeyFrameSet(
        destinationKeyFrames.begin(),
        destinationKeyFrames.end());
    const std::vector<MapPoint *> destinationMapPoints =
        p_currentMap_in->getAllMapPoints();
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
            p_currentMap_in->getKeyFrameById(p_keyFrame->mnId);
        if (p_indexedKeyFrame != nullptr && p_indexedKeyFrame != p_keyFrame)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: KeyFrame ID "
                      << p_keyFrame->mnId << " collides in destination map."
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
        if (p_plane == nullptr || !ownerIsTransferable(p_plane->getMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source plane "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (semantic::Marker *p_marker : importedMarkers)
    {
        if (p_marker == nullptr || !ownerIsTransferable(p_marker->getMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source marker "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (semantic::Passage *p_passage : importedPassages)
    {
        if (p_passage == nullptr || !ownerIsTransferable(p_passage->getMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source passage "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (semantic::Room *p_room : importedRooms)
    {
        if (p_room == nullptr || !ownerIsTransferable(p_room->getMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source room "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (semantic::Floor *p_floor : importedFloors)
    {
        if (p_floor == nullptr || !ownerIsTransferable(p_floor->getMap()))
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

    if (!planImportedIds(p_currentMap_in->getAllPlanes(),
                         importedPlanes,
                         "plane",
                         planeIdAssignments) ||
        !planImportedIds(p_currentMap_in->getAllMarkers(),
                         importedMarkers,
                         "marker",
                         markerIdAssignments) ||
        !planImportedIds(p_currentMap_in->getAllPassages(),
                         importedPassages,
                         "passage",
                         passageIdAssignments) ||
        !planImportedIds(p_currentMap_in->getAllRooms(),
                         importedRooms,
                         "room",
                         roomIdAssignments) ||
        !planImportedIds(p_currentMap_in->getAllFloors(),
                         importedFloors,
                         "floor",
                         floorIdAssignments))
    {
        return;
    }

    const std::vector<MapPoint *> importedReferenceMapPoints =
        p_otherMap_in->getReferenceMapPoints();
    const std::vector<KeyFrame *> importedKeyFrameOrigins =
        p_otherMap_in->keyFrameOrigins;
    KeyFrame *p_importedFirstRegionKeyFrame =
        p_otherMap_in->p_firstRegionKeyFrame;

    const Eigen::Matrix3f R = T_otherToCurrent.linear().cast<float>();
    const Eigen::Vector3f t = T_otherToCurrent.translation().cast<float>();
    const Sophus::SE3f    T_otherToCurrent_SE3f(R, t);

    {
        std::scoped_lock mapUpdateLocks(p_currentMap_in->mMutexMapUpdate,
                                        p_otherMap_in->mMutexMapUpdate);

        p_otherMap_in->applyScaledRotation(T_otherToCurrent_SE3f, 1.0f, false);

        for (KeyFrame *p_keyFrame : importedKeyFrames)
        {
            p_keyFrame->updateMap(p_currentMap_in);
            p_currentMap_in->addKeyFrame(p_keyFrame);
            p_otherMap_in->eraseKeyFrame(p_keyFrame);
        }

        for (MapPoint *p_mapPoint : importedMapPoints)
        {
            p_mapPoint->updateMap(p_currentMap_in);
            p_currentMap_in->addMapPoint(p_mapPoint);
            p_otherMap_in->eraseMapPoint(p_mapPoint);
        }

        for (const auto &[p_plane, assignedId] : planeIdAssignments)
        {
            p_plane->setId(assignedId);
            p_plane->setMap(p_currentMap_in);
            p_currentMap_in->addMapPlane(p_plane);
            p_otherMap_in->eraseRoomWallPlane(p_plane);
            p_otherMap_in->eraseMapPlane(p_plane);
        }

        std::unordered_map<int, int> importedMarkerIdRemap;
        for (const auto &[p_marker, assignedId] : markerIdAssignments)
        {
            importedMarkerIdRemap.insert_or_assign(p_marker->getId(),
                                                   assignedId);
            p_marker->setId(assignedId);
            p_marker->setMap(p_currentMap_in);
            p_currentMap_in->addMapMarker(p_marker);
            p_otherMap_in->eraseMapMarker(p_marker);
        }

        for (const auto &[p_passage, assignedId] : passageIdAssignments)
        {
            /* Same stable lineage as a current-map recovery proxy: fold the
             * transferred (now in-frame) state into the proxy instead of
             * duplicating the doorway. The transferred object retires with
             * the absorbed map; it never enters the current map. */
            semantic::Passage *p_proxy =
                p_currentMap_in->getPassageById(p_passage->getId());
            if (p_proxy != nullptr &&
                resurfaceProxyFromTransferred(p_proxy, p_passage))
            {
                p_passage->setBad();
                continue;
            }
            p_passage->setId(assignedId);
            p_passage->setMap(p_currentMap_in);
            p_currentMap_in->addMapPassage(p_passage);
            p_otherMap_in->eraseMapPassage(p_passage);
        }

        for (semantic::Room *p_room : importedRooms)
        {
            semantic::Marker *p_metaMarker = p_room->getMetaMarker();
            if (p_metaMarker != nullptr)
            {
                p_room->setMetaMarkerId(p_metaMarker->getId());
            }
            else
            {
                const auto markerIdIterator =
                    importedMarkerIdRemap.find(p_room->getMetaMarkerId());
                if (markerIdIterator != importedMarkerIdRemap.end())
                {
                    p_room->setMetaMarkerId(markerIdIterator->second);
                }
            }
        }

        for (const auto &[p_room, assignedId] : roomIdAssignments)
        {
            p_room->setId(assignedId);
            p_room->setMap(p_currentMap_in);
            if (importedDetectedRoomSet.count(p_room) > 0U)
            {
                p_currentMap_in->addDetectedMapRoom(p_room);
            }
            else
            {
                p_currentMap_in->addCandidateMapRoom(p_room);
            }
            p_otherMap_in->eraseDetectedMapRoom(p_room);
            p_otherMap_in->eraseMarkerBasedMapRoom(p_room);
        }

        for (const auto &[p_floor, assignedId] : floorIdAssignments)
        {
            p_floor->setId(assignedId);
            p_floor->setMap(p_currentMap_in);
            p_currentMap_in->addMapFloor(p_floor);
            p_otherMap_in->eraseMapFloor(p_floor);
        }

        std::vector<MapPoint *> mergedReferenceMapPoints =
            p_currentMap_in->getReferenceMapPoints();
        for (MapPoint *p_mapPoint : importedReferenceMapPoints)
        {
            if (p_mapPoint != nullptr &&
                p_mapPoint->getMap() == p_currentMap_in &&
                std::find(mergedReferenceMapPoints.begin(),
                          mergedReferenceMapPoints.end(),
                          p_mapPoint) == mergedReferenceMapPoints.end())
            {
                mergedReferenceMapPoints.push_back(p_mapPoint);
            }
        }
        p_currentMap_in->setReferenceMapPoints(mergedReferenceMapPoints);
        p_otherMap_in->setReferenceMapPoints({});

        for (KeyFrame *p_originKeyFrame : importedKeyFrameOrigins)
        {
            if (p_originKeyFrame != nullptr &&
                p_originKeyFrame->getMap() == p_currentMap_in &&
                std::find(p_currentMap_in->keyFrameOrigins.begin(),
                          p_currentMap_in->keyFrameOrigins.end(),
                          p_originKeyFrame) ==
                    p_currentMap_in->keyFrameOrigins.end())
            {
                p_currentMap_in->keyFrameOrigins.push_back(p_originKeyFrame);
            }
        }
        p_otherMap_in->keyFrameOrigins.clear();

        if (p_currentMap_in->p_firstRegionKeyFrame == nullptr &&
            p_importedFirstRegionKeyFrame != nullptr &&
            p_importedFirstRegionKeyFrame->getMap() == p_currentMap_in)
        {
            p_currentMap_in->p_firstRegionKeyFrame =
                p_importedFirstRegionKeyFrame;
        }
        p_otherMap_in->p_firstRegionKeyFrame = nullptr;

        p_currentMap_in->setSkeletonClusterPoints({});
        p_currentMap_in->setSkeletonEdges({});
        p_otherMap_in->setSkeletonClusterPoints({});
        p_otherMap_in->setSkeletonEdges({});
        p_otherMap_in->clearTransferredEntityIndexes();

        /* Fuse duplicate floors: keep only one floor per map (system supports
         * single-floor semantics). Reassign rooms from duplicate floors to the
         * primary floor and erase the extras. */
        std::vector<semantic::Floor *> allFloors =
            p_currentMap_in->getAllFloors();
        if (allFloors.size() > 1)
        {
            semantic::Floor *p_keeperFloor =
                semantic::Floor::selectBestObservedFloor(allFloors);
            for (semantic::Floor *p_duplicateFloor : allFloors)
            {
                if (p_duplicateFloor == nullptr ||
                    p_duplicateFloor == p_keeperFloor)
                {
                    continue;
                }

                for (semantic::Room *p_room : p_duplicateFloor->getRooms())
                {
                    if (p_room != nullptr && !p_room->isBad())
                    {
                        p_keeperFloor->addRoom(p_room);
                    }
                }

                p_currentMap_in->eraseMapFloor(p_duplicateFloor);
                std::cout
                    << "[Atlas::MergeMapPair] Fused duplicate semantic::Floor#"
                    << p_duplicateFloor->getId() << " into semantic::Floor#"
                    << p_keeperFloor->getId()
                    << " and retained the better-observed plane identity."
                    << std::endl;
            }
        }

        utils::utils::Utils::fuseDuplicateRoomsAfterMerge(p_currentMap_in,
                                                          importedRooms);

        semantic::Floor *p_mergedFloor =
            semantic::Floor::selectBestObservedFloor(
                p_currentMap_in->getAllFloors());
        if (p_mergedFloor != nullptr)
        {
            for (semantic::Room *p_room :
                 p_currentMap_in->getAllDetectedMapRooms())
            {
                if (p_room != nullptr && !p_room->isBad())
                {
                    p_mergedFloor->addRoom(p_room);
                }
            }
        }

        for (semantic::Room *p_room : p_currentMap_in->getAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (geometric::Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    p_currentMap_in->addRoomWallPlane(p_wall);
                }
            }
        }

        utils::utils::Utils::reAssociatePassages(this);
    }

    /* Retire the absorbed map while keeping the current map active. */
    setMapBad(p_otherMap_in);
    changeMap(p_currentMap_in);

    std::cout << "[Atlas::MergeMapPair] Merged map " << p_otherMap_in->getId()
              << " into map " << p_currentMap_in->getId() << " fused "
              << importedRooms.size() << " rooms into current map."
              << std::endl;
    std::cout << "[FloorVerify] Map#" << p_currentMap_in->getId() << " and Map#"
              << p_otherMap_in->getId() << " result=ACCEPTED committed=1"
              << std::endl;

    /* Notify downstream consumers that the current map changed. */
    p_currentMap_in->increaseChangeIndex();
}

} // namespace core
} // namespace vs_graphs
