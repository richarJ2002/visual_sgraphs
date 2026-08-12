/**
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
#include "Utils.h"
#include "Viewer.h"

#include "Geometric/Plane.h"
#include "GeometricCamera.h"
#include "KannalaBrandt8.h"
#include "Pinhole.h"
#include "Semantic/Room.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <set>
#include <sophus/se3.hpp>
#include <unordered_map>

namespace ORB_SLAM3
{
namespace
{
template <typename Entity>
bool planImportedIds(const std::vector<Entity *> &existingEntities_in,
                     const std::vector<Entity *> &importedEntities_in,
                     const char                  *entityName_in,
                     std::vector<std::pair<Entity *, int>> &assignments_out)
{
    const std::set<Entity *> importedEntities(importedEntities_in.begin(),
                                              importedEntities_in.end());
    std::set<int> destinationIds;

    for (Entity *p_entity : existingEntities_in)
    {
        if (p_entity == nullptr)
        {
            continue;
        }

        if (importedEntities.count(p_entity) > 0U)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source "
                      << entityName_in
                      << " already belongs to the destination container."
                      << std::endl;
            return false;
        }

        if (p_entity->getId() >= 0)
        {
            destinationIds.insert(p_entity->getId());
        }
    }

    std::vector<Entity *> orderedImportedEntities;
    orderedImportedEntities.reserve(importedEntities_in.size());
    std::set<int> importedOriginalIds;

    for (Entity *p_entity : importedEntities_in)
    {
        if (p_entity == nullptr)
        {
            continue;
        }

        if (!importedOriginalIds.insert(p_entity->getId()).second)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: duplicate "
                      << entityName_in << " ID " << p_entity->getId()
                      << " in source map." << std::endl;
            return false;
        }

        orderedImportedEntities.push_back(p_entity);
    }

    std::sort(orderedImportedEntities.begin(),
              orderedImportedEntities.end(),
              [](const Entity *p_first, const Entity *p_second)
              { return p_first->getId() < p_second->getId(); });

    std::set<int> reservedIds = destinationIds;
    for (Entity *p_entity : orderedImportedEntities)
    {
        if (p_entity->getId() >= 0 &&
            destinationIds.count(p_entity->getId()) == 0U)
        {
            reservedIds.insert(p_entity->getId());
        }
    }

    int nextAvailableId = reservedIds.empty() ? 0 : *reservedIds.rbegin() + 1;
    assignments_out.clear();
    assignments_out.reserve(orderedImportedEntities.size());

    for (Entity *p_entity : orderedImportedEntities)
    {
        int assignedId = p_entity->getId();
        if (assignedId < 0 || destinationIds.count(assignedId) > 0U)
        {
            while (reservedIds.count(nextAvailableId) > 0U)
            {
                ++nextAvailableId;
            }
            assignedId = nextAvailableId++;
        }

        reservedIds.insert(assignedId);
        assignments_out.emplace_back(p_entity, assignedId);
    }

    return true;
}
} // namespace

Atlas::Atlas()
{
    mpCurrentMap = static_cast<Map *>(NULL);
}

Atlas::Atlas(int initKFid) :
    mnLastInitKFidMap(initKFid),
    mHasViewer(false)
{
    mpCurrentMap = static_cast<Map *>(NULL);
    CreateNewMap();
}

Atlas::~Atlas()
{
    /*
     * A map may move through the active, pending-retirement, and retired sets
     * during its lifetime. Delete the union exactly once at Atlas shutdown.
     */
    std::set<Map *> mapsToDelete = mspMaps;
    mapsToDelete.insert(mspBadMaps.begin(), mspBadMaps.end());
    mapsToDelete.insert(mspRetiredMaps.begin(), mspRetiredMaps.end());

    for (Map *p_map : mapsToDelete)
    {
        if (p_map != nullptr)
        {
            delete p_map;
        }
    }

    mspMaps.clear();
    mspBadMaps.clear();
    mspRetiredMaps.clear();
}

void Atlas::CreateNewMap()
{
    std::unique_lock<std::mutex> atlasLock(mMutexAtlas);
    createNewMapWhileAtlasLocked();
}

void Atlas::createNewMapWhileAtlasLocked()
{
    std::cout << "\n[Atlas]" << std::endl;
    std::cout << "- Creating a new map (MapId: " << Map::nNextId
              << ", Init KeyFrame: " << mnLastInitKFidMap << ") ..."
              << std::endl;

    if (mpCurrentMap)
    {
        if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() + 1;

        /* Snapshot room geometry before the map is stranded so that
         * rooms in the new map can inherit identity tags.            */
        exportRoomContextFromCurrentMap();

        mpCurrentMap->SetStoredMap();
        std::cout << "- The created map with MapId #" << mpCurrentMap->GetId()
                  << " has been stored!" << std::endl;
    }

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutexAtlas);
    std::cout << "\n[Atlas]" << std::endl;
    std::cout << "- Changing to map with MapId #" << pMap->GetId() << " ..."
              << std::endl;

    if (mpCurrentMap)
        mpCurrentMap->SetStoredMap();

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer *pViewer)
{
    mpViewer   = pViewer;
    mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame *pKF)
{
    Map *pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint *pMP)
{
    Map *pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

void Atlas::AddMapMarker(Marker *marker)
{
    Map *pMapMP = marker->getMap();
    pMapMP->AddMapMarker(marker);
}

void Atlas::AddMapPlane(ORB_SLAM3::Plane *plane)
{
    ORB_SLAM3::Map *pMapMP = plane->GetMap();
    pMapMP->AddMapPlane(plane);
}

void Atlas::AddRoomWallPlane(ORB_SLAM3::Plane *pPlane)
{
    ORB_SLAM3::Map *pMapMP = pPlane->GetMap();
    pMapMP->AddRoomWallPlane(pPlane);
}

void Atlas::AddMapPassage(ORB_SLAM3::Passage *passage)
{
    ORB_SLAM3::Map *pMapMP = passage->getMap();
    pMapMP->AddMapPassage(passage);
}

void Atlas::AddDetectedMapRoom(Room *room)
{
    Map *pMapMP = room->getMap();
    pMapMP->AddDetectedMapRoom(room);
}

void Atlas::AddCandidateMapRoom(Room *room)
{
    Map *pMapMP = room->getMap();
    pMapMP->AddCandidateMapRoom(room);
}

void Atlas::AddMapFloor(Floor *floor)
{
    Map *pMapMP = floor->getMap();
    pMapMP->AddMapFloor(floor);
}

GeometricCamera *Atlas::AddCamera(GeometricCamera *pCam)
{
    // Check if the camera already exists
    bool bAlreadyInMap = false;
    int  index_cam     = -1;
    for (size_t i = 0; i < mvpCameras.size(); ++i)
    {
        GeometricCamera *pCam_i = mvpCameras[i];
        if (!pCam)
            std::cout << "Not pCam" << std::endl;
        if (!pCam_i)
            std::cout << "Not pCam_i" << std::endl;
        if (pCam->GetType() != pCam_i->GetType())
            continue;

        if (pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            if (((Pinhole *)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam     = i;
            }
        }
        else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            if (((KannalaBrandt8 *)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam     = i;
            }
        }
    }

    if (bAlreadyInMap)
    {
        return mvpCameras[index_cam];
    }
    else
    {
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector<GeometricCamera *> Atlas::GetAllCameras()
{
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned int Atlas::MarkersInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->MarkersInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<std::vector<Eigen::Vector3d>> Atlas::GetSkeletoClusterPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetSkeletonClusterPoints();
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
    Atlas::GetSkeletonEdges(void)
{
    /* Lock access to the active map */
    unique_lock<mutex> lock(mMutexAtlas);

    if (mpCurrentMap == nullptr)
    {
        return {};
    }

    /* Return the connected edges from the active map */
    return mpCurrentMap->GetSkeletonEdges();
}

void Atlas::SetSkeletonEdges(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        &newSkeletonEdges)
{
    /* Lock access to the active map */
    unique_lock<mutex> lock(mMutexAtlas);

    if (mpCurrentMap == nullptr)
    {
        return;
    }

    /* Store the connected edges in the active map */
    mpCurrentMap->SetSkeletonEdges(newSkeletonEdges);
}

void Atlas::SetSkeletonClusterPoints(
    const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints)
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetSkeletonClusterPoints(newClusterPoints);
}

std::vector<KeyFrame *> Atlas::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

KeyFrame *Atlas::GetKeyFrameById(long unsigned int mnId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetKeyFrameById(mnId)
                                   : nullptr;
}

ORB_SLAM3::Passage *Atlas::GetPassageById(int passageId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetPassageById(passageId)
                                   : nullptr;
}

Floor *Atlas::GetFloorById(int floorId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetFloorById(floorId)
                                   : nullptr;
}

Plane *Atlas::GetPlaneById(int planeId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetPlaneById(planeId)
                                   : nullptr;
}

ORB_SLAM3::Plane *Atlas::GetRoomWallPlaneById(int planeId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetRoomWallPlaneById(planeId)
                                   : nullptr;
}

Marker *Atlas::GetMarkerById(int markerId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetMarkerById(markerId)
                                   : nullptr;
}

std::vector<MapPoint *> Atlas::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<Marker *> Atlas::GetAllMarkers()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMarkers();
}

std::vector<ORB_SLAM3::Plane *> Atlas::GetAllPlanes()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllPlanes();
}

std::vector<ORB_SLAM3::Passage *> Atlas::GetAllPassages()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllPassages();
}

std::vector<Room *> Atlas::GetAllRooms()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllRooms();
}

std::vector<Room *> Atlas::GetAllDetectedMapRooms()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllDetectedMapRooms();
}

std::vector<Room *> Atlas::GetAllMarkerBasedMapRooms()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMarkerBasedMapRooms();
}

std::vector<Room *> Atlas::GetAllCandidateMapRooms()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllCandidateMapRooms();
}

std::vector<Floor *> Atlas::GetAllFloors()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllFloors();
}

Plane *Atlas::GetBiggestGroundPlane()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetBiggestGroundPlane()
                                   : nullptr;
}

std::vector<MapPoint *> Atlas::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

vector<Map *> Atlas::GetAllMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map *elem1, Map *elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    vector<Map *> vMaps(mspMaps.begin(), mspMaps.end());
    sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

bool Atlas::isActiveMap(Map *p_map_in)
{
    if (p_map_in == nullptr)
    {
        return false;
    }

    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.count(p_map_in) > 0 && !p_map_in->IsBad();
}

int Atlas::CountMaps()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mspMaps.clear();
    mpCurrentMap      = static_cast<Map *>(NULL);
    mnLastInitKFidMap = 0;
}

Map *Atlas::GetCurrentMap()
{
    std::unique_lock<std::mutex> atlasLock(mMutexAtlas);

    if (!mpCurrentMap)
    {
        createNewMapWhileAtlasLocked();
    }

    while (mpCurrentMap != nullptr && mpCurrentMap->IsBad())
    {
        /* Allow ChangeMap() to install the merge survivor while waiting. */
        atlasLock.unlock();
        usleep(3000);
        atlasLock.lock();
    }

    if (mpCurrentMap == nullptr)
    {
        createNewMapWhileAtlasLocked();
    }

    return mpCurrentMap;
}

std::unique_lock<std::mutex> Atlas::acquireSemanticUpdateLock()
{
    return std::unique_lock<std::mutex>(mMutexSemanticUpdate);
}

void Atlas::SetMapBad(Map *p_map_in)
{
    if (p_map_in == nullptr)
    {
        return;
    }

    std::unique_lock<std::mutex> atlasLock(mMutexAtlas);

    mspMaps.erase(p_map_in);
    p_map_in->SetBad();

    mspBadMaps.insert(p_map_in);
}

void Atlas::RemoveBadMaps()
{
    std::unique_lock<std::mutex> atlasLock(mMutexAtlas);

    /* Preserve ownership until no runtime reader can retain a raw Map*. */
    mspRetiredMaps.insert(mspBadMaps.begin(), mspBadMaps.end());
    mspBadMaps.clear();
}

/**
 * @brief Merges the semantic graph of the other map into the current map.
 *
 * Mirrors the semantic-transfer pattern of LoopClosing::MergeLocal using
 * Horn's deterministic closed-form solution; no g2o types are used. Caller
 * must already hold the semantic-update lock.
 */
void Atlas::MergeMapPair(Map *p_currentMap_in, Map *p_otherMap_in)
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

    if (p_currentMap_in->IsBad() || p_otherMap_in->IsBad())
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

    if (!Utils::collectCorrespondingWalls(p_currentMap_in,
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
        Utils::computeMapTransform_Horn(normalsOther,
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
    Floor *p_currentFloor = Floor::selectBestObservedFloor(
        p_currentMap_in->GetAllFloors());
    Floor *p_otherFloor = Floor::selectBestObservedFloor(
        p_otherMap_in->GetAllFloors());

    const std::optional<Floor::PlaneIdentity> currentFloorIdentity =
        p_currentFloor != nullptr ? p_currentFloor->getPlaneIdentity()
                                  : std::nullopt;
    const std::optional<Floor::PlaneIdentity> otherFloorIdentity =
        p_otherFloor != nullptr ? p_otherFloor->getPlaneIdentity()
                                : std::nullopt;

    const g2o::Sim3 floorTransform_otherWorldToCurrentWorld(
        T_otherToCurrent.linear(), T_otherToCurrent.translation(), 1.0);

    if (currentFloorIdentity.has_value() && otherFloorIdentity.has_value())
    {
        const std::optional<Floor::PlaneIdentity>
            transformedOtherFloorIdentity = Floor::transformPlaneIdentity(
                *otherFloorIdentity,
                floorTransform_otherWorldToCurrentWorld);

        double floorNormalAngle_deg = std::numeric_limits<double>::infinity();
        double floorOffset_m        = std::numeric_limits<double>::infinity();

        if (!transformedOtherFloorIdentity.has_value() ||
            !Floor::planeIdentitiesMatch(
                *currentFloorIdentity,
                transformedOtherFloorIdentity.value_or(*otherFloorIdentity),
                Floor::kMergeMaxPlaneNormalAngle_deg,
                Floor::kMergeMaxPlaneOffset_m,
                floorNormalAngle_deg,
                floorOffset_m))
        {
            std::cerr << "[FloorVerify] Rejecting merge: Map#"
                      << p_currentMap_in->GetId() << " and Map#"
                      << p_otherMap_in->GetId()
                      << " floor planes mismatch (angle="
                      << floorNormalAngle_deg << " deg, offset="
                      << floorOffset_m << " m; limits="
                      << Floor::kMergeMaxPlaneNormalAngle_deg << " deg/"
                      << Floor::kMergeMaxPlaneOffset_m
                      << " m). result=REJECTED committed=0" << std::endl;
            return;
        }

        std::cout << "[FloorVerify] Map#" << p_currentMap_in->GetId()
                  << " and Map#" << p_otherMap_in->GetId()
                  << " floor planes match (angle=" << floorNormalAngle_deg
                  << " deg, offset=" << floorOffset_m
                  << " m). result=ACCEPTED committed=0" << std::endl;
    }
    else
    {
        std::cout << "[FloorVerify] Map#" << p_currentMap_in->GetId()
                  << " and Map#" << p_otherMap_in->GetId()
                  << " floor verification deferred (current="
                  << (currentFloorIdentity.has_value() ? "valid" : "missing")
                  << ", other="
                  << (otherFloorIdentity.has_value() ? "valid" : "missing")
                  << "); result=DEFERRED committed=0"
                  << std::endl;
        return;
    }

    /* Snapshot every source-owned object and preflight destination indexes
     * before changing geometry, ownership, or any externally visible ID. */
    std::vector<KeyFrame *> importedKeyFrames =
        p_otherMap_in->GetAllKeyFrames();
    std::vector<MapPoint *> importedMapPoints =
        p_otherMap_in->GetAllMapPoints();
    std::vector<Room *> importedDetectedRooms =
        p_otherMap_in->GetAllDetectedMapRooms();
    std::vector<Room *> importedMarkerRooms =
        p_otherMap_in->GetAllMarkerBasedMapRooms();
    std::vector<Plane *> importedPlanes = p_otherMap_in->GetAllPlanes();
    std::vector<ORB_SLAM3::Passage *> importedPassages =
        p_otherMap_in->GetAllPassages();
    std::vector<ORB_SLAM3::Floor *> importedFloors =
        p_otherMap_in->GetAllFloors();
    std::vector<Marker *> importedMarkers = p_otherMap_in->GetAllMarkers();

    const std::set<Room *> importedDetectedRoomSet(
        importedDetectedRooms.begin(), importedDetectedRooms.end());
    importedMarkerRooms.erase(
        std::remove_if(importedMarkerRooms.begin(),
                       importedMarkerRooms.end(),
                       [&importedDetectedRoomSet](Room *p_room)
                       { return importedDetectedRoomSet.count(p_room) > 0U; }),
        importedMarkerRooms.end());

    std::vector<Room *> importedRooms = importedDetectedRooms;
    importedRooms.insert(importedRooms.end(),
                         importedMarkerRooms.begin(),
                         importedMarkerRooms.end());

    const auto ownerIsTransferable =
        [p_otherMap_in](Map *p_ownerMap)
    { return p_ownerMap == p_otherMap_in; };

    const std::vector<KeyFrame *> destinationKeyFrames =
        p_currentMap_in->GetAllKeyFrames();
    const std::set<KeyFrame *> destinationKeyFrameSet(
        destinationKeyFrames.begin(), destinationKeyFrames.end());
    const std::vector<MapPoint *> destinationMapPoints =
        p_currentMap_in->GetAllMapPoints();
    const std::set<MapPoint *> destinationMapPointSet(
        destinationMapPoints.begin(), destinationMapPoints.end());

    for (KeyFrame *p_keyFrame : importedKeyFrames)
    {
        if (p_keyFrame == nullptr ||
            !ownerIsTransferable(p_keyFrame->GetMap()) ||
            destinationKeyFrameSet.count(p_keyFrame) > 0U)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source "
                         "keyframe has inconsistent ownership."
                      << std::endl;
            return;
        }

        KeyFrame *p_indexedKeyFrame =
            p_currentMap_in->GetKeyFrameById(p_keyFrame->mnId);
        if (p_indexedKeyFrame != nullptr &&
            p_indexedKeyFrame != p_keyFrame)
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
            !ownerIsTransferable(p_mapPoint->GetMap()) ||
            destinationMapPointSet.count(p_mapPoint) > 0U)
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source map "
                         "point has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (Plane *p_plane : importedPlanes)
    {
        if (p_plane == nullptr || !ownerIsTransferable(p_plane->GetMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source plane "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (Marker *p_marker : importedMarkers)
    {
        if (p_marker == nullptr || !ownerIsTransferable(p_marker->getMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source marker "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (Passage *p_passage : importedPassages)
    {
        if (p_passage == nullptr ||
            !ownerIsTransferable(p_passage->getMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source passage "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (Room *p_room : importedRooms)
    {
        if (p_room == nullptr || !ownerIsTransferable(p_room->getMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source room "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    for (Floor *p_floor : importedFloors)
    {
        if (p_floor == nullptr || !ownerIsTransferable(p_floor->getMap()))
        {
            std::cerr << "[Atlas::MergeMapPair] Aborting merge: source floor "
                         "has inconsistent ownership."
                      << std::endl;
            return;
        }
    }

    std::vector<std::pair<Plane *, int>> planeIdAssignments;
    std::vector<std::pair<Marker *, int>> markerIdAssignments;
    std::vector<std::pair<Passage *, int>> passageIdAssignments;
    std::vector<std::pair<Room *, int>> roomIdAssignments;
    std::vector<std::pair<Floor *, int>> floorIdAssignments;

    if (!planImportedIds(p_currentMap_in->GetAllPlanes(),
                         importedPlanes,
                         "plane",
                         planeIdAssignments) ||
        !planImportedIds(p_currentMap_in->GetAllMarkers(),
                         importedMarkers,
                         "marker",
                         markerIdAssignments) ||
        !planImportedIds(p_currentMap_in->GetAllPassages(),
                         importedPassages,
                         "passage",
                         passageIdAssignments) ||
        !planImportedIds(p_currentMap_in->GetAllRooms(),
                         importedRooms,
                         "room",
                         roomIdAssignments) ||
        !planImportedIds(p_currentMap_in->GetAllFloors(),
                         importedFloors,
                         "floor",
                         floorIdAssignments))
    {
        return;
    }

    const std::vector<MapPoint *> importedReferenceMapPoints =
        p_otherMap_in->GetReferenceMapPoints();
    const std::vector<KeyFrame *> importedKeyFrameOrigins =
        p_otherMap_in->mvpKeyFrameOrigins;
    KeyFrame *p_importedFirstRegionKeyFrame =
        p_otherMap_in->mpFirstRegionKF;

    const Eigen::Matrix3f R = T_otherToCurrent.linear().cast<float>();
    const Eigen::Vector3f t = T_otherToCurrent.translation().cast<float>();
    const Sophus::SE3f    T_otherToCurrent_SE3f(R, t);

    {
        std::scoped_lock mapUpdateLocks(p_currentMap_in->mMutexMapUpdate,
                                        p_otherMap_in->mMutexMapUpdate);

        p_otherMap_in->ApplyScaledRotation(T_otherToCurrent_SE3f, 1.0f, false);

        for (KeyFrame *p_keyFrame : importedKeyFrames)
        {
            p_keyFrame->UpdateMap(p_currentMap_in);
            p_currentMap_in->AddKeyFrame(p_keyFrame);
            p_otherMap_in->EraseKeyFrame(p_keyFrame);
        }

        for (MapPoint *p_mapPoint : importedMapPoints)
        {
            p_mapPoint->UpdateMap(p_currentMap_in);
            p_currentMap_in->AddMapPoint(p_mapPoint);
            p_otherMap_in->EraseMapPoint(p_mapPoint);
        }

        for (const auto &[p_plane, assignedId] : planeIdAssignments)
        {
            p_plane->setId(assignedId);
            p_plane->SetMap(p_currentMap_in);
            p_currentMap_in->AddMapPlane(p_plane);
            p_otherMap_in->EraseRoomWallPlane(p_plane);
            p_otherMap_in->EraseMapPlane(p_plane);
        }

        std::unordered_map<int, int> importedMarkerIdRemap;
        for (const auto &[p_marker, assignedId] : markerIdAssignments)
        {
            importedMarkerIdRemap.insert_or_assign(p_marker->getId(),
                                                   assignedId);
            p_marker->setId(assignedId);
            p_marker->setMap(p_currentMap_in);
            p_currentMap_in->AddMapMarker(p_marker);
            p_otherMap_in->EraseMapMarker(p_marker);
        }

        for (const auto &[p_passage, assignedId] : passageIdAssignments)
        {
            p_passage->setId(assignedId);
            p_passage->setMap(p_currentMap_in);
            p_currentMap_in->AddMapPassage(p_passage);
            p_otherMap_in->EraseMapPassage(p_passage);
        }

        for (Room *p_room : importedRooms)
        {
            Marker *p_metaMarker = p_room->getMetaMarker();
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
                p_currentMap_in->AddDetectedMapRoom(p_room);
            }
            else
            {
                p_currentMap_in->AddCandidateMapRoom(p_room);
            }
            p_otherMap_in->EraseDetectedMapRoom(p_room);
            p_otherMap_in->EraseMarkerBasedMapRoom(p_room);
        }

        for (const auto &[p_floor, assignedId] : floorIdAssignments)
        {
            p_floor->setId(assignedId);
            p_floor->setMap(p_currentMap_in);
            p_currentMap_in->AddMapFloor(p_floor);
            p_otherMap_in->EraseMapFloor(p_floor);
        }

        std::vector<MapPoint *> mergedReferenceMapPoints =
            p_currentMap_in->GetReferenceMapPoints();
        for (MapPoint *p_mapPoint : importedReferenceMapPoints)
        {
            if (p_mapPoint != nullptr && p_mapPoint->GetMap() == p_currentMap_in &&
                std::find(mergedReferenceMapPoints.begin(),
                          mergedReferenceMapPoints.end(),
                          p_mapPoint) == mergedReferenceMapPoints.end())
            {
                mergedReferenceMapPoints.push_back(p_mapPoint);
            }
        }
        p_currentMap_in->SetReferenceMapPoints(mergedReferenceMapPoints);
        p_otherMap_in->SetReferenceMapPoints({});

        for (KeyFrame *p_originKeyFrame : importedKeyFrameOrigins)
        {
            if (p_originKeyFrame != nullptr &&
                p_originKeyFrame->GetMap() == p_currentMap_in &&
                std::find(p_currentMap_in->mvpKeyFrameOrigins.begin(),
                          p_currentMap_in->mvpKeyFrameOrigins.end(),
                          p_originKeyFrame) ==
                    p_currentMap_in->mvpKeyFrameOrigins.end())
            {
                p_currentMap_in->mvpKeyFrameOrigins.push_back(p_originKeyFrame);
            }
        }
        p_otherMap_in->mvpKeyFrameOrigins.clear();

        if (p_currentMap_in->mpFirstRegionKF == nullptr &&
            p_importedFirstRegionKeyFrame != nullptr &&
            p_importedFirstRegionKeyFrame->GetMap() == p_currentMap_in)
        {
            p_currentMap_in->mpFirstRegionKF = p_importedFirstRegionKeyFrame;
        }
        p_otherMap_in->mpFirstRegionKF = nullptr;

        p_currentMap_in->SetSkeletonClusterPoints({});
        p_currentMap_in->SetSkeletonEdges({});
        p_otherMap_in->SetSkeletonClusterPoints({});
        p_otherMap_in->SetSkeletonEdges({});
        p_otherMap_in->ClearTransferredEntityIndexes();

    /* Fuse duplicate floors: keep only one floor per map (system supports
     * single-floor semantics). Reassign rooms from duplicate floors to the
     * primary floor and erase the extras. */
        std::vector<Floor *> allFloors = p_currentMap_in->GetAllFloors();
        if (allFloors.size() > 1)
        {
            Floor *p_keeperFloor = Floor::selectBestObservedFloor(allFloors);
            for (Floor *p_duplicateFloor : allFloors)
            {
                if (p_duplicateFloor == nullptr ||
                    p_duplicateFloor == p_keeperFloor)
                {
                    continue;
                }

                for (Room *p_room : p_duplicateFloor->getRooms())
                {
                    if (p_room != nullptr && !p_room->isBad())
                    {
                        p_keeperFloor->addRoom(p_room);
                    }
                }

                p_currentMap_in->EraseMapFloor(p_duplicateFloor);
                std::cout << "[Atlas::MergeMapPair] Fused duplicate Floor#"
                          << p_duplicateFloor->getId() << " into Floor#"
                          << p_keeperFloor->getId()
                          << " and retained the better-observed plane identity."
                          << std::endl;
            }
        }

        Utils::fuseDuplicateRoomsAfterMerge(p_currentMap_in, importedRooms);

        Floor *p_mergedFloor = Floor::selectBestObservedFloor(
            p_currentMap_in->GetAllFloors());
        if (p_mergedFloor != nullptr)
        {
            for (Room *p_room : p_currentMap_in->GetAllDetectedMapRooms())
            {
                if (p_room != nullptr && !p_room->isBad())
                {
                    p_mergedFloor->addRoom(p_room);
                }
            }
        }

        for (Room *p_room : p_currentMap_in->GetAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (Plane *p_wall : p_room->getWalls())
            {
                if (p_wall != nullptr && !p_wall->isBad())
                {
                    p_currentMap_in->AddRoomWallPlane(p_wall);
                }
            }
        }

        Utils::reAssociatePassages(this);
    }

    /* Retire the absorbed map while keeping the current map active. */
    SetMapBad(p_otherMap_in);
    ChangeMap(p_currentMap_in);

    std::cout << "[Atlas::MergeMapPair] Merged map " << p_otherMap_in->GetId()
              << " into map " << p_currentMap_in->GetId() << " fused "
              << importedRooms.size() << " rooms into current map."
              << std::endl;
    std::cout << "[FloorVerify] Map#" << p_currentMap_in->GetId()
              << " and Map#" << p_otherMap_in->GetId()
              << " result=ACCEPTED committed=1" << std::endl;

    /* Notify downstream consumers that the current map changed. */
    p_currentMap_in->IncreaseChangeIndex();
}

bool Atlas::isInertial()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

void Atlas::PreSave()
{
    if (mpCurrentMap)
    {
        if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid() +
                                1; // The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map *elem1, Map *elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(),
              mspMaps.end(),
              std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera *> spCams(mvpCameras.begin(), mvpCameras.end());
    for (Map *pMi : mvpBackupMaps)
    {
        if (!pMi || pMi->IsBad())
            continue;

        if (pMi->GetAllKeyFrames().size() == 0)
        {
            // Empty map, erase before of save it.
            SetMapBad(pMi);
            continue;
        }
        pMi->PreSave(spCams);
    }
    RemoveBadMaps();
}

void Atlas::PostLoad()
{
    map<unsigned int, GeometricCamera *> mpCams;
    for (GeometricCamera *pCam : mvpCameras)
    {
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    for (Map *pMi : mvpBackupMaps)
    {
        mspMaps.insert(pMi);
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }
    mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase *pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase *Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary *pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary *Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int  num = 0;
    for (Map *pMap_i : mspMaps)
    {
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP()
{
    unique_lock<mutex> lock(mMutexAtlas);
    long unsigned int  num = 0;
    for (Map *pMap_i : mspMaps)
    {
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}

map<long unsigned int, KeyFrame *> Atlas::GetAtlasKeyframes()
{
    map<long unsigned int, KeyFrame *> mpIdKFs;
    for (Map *pMap_i : mvpBackupMaps)
    {
        vector<KeyFrame *> vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for (KeyFrame *pKF_j_Mi : vpKFs_Mi)
        {
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

void Atlas::exportRoomContextFromCurrentMap()
{
    std::unique_lock<std::mutex> lock(mRoomContextMutex);

    if (!mpCurrentMap)
        return;

    /* Export BOTH confirmed detected rooms AND candidate/marker-based rooms.
     * Candidate rooms (prospective/provisional) may not have full wall loops yet
     * but still carry spatial identity needed for cross-restart matching. */
    std::vector<Room *> rooms = mpCurrentMap->GetAllDetectedMapRooms();
    std::vector<Room *> candidateRooms = mpCurrentMap->GetAllCandidateMapRooms();
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());

    if (rooms.empty())
        return;

    std::vector<RoomContextSnapshot> snapshots;
    snapshots.reserve(rooms.size());

    const long unsigned int mapId = mpCurrentMap->GetId();

    for (Room *room : rooms)
    {
        if (!room || room->isBad())
            continue;

        RoomContextSnapshot snap;
        snap.roomId   = room->getId();
        snap.centroid = room->getCentroid();
        snap.timestamp =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();

        for (Plane *wall : room->getWalls())
        {
            if (!wall || wall->isBad())
                continue;

            std::optional<Eigen::Vector3d> orientedNormal =
                room->getWallNormalTowardRoom_World(wall);
            if (orientedNormal)
                snap.wallNormals.push_back(*orientedNormal);

            snap.wallCentroids.push_back(wall->getCentroid());
            snap.wallDistances.push_back(wall->getGlobalEquation().distance());
        }

        for (Passage *passage : room->getPassages())
        {
            if (!passage)
                continue;
            snap.passageCentroids.push_back(passage->getCentroid());
        }

        /* Assign persistent tag to old map rooms for merge trigger.
         * Tag format: "room_<id>" matches what matchRoomsToContext() assigns. */
        const std::string roomTag = "room_" + std::to_string(room->getId());
        snap.roomTag = roomTag;
        if (!room->hasRoomTag())
        {
            room->setRoomTag(roomTag);
        }

        snapshots.push_back(snap);
    }

    const std::size_t exportedRoomCount = snapshots.size();
    mRoomContextHistory[mapId]          = std::move(snapshots);

    std::cout << "[Atlas] Exported room context: " << exportedRoomCount
              << " rooms (mapId: " << mapId << ")" << std::endl;
}

void Atlas::matchRoomsToContext(Map *pNewMap)
{
    if (!pNewMap)
        return;

    std::unique_lock<std::mutex> lock(mRoomContextMutex);

    if (mRoomContextHistory.empty())
        return;

    /* Match BOTH detected rooms AND candidate/prospective rooms.
     * Candidate rooms need identity tags for cross-restart continuity. */
    std::vector<Room *> newRooms = pNewMap->GetAllDetectedMapRooms();
    std::vector<Room *> candidateRooms = pNewMap->GetAllCandidateMapRooms();
    newRooms.insert(newRooms.end(), candidateRooms.begin(), candidateRooms.end());

    if (newRooms.empty())
        return;

    std::vector<RoomContextSnapshot> allContext;
    for (const auto &entry : mRoomContextHistory)
        for (const auto &snap : entry.second)
            allContext.push_back(snap);

    if (allContext.empty())
        return;

    for (Room *room : newRooms)
    {
        if (!room || room->isBad())
            continue;

        if (room->hasRoomTag())
            continue;

        Eigen::Vector3d roomCentroid = room->getCentroid();
        double          bestDist     = std::numeric_limits<double>::max();
        const RoomContextSnapshot *bestMatch = nullptr;

        for (const RoomContextSnapshot &snap : allContext)
        {
            /* PREFER tag-based matching if snapshot has persistent tag.
             * This provides deterministic identity across restarts. */
            if (!snap.roomTag.empty() && room->hasRoomTag())
            {
                if (room->getRoomTag() == snap.roomTag)
                {
                    bestMatch = &snap;
                    bestDist  = 0.0;
                    break;
                }
            }

            double dist = (snap.centroid - roomCentroid).norm();

            if (dist > kRoomContextMatchThreshold_m)
                continue;

            /* Verify wall-normal agreement: compare the first available
             * wall normal of the new room against each snapshot wall
             * normal. Accept when |cosθ| > kWallNormalAlignmentCosTheta. */
            std::vector<Plane *> roomWalls = room->getWalls();
            if (roomWalls.empty() || snap.wallNormals.empty())
            {
                /* Fallback: accept on centroid distance alone when no wall
                 * normals are available for normal validation. */
                if (dist < bestDist)
                {
                    bestDist  = dist;
                    bestMatch = &snap;
                }
                continue;
            }

            std::optional<Eigen::Vector3d> newRoomNormal =
                room->getWallNormalTowardRoom_World(roomWalls[0]);
            if (!newRoomNormal)
                continue;

            bool normalOk = false;
            for (const Eigen::Vector3d &snapNormal : snap.wallNormals)
            {
                double dot = newRoomNormal->dot(snapNormal);
                if (std::abs(dot) > kWallNormalAlignmentCosTheta)
                {
                    normalOk = true;
                    break;
                }
            }

            if (!normalOk)
                continue;

            if (dist < bestDist)
            {
                bestDist  = dist;
                bestMatch = &snap;
            }
        }

        if (bestMatch)
        {
            room->setRoomTag("room_" + std::to_string(bestMatch->roomId));

            /* Locate the snapshot pointer in the stored history so the
             * room can hold a non-owning reference for WP3. */
            for (auto &entry : mRoomContextHistory)
            {
                for (auto &storedSnap : entry.second)
                {
                    if (storedSnap.roomId == bestMatch->roomId &&
                        storedSnap.centroid.isApprox(bestMatch->centroid))
                    {
                        room->setMatchedContext(&storedSnap);
                        break;
                    }
                }
            }

            /* ----------------------------------------------------------- *
             * CONTINUITY: Transfer walls/passages from prior room instance.
             * The matched prior room (in stored map) already has accumulated
             * boundary walls. Re-associate them to this new room instance
             * so boundary validation continues from where it left off.
             * ----------------------------------------------------------- */

            /* Find the stored map that contains the prior room. */
            Map *p_priorMap = nullptr;
            Room *p_priorRoom = nullptr;
            const auto allMaps = GetAllMaps();
            for (Map *p_map : allMaps)
            {
                if (!p_map || p_map->IsBad() || p_map == mpCurrentMap)
                    continue;
                for (Room *r : p_map->GetAllDetectedMapRooms())
                {
                    if (r && !r->isBad() && r->getId() == bestMatch->roomId)
                    {
                        p_priorRoom = r;
                        p_priorMap = p_map;
                        break;
                    }
                }
                if (p_priorRoom) break;
            }

            if (p_priorRoom && p_priorMap)
            {
                std::cout << "[Atlas] Prior Room#" << p_priorRoom->getId()
                          << " has " << p_priorRoom->getWalls().size() << " walls" << std::endl;

                /* Transfer walls from prior room to current room */
                for (Plane *p_wall : p_priorRoom->getWalls())
                {
                    if (!p_wall || p_wall->isBad())
                        continue;

                    /* Re-associate wall to new room */
                    p_priorRoom->removeWall(p_wall);
                    room->setWalls(p_wall);

                    std::cout << "[Atlas] Transferred Wall#" << p_wall->getId()
                              << " from prior Room#" << p_priorRoom->getId()
                              << " to matched Room#" << room->getId() << std::endl;
                }

                /* Passages will be re-associated by associatePassagesToRooms() */

                std::cout << "[Atlas] Room#" << room->getId()
                          << " now has " << room->getWalls().size()
                          << " walls (continuing from prior Room#" << bestMatch->roomId << ")"
                          << std::endl;
            }
            else
            {
                std::cout << "[Atlas] NO prior room found for match (p_priorRoom=" << p_priorRoom
                          << ", p_priorMap=" << p_priorMap << ")" << std::endl;
            }

            std::cout << "[Atlas] Matched room " << room->getId()
                      << " in new map, tagged with identity \""
                      << room->getRoomTag() << "\" (prior room "
                      << bestMatch->roomId << ", dist=" << bestDist << " m)"
                      << std::endl;
        }
    }
}

const std::vector<RoomContextSnapshot> &
    Atlas::getRoomContextForMap(long unsigned int mapId) const
{
    static const std::vector<RoomContextSnapshot> empty;
    auto it = mRoomContextHistory.find(mapId);
    if (it == mRoomContextHistory.end())
        return empty;
    return it->second;
}

} // namespace ORB_SLAM3
