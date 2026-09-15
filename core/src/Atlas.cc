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
#include "Semantic/SemanticVerify.h"
#include "Types/SystemParams.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <set>
#include <sophus/se3.hpp>
#include <unordered_map>

#if defined(VS_GRAPHS_ENABLE_ATLAS_LOCK_ORDER_TEST_HOOK)
extern "C" void vsGraphsAtlasLockOrderBeforeMapSnapshot() __attribute__((weak));
#endif

namespace vs_graphs
{
namespace core
{
namespace
{
void advanceIdentityAllocator(std::atomic<int> &nextIdentity_inout,
                              const int         observedIdentity_in)
{
    if (observedIdentity_in < 0)
    {
        return;
    }

    int expectedNextIdentity =
        nextIdentity_inout.load(std::memory_order_relaxed);
    const int requiredNextIdentity = observedIdentity_in + 1;
    while (expectedNextIdentity < requiredNextIdentity &&
           !nextIdentity_inout.compare_exchange_weak(expectedNextIdentity,
                                                     requiredNextIdentity,
                                                     std::memory_order_relaxed,
                                                     std::memory_order_relaxed))
    {}
}

template <typename Entity>
bool planImportedIds(const std::vector<Entity *>           &existingEntities_in,
                     const std::vector<Entity *>           &importedEntities_in,
                     const char                            *entityName_in,
                     std::vector<std::pair<Entity *, int>> &assignments_out)
{
    const std::set<Entity *> importedEntities(importedEntities_in.begin(),
                                              importedEntities_in.end());
    std::set<int>            destinationIds;

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

static std::size_t countLiveRooms(Map *p_map_in)
{
    if (p_map_in == nullptr)
    {
        return 0U;
    }
    std::size_t liveCount = 0U;
    for (semantic::Room *p_room : p_map_in->GetAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            ++liveCount;
        }
    }
    return liveCount;
}

static std::size_t countLiveWallPlanes(Map *p_map_in)
{
    if (p_map_in == nullptr)
    {
        return 0U;
    }
    std::size_t liveCount = 0U;
    for (geometric::Plane *p_plane : p_map_in->GetAllPlanes())
    {
        if (p_plane != nullptr && !p_plane->isBad() &&
            p_plane->getPlaneType() == geometric::Plane::planeVariant::WALL)
        {
            ++liveCount;
        }
    }
    return liveCount;
}

static std::size_t countLivePassages(Map *p_map_in)
{
    if (p_map_in == nullptr)
    {
        return 0U;
    }
    std::size_t liveCount = 0U;
    for (semantic::Passage *p_passage : p_map_in->GetAllPassages())
    {
        if (p_passage != nullptr && !p_passage->isBad())
        {
            ++liveCount;
        }
    }
    return liveCount;
}

static std::size_t countLiveFloors(Map *p_map_in)
{
    if (p_map_in == nullptr)
    {
        return 0U;
    }
    std::size_t liveCount = 0U;
    for (semantic::Floor *p_floor : p_map_in->GetAllFloors())
    {
        if (p_floor != nullptr && p_floor->hasPlaneIdentity())
        {
            ++liveCount;
        }
    }
    return liveCount;
}

static std::size_t consecutiveContentHash(Map *p_oldMap_in,
                                          Map *p_currentMap_in)
{
    std::size_t contentHash = countLiveRooms(p_oldMap_in);
    contentHash = contentHash * 31U + countLiveWallPlanes(p_oldMap_in);
    contentHash = contentHash * 31U + countLivePassages(p_oldMap_in);
    contentHash = contentHash * 31U + countLiveFloors(p_oldMap_in);
    contentHash = contentHash * 31U + countLiveRooms(p_currentMap_in);
    contentHash = contentHash * 31U + countLiveWallPlanes(p_currentMap_in);
    contentHash = contentHash * 31U + countLivePassages(p_currentMap_in);
    contentHash = contentHash * 31U + countLiveFloors(p_currentMap_in);
    return contentHash;
}

/*! @brief Room-prior seed: the old final room and the new starting room
 * must carry the same non-empty tag. A silent mismatch means no prior link
 * (e.g. loop closure between non-consecutive maps): not this path's job. */
static bool consecutiveSeedTagsMatch(Map *p_oldMap_in, Map *p_currentMap_in)
{
    if (p_oldMap_in == nullptr || p_currentMap_in == nullptr)
    {
        return false;
    }
    semantic::Room *p_oldFinalRoom = p_oldMap_in->getFinalRoom();
    semantic::Room *p_newStartRoom = p_currentMap_in->getStartingRoom();
    return p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
           p_oldFinalRoom->hasRoomTag() && p_newStartRoom->hasRoomTag() &&
           !p_oldFinalRoom->getRoomTag().empty() &&
           p_oldFinalRoom->getRoomTag() == p_newStartRoom->getRoomTag();
}

static std::set<std::string> collectAnchorTags(Map *p_oldMap_in,
                                               Map *p_currentMap_in)
{
    std::set<std::string> anchorTags;
    if (p_oldMap_in == nullptr || p_currentMap_in == nullptr)
    {
        return anchorTags;
    }
    std::set<std::string> currentTags;
    for (semantic::Room *p_room : p_currentMap_in->GetAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() && p_room->hasRoomTag() &&
            !p_room->getRoomTag().empty())
        {
            currentTags.insert(p_room->getRoomTag());
        }
    }
    for (semantic::Room *p_room : p_oldMap_in->GetAllRooms())
    {
        if (p_room != nullptr && !p_room->isBad() && p_room->hasRoomTag() &&
            !p_room->getRoomTag().empty() &&
            currentTags.count(p_room->getRoomTag()) > 0U)
        {
            anchorTags.insert(p_room->getRoomTag());
        }
    }
    return anchorTags;
}

/*! @brief Folds a transferred passage into the same-lineage proxy.
 *
 * The proxy keeps its stable current-map ID and live room links and adopts
 * the transferred (already in-frame) geometry, supporting walls, door,
 * known-side direction and traversal history. Traversal windows are
 * disjoint (pre- vs post-reset), so counts add. Returns true when the
 * transferred object must NOT enter the current map (it retires with the
 * absorbed map instead). */
static bool resurfaceProxyFromTransferred(semantic::Passage *p_proxy_inout,
                                          semantic::Passage *p_transferred_in)
{
    if (p_proxy_inout == nullptr || p_transferred_in == nullptr ||
        p_proxy_inout == p_transferred_in ||
        !p_proxy_inout->isRecoveryProxy() || p_proxy_inout->isBad() ||
        p_transferred_in->isBad())
    {
        return false;
    }

    const Eigen::Vector3d transferredCentroid = p_transferred_in->getCentroid();
    const Eigen::Vector4d transferredCoefficients =
        p_transferred_in->getGlobalEquation().coeffs();
    const double transferredNormalNorm =
        transferredCoefficients.head<3>().norm();
    const double transferredWidth_m  = p_transferred_in->getWidth();
    const double transferredHeight_m = p_transferred_in->getHeight();
    if (transferredCentroid.allFinite() &&
        transferredCoefficients.allFinite() && transferredNormalNorm > 1e-8 &&
        std::isfinite(transferredWidth_m) && transferredWidth_m > 0.0 &&
        std::isfinite(transferredHeight_m) && transferredHeight_m > 0.0)
    {
        p_proxy_inout->setCentroid(transferredCentroid);
        p_proxy_inout->setGlobalEquation(p_transferred_in->getGlobalEquation());
        p_proxy_inout->setWidth(transferredWidth_m);
        p_proxy_inout->setHeight(transferredHeight_m);
        p_proxy_inout->setRecoveryProxy(false);
    }
    for (geometric::Plane *p_wall : p_transferred_in->getAssociateWalls())
    {
        if (p_wall != nullptr)
        {
            p_proxy_inout->addAssociateWall(p_wall);
        }
    }
    if (p_proxy_inout->getAssociateDoor() == nullptr &&
        p_transferred_in->getAssociateDoor() != nullptr)
    {
        p_proxy_inout->setAssociateDoor(p_transferred_in->getAssociateDoor());
    }
    const semantic::Passage::KnownSideProvenance transferredSide =
        p_transferred_in->getKnownSideProvenance();
    if (!p_proxy_inout->getKnownSideProvenance().hasDirection() &&
        transferredSide.hasDirection())
    {
        p_proxy_inout->setKnownSideDirection(transferredSide.direction_World);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < p_transferred_in->getTraversalKnownToFarCount();
         ++observationIndex)
    {
        p_proxy_inout->addTraversalObservation(
            semantic::Passage::TraversalDirection::KNOWN_TO_FAR);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < p_transferred_in->getTraversalFarToKnownCount();
         ++observationIndex)
    {
        p_proxy_inout->addTraversalObservation(
            semantic::Passage::TraversalDirection::FAR_TO_KNOWN);
    }
    for (std::size_t observationIndex = 0U;
         observationIndex < p_transferred_in->getTraversalUnknownCount();
         ++observationIndex)
    {
        p_proxy_inout->addTraversalObservation(
            semantic::Passage::TraversalDirection::UNKNOWN);
    }
    if (p_transferred_in->isPassable())
    {
        p_proxy_inout->setPassable(true);
    }
    std::cout << "SG_PIPELINE {\"event\":\"passage_resurfaced\","
                 "\"map_id\":"
              << p_proxy_inout->getMap()->GetId()
              << ",\"passage_id\":" << p_proxy_inout->getId() << "}"
              << std::endl;
    return true;
}
} // namespace

Atlas::Atlas()
{
    mpCurrentMap = static_cast<Map *>(nullptr);
}

Atlas::Atlas(int initKFid) :
    mnLastInitKFidMap(initKFid),
    mHasViewer(false)
{
    mpCurrentMap = static_cast<Map *>(nullptr);
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

    Map *p_previousMap = mpCurrentMap;
    mpCurrentMap       = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
    if (p_previousMap != nullptr)
    {
        /* Mission-chain trace link: the stranded map points at its
         * successor. Same-map clears never pass through here, so the link
         * stays null for them by construction. */
        p_previousMap->setFollowingMap(mpCurrentMap);
    }
    {
        std::lock_guard<std::mutex> contextLock(mRoomContextMutex);
        newMapCreatedPending_ = true;
    }
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

void Atlas::AddMapMarker(semantic::Marker *marker)
{
    Map *pMapMP = marker->getMap();
    pMapMP->AddMapMarker(marker);
}

void Atlas::AddMapPlane(vs_graphs::core::geometric::Plane *plane)
{
    vs_graphs::core::Map *pMapMP = plane->GetMap();
    pMapMP->AddMapPlane(plane);
}

void Atlas::AddRoomWallPlane(vs_graphs::core::geometric::Plane *pPlane)
{
    vs_graphs::core::Map *pMapMP = pPlane->GetMap();
    pMapMP->AddRoomWallPlane(pPlane);
}

void Atlas::AddMapPassage(vs_graphs::core::semantic::Passage *passage)
{
    if (passage == nullptr)
    {
        return;
    }
    observePassageIdentity(passage->getId());
    vs_graphs::core::Map *pMapMP = passage->getMap();
    pMapMP->AddMapPassage(passage);
}

void Atlas::AddDetectedMapRoom(semantic::Room *room)
{
    if (room == nullptr)
    {
        return;
    }
    observeRoomIdentity(room->getId());
    Map *pMapMP = room->getMap();
    pMapMP->AddDetectedMapRoom(room);
}

void Atlas::AddCandidateMapRoom(semantic::Room *room)
{
    if (room == nullptr)
    {
        return;
    }
    observeRoomIdentity(room->getId());
    Map *pMapMP = room->getMap();
    pMapMP->AddCandidateMapRoom(room);
}

void Atlas::AddMapFloor(semantic::Floor *floor)
{
    if (floor == nullptr)
    {
        return;
    }
    observeFloorIdentity(floor->getId());
    Map *pMapMP = floor->getMap();
    pMapMP->AddMapFloor(floor);
}

int Atlas::reserveRoomIdentity(void)
{
    return nextRoomIdentity_.fetch_add(1, std::memory_order_relaxed);
}

int Atlas::reservePassageIdentity(void)
{
    return nextPassageIdentity_.fetch_add(1, std::memory_order_relaxed);
}

int Atlas::reserveFloorIdentity(void)
{
    return nextFloorIdentity_.fetch_add(1, std::memory_order_relaxed);
}

void Atlas::observeRoomIdentity(const int roomId_in)
{
    advanceIdentityAllocator(nextRoomIdentity_, roomId_in);
}

void Atlas::observePassageIdentity(const int passageId_in)
{
    advanceIdentityAllocator(nextPassageIdentity_, passageId_in);
}

void Atlas::observeFloorIdentity(const int floorId_in)
{
    advanceIdentityAllocator(nextFloorIdentity_, floorId_in);
}

void Atlas::setCurrentSemanticRoomIdentity(const int roomId_in)
{
    currentSemanticRoomIdentity_.store(roomId_in, std::memory_order_release);
    observeRoomIdentity(roomId_in);
}

int Atlas::getCurrentSemanticRoomIdentity(void) const
{
    return currentSemanticRoomIdentity_.load(std::memory_order_acquire);
}

camera_models::GeometricCamera *Atlas::AddCamera(camera_models::GeometricCamera *pCam)
{
    // Check if the camera already exists
    bool bAlreadyInMap = false;
    int  index_cam     = -1;
    for (size_t i = 0; i < mvpCameras.size(); ++i)
    {
        camera_models::GeometricCamera *pCam_i = mvpCameras[i];
        if (!pCam)
            std::cout << "Not pCam" << std::endl;
        if (!pCam_i)
            std::cout << "Not pCam_i" << std::endl;
        if (pCam->getType() != pCam_i->getType())
            continue;

        if (pCam->getType() == camera_models::GeometricCamera::CAM_PINHOLE)
        {
            if (((camera_models::Pinhole *)pCam_i)->isEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam     = i;
            }
        }
        else if (pCam->getType() == camera_models::GeometricCamera::CAM_FISHEYE)
        {
            if (((camera_models::KannalaBrandt8 *)pCam_i)->isEqual(pCam))
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

std::vector<camera_models::GeometricCamera *> Atlas::GetAllCameras()
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

vs_graphs::core::semantic::Passage *Atlas::GetPassageById(int passageId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetPassageById(passageId)
                                   : nullptr;
}

semantic::Floor *Atlas::GetFloorById(int floorId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetFloorById(floorId)
                                   : nullptr;
}

geometric::Plane *Atlas::GetPlaneById(int planeId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetPlaneById(planeId)
                                   : nullptr;
}

vs_graphs::core::geometric::Plane *Atlas::GetRoomWallPlaneById(int planeId)
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap != nullptr ? mpCurrentMap->GetRoomWallPlaneById(planeId)
                                   : nullptr;
}

semantic::Marker *Atlas::GetMarkerById(int markerId)
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

std::vector<semantic::Marker *> Atlas::GetAllMarkers()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMarkers();
}

std::vector<vs_graphs::core::geometric::Plane *> Atlas::GetAllPlanes()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllPlanes();
}

std::vector<vs_graphs::core::semantic::Passage *> Atlas::GetAllPassages()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllPassages();
}

std::vector<semantic::Room *> Atlas::GetAllRooms()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllRooms();
}

std::vector<semantic::Room *> Atlas::GetAllDetectedMapRooms()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllDetectedMapRooms();
}

std::vector<semantic::Room *> Atlas::GetAllMarkerBasedMapRooms()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMarkerBasedMapRooms();
}

std::vector<semantic::Room *> Atlas::GetAllCandidateMapRooms()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllCandidateMapRooms();
}

std::vector<semantic::Floor *> Atlas::GetAllFloors()
{
    unique_lock<mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllFloors();
}

geometric::Plane *Atlas::GetBiggestGroundPlane()
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
    /* Same-map reset (Tracking::ResetActiveMap) wipes rooms/floors/passages
     * from the live Map object without creating a new Map. Snapshot first so
     * the bootstrap recovery path can recreate the same stable identities
     * afterwards; otherwise the next cycle allocates fresh RoomN/FloorM. */
    exportRoomContextFromCurrentMap();
    mpCurrentMap->clear();
    /* A same-map clear keeps the map id, so the visualization/voxblox
     * revision token would not observe the reset and stale markers and
     * clouds would persist alongside the fresh map. A clear is at least as
     * big a change as the loop-closure corrections this index exists for. */
    mpCurrentMap->InformNewBigChange();
}

void Atlas::clearAtlas()
{
    unique_lock<mutex> lock(mMutexAtlas);
    mspMaps.clear();
    mpCurrentMap      = static_cast<Map *>(nullptr);
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
    semantic::Floor *p_currentFloor =
        semantic::Floor::selectBestObservedFloor(p_currentMap_in->GetAllFloors());
    semantic::Floor *p_otherFloor =
        semantic::Floor::selectBestObservedFloor(p_otherMap_in->GetAllFloors());

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
            transformedOtherFloorIdentity = semantic::Floor::transformPlaneIdentity(
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
                      << p_currentMap_in->GetId() << " and Map#"
                      << p_otherMap_in->GetId()
                      << " floor planes mismatch (angle="
                      << floorNormalAngle_deg
                      << " deg, offset=" << floorOffset_m
                      << " m; limits=" << semantic::Floor::kMergeMaxPlaneNormalAngle_deg
                      << " deg/" << semantic::Floor::kMergeMaxPlaneOffset_m
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
                  << "); result=DEFERRED committed=0" << std::endl;
        return;
    }

    /* Snapshot every source-owned object and preflight destination indexes
     * before changing geometry, ownership, or any externally visible ID. */
    std::vector<KeyFrame *> importedKeyFrames =
        p_otherMap_in->GetAllKeyFrames();
    std::vector<MapPoint *> importedMapPoints =
        p_otherMap_in->GetAllMapPoints();
    std::vector<semantic::Room *> importedDetectedRooms =
        p_otherMap_in->GetAllDetectedMapRooms();
    std::vector<semantic::Room *> importedMarkerRooms =
        p_otherMap_in->GetAllMarkerBasedMapRooms();
    std::vector<geometric::Plane *> importedPlanes = p_otherMap_in->GetAllPlanes();
    std::vector<vs_graphs::core::semantic::Passage *> importedPassages =
        p_otherMap_in->GetAllPassages();
    std::vector<vs_graphs::core::semantic::Floor *> importedFloors =
        p_otherMap_in->GetAllFloors();
    std::vector<semantic::Marker *> importedMarkers = p_otherMap_in->GetAllMarkers();

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
        p_currentMap_in->GetAllKeyFrames();
    const std::set<KeyFrame *> destinationKeyFrameSet(
        destinationKeyFrames.begin(),
        destinationKeyFrames.end());
    const std::vector<MapPoint *> destinationMapPoints =
        p_currentMap_in->GetAllMapPoints();
    const std::set<MapPoint *> destinationMapPointSet(
        destinationMapPoints.begin(),
        destinationMapPoints.end());

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
            !ownerIsTransferable(p_mapPoint->GetMap()) ||
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
        if (p_plane == nullptr || !ownerIsTransferable(p_plane->GetMap()))
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

    std::vector<std::pair<geometric::Plane *, int>>   planeIdAssignments;
    std::vector<std::pair<semantic::Marker *, int>>  markerIdAssignments;
    std::vector<std::pair<semantic::Passage *, int>> passageIdAssignments;
    std::vector<std::pair<semantic::Room *, int>>    roomIdAssignments;
    std::vector<std::pair<semantic::Floor *, int>>   floorIdAssignments;

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
    KeyFrame *p_importedFirstRegionKeyFrame = p_otherMap_in->mpFirstRegionKF;

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
            /* Same stable lineage as a current-map recovery proxy: fold the
             * transferred (now in-frame) state into the proxy instead of
             * duplicating the doorway. The transferred object retires with
             * the absorbed map; it never enters the current map. */
            semantic::Passage *p_proxy =
                p_currentMap_in->GetPassageById(p_passage->getId());
            if (p_proxy != nullptr &&
                resurfaceProxyFromTransferred(p_proxy, p_passage))
            {
                p_passage->setBad();
                continue;
            }
            p_passage->setId(assignedId);
            p_passage->setMap(p_currentMap_in);
            p_currentMap_in->AddMapPassage(p_passage);
            p_otherMap_in->EraseMapPassage(p_passage);
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
            if (p_mapPoint != nullptr &&
                p_mapPoint->GetMap() == p_currentMap_in &&
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
        std::vector<semantic::Floor *> allFloors = p_currentMap_in->GetAllFloors();
        if (allFloors.size() > 1)
        {
            semantic::Floor *p_keeperFloor = semantic::Floor::selectBestObservedFloor(allFloors);
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

                p_currentMap_in->EraseMapFloor(p_duplicateFloor);
                std::cout << "[Atlas::MergeMapPair] Fused duplicate semantic::Floor#"
                          << p_duplicateFloor->getId() << " into semantic::Floor#"
                          << p_keeperFloor->getId()
                          << " and retained the better-observed plane identity."
                          << std::endl;
            }
        }

        Utils::fuseDuplicateRoomsAfterMerge(p_currentMap_in, importedRooms);

        semantic::Floor *p_mergedFloor =
            semantic::Floor::selectBestObservedFloor(p_currentMap_in->GetAllFloors());
        if (p_mergedFloor != nullptr)
        {
            for (semantic::Room *p_room : p_currentMap_in->GetAllDetectedMapRooms())
            {
                if (p_room != nullptr && !p_room->isBad())
                {
                    p_mergedFloor->addRoom(p_room);
                }
            }
        }

        for (semantic::Room *p_room : p_currentMap_in->GetAllRooms())
        {
            if (p_room == nullptr || p_room->isBad())
            {
                continue;
            }

            for (geometric::Plane *p_wall : p_room->getWalls())
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
    std::cout << "[FloorVerify] Map#" << p_currentMap_in->GetId() << " and Map#"
              << p_otherMap_in->GetId() << " result=ACCEPTED committed=1"
              << std::endl;

    /* Notify downstream consumers that the current map changed. */
    p_currentMap_in->IncreaseChangeIndex();
}

void Atlas::attemptConsecutiveMergeIfGated(void)
{
    /* LOCK ORDER: the caller holds mMutexSemanticUpdate for the whole call.
     * This method briefly takes mMutexAtlas for attempt-state bookkeeping,
     * and MergeMapPair takes both maps' mMutexMapUpdate. No path in the
     * codebase acquires these in reverse, so the order
     * semantic-update -> atlas -> map-update is deadlock-free. */
    Map *p_currentMap = GetCurrentMap();
    if (p_currentMap == nullptr || p_currentMap->IsBad())
    {
        return;
    }

    types::SystemParams      *p_params = types::SystemParams::getParams();
    const unsigned int cooldown_s =
        p_params != nullptr ? p_params->mapMerge.mergeCooldown_s : 30U;
    const unsigned int minimumAnchors =
        p_params != nullptr ? p_params->mapMerge.minAnchorRooms : 2U;
    const unsigned int minimumRooms =
        p_params != nullptr ? p_params->mapMerge.minRoomsPerMap : 1U;
    const unsigned int minimumWalls =
        p_params != nullptr ? p_params->mapMerge.minWallsPerMap : 3U;
    const semantic::SemanticVerify::MapMergeConfig mergeConfig =
        semantic::SemanticVerify::mapMergeConfigFromSystemParams();

    for (Map *p_oldMap : GetAllMaps())
    {
        if (p_oldMap == nullptr || p_oldMap == p_currentMap ||
            p_oldMap->IsBad())
        {
            continue;
        }
        if (!consecutiveSeedTagsMatch(p_oldMap, p_currentMap))
        {
            continue;
        }
        if (countLiveRooms(p_oldMap) < minimumRooms ||
            countLiveRooms(p_currentMap) < minimumRooms ||
            countLiveWallPlanes(p_oldMap) < minimumWalls ||
            countLiveWallPlanes(p_currentMap) < minimumWalls ||
            countLiveFloors(p_oldMap) == 0U ||
            countLiveFloors(p_currentMap) == 0U)
        {
            continue;
        }

        const long unsigned int oldMapId = p_oldMap->GetId();
        const std::size_t       contentHash =
            consecutiveContentHash(p_oldMap, p_currentMap);
        const std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now();
        MergeAttemptState attemptState;
        {
            std::unique_lock<std::mutex> atlasLock(mMutexAtlas);
            const auto storedState = mConsecutiveMergeState.find(oldMapId);
            if (storedState != mConsecutiveMergeState.end())
            {
                attemptState = storedState->second;
            }
        }
        if (attemptState.hasEverAttempted)
        {
            if (now - attemptState.lastAttemptTime <
                std::chrono::seconds(cooldown_s))
            {
                continue;
            }
            if (attemptState.contentHashAtLastAttempt == contentHash)
            {
                continue;
            }
        }

        const std::set<std::string> anchorTags =
            collectAnchorTags(p_oldMap, p_currentMap);
        const std::size_t anchorCount   = anchorTags.size();
        auto              recordAttempt = [&](void)
        {
            attemptState.lastAttemptTime          = now;
            attemptState.contentHashAtLastAttempt = contentHash;
            attemptState.hasEverAttempted         = true;
            std::unique_lock<std::mutex> atlasLock(mMutexAtlas);
            mConsecutiveMergeState[oldMapId] = attemptState;
        };
        /* The seed room itself must be anchored: matching side rooms while
         * the prior-link room takes part nowhere would fuse on a
         * coincidental resemblance. Same DEFER as too few anchors. */
        semantic::Room      *p_oldFinalRoom = p_oldMap->getFinalRoom();
        const bool seedAnchored =
            p_oldFinalRoom != nullptr && p_oldFinalRoom->hasRoomTag() &&
            anchorTags.count(p_oldFinalRoom->getRoomTag()) > 0U;
        if (anchorCount < minimumAnchors || !seedAnchored)
        {
            recordAttempt();
            std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_"
                         "attempt\",\"old_map_id\":"
                      << oldMapId << ",\"new_map_id\":" << p_currentMap->GetId()
                      << ",\"anchors\":" << anchorCount
                      << ",\"decision\":\"DEFER\",\"reason\":\""
                         "SHARED_ROOM_IDENTITY_MISSING\"}"
                      << std::endl;
            continue;
        }

        std::vector<Eigen::Vector3d> normalsCurrent, centroidsCurrent;
        std::vector<Eigen::Vector3d> normalsOld, centroidsOld;
        if (!Utils::collectCorrespondingWalls(p_currentMap,
                                              p_oldMap,
                                              normalsCurrent,
                                              centroidsCurrent,
                                              normalsOld,
                                              centroidsOld))
        {
            recordAttempt();
            std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_"
                         "attempt\",\"old_map_id\":"
                      << oldMapId << ",\"new_map_id\":" << p_currentMap->GetId()
                      << ",\"anchors\":" << anchorCount
                      << ",\"decision\":\"DEFER\",\"reason\":\""
                         "WALL_EVIDENCE_MISSING\"}"
                      << std::endl;
            continue;
        }
        const Eigen::Isometry3d transformOldToCurrent =
            Utils::computeMapTransform_Horn(normalsOld,
                                            centroidsOld,
                                            normalsCurrent,
                                            centroidsCurrent);
        if (!transformOldToCurrent.matrix().allFinite())
        {
            recordAttempt();
            std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_"
                         "attempt\",\"old_map_id\":"
                      << oldMapId << ",\"new_map_id\":" << p_currentMap->GetId()
                      << ",\"anchors\":" << anchorCount
                      << ",\"decision\":\"DEFER\",\"reason\":\""
                         "WALL_EVIDENCE_MISSING\"}"
                      << std::endl;
            continue;
        }
        const g2o::Sim3 transformSim3(transformOldToCurrent.linear(),
                                      transformOldToCurrent.translation(),
                                      1.0);
        const semantic::SemanticMergeGateResult gateResult =
            semantic::SemanticVerify::evaluateConsecutiveMergeGate(p_currentMap,
                                                         p_oldMap,
                                                         transformSim3,
                                                         mergeConfig);
        recordAttempt();
        std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_attempt\","
                     "\"old_map_id\":"
                  << oldMapId << ",\"new_map_id\":" << p_currentMap->GetId()
                  << ",\"anchors\":" << anchorCount << ",\"decision\":\""
                  << semantic::SemanticVerify::mergeDecisionName(gateResult.decision)
                  << "\",\"reason\":\""
                  << semantic::SemanticVerify::mergeReasonName(gateResult.reason)
                  << "\",\"matched_walls\":" << gateResult.matchedWallCount
                  << ",\"matched_passages\":" << gateResult.matchedPassageCount
                  << "}" << std::endl;
        if (gateResult.decision != semantic::SemanticMergeDecision::ACCEPT)
        {
            continue;
        }
        MergeMapPair(p_currentMap, p_oldMap);
        std::cout << "SG_PIPELINE {\"event\":\"consecutive_merge_"
                     "committed\",\"old_map_id\":"
                  << oldMapId << ",\"new_map_id\":" << p_currentMap->GetId()
                  << "}" << std::endl;
        /* One merge per call: the current map changed shape, so remaining
         * pairs re-evaluate from scratch next cycle. */
        break;
    }
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

    std::set<camera_models::GeometricCamera *> spCams(mvpCameras.begin(), mvpCameras.end());
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
    map<unsigned int, camera_models::GeometricCamera *> mpCams;
    for (camera_models::GeometricCamera *pCam : mvpCameras)
    {
        mpCams[pCam->getId()] = pCam;
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
    if (!mpCurrentMap)
        return;

    /* Export BOTH confirmed detected rooms AND candidate/marker-based rooms.
     * Candidate rooms (prospective/provisional) may not have full wall loops
     * yet but still carry spatial identity needed for cross-restart matching.
     */
    std::vector<semantic::Room *> rooms = mpCurrentMap->GetAllDetectedMapRooms();
    std::vector<semantic::Room *> candidateRooms =
        mpCurrentMap->GetAllCandidateMapRooms();
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());

    if (rooms.empty())
        return;

    std::vector<semantic::RoomContextSnapshot> snapshots;
    snapshots.reserve(rooms.size());

    const long unsigned int mapId = mpCurrentMap->GetId();

    for (semantic::Room *room : rooms)
    {
        if (!room || room->isBad())
            continue;

        semantic::RoomContextSnapshot snap;
        snap.roomId        = room->getId();
        semantic::Floor *p_snapFloor = room->getFloor();
        snap.floorId       = p_snapFloor != nullptr ? p_snapFloor->getId() : -1;
        snap.centroid      = room->getCentroid();
        snap.wasConfirmedRoom =
            room->getRoomVariant() == semantic::Room::roomVariant::ROOM;
        snap.wasPreviouslyVisited = room->hasPreviouslyVisited();
        snap.boundaryStatus       = static_cast<int>(room->getBoundaryStatus());
        snap.timestamp =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();

        for (geometric::Plane *wall : room->getWalls())
        {
            semantic::WallBounds bounds;
            if (!wall || wall->isBad())
            {
                snap.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snap.wallCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snap.wallDistances.push_back(
                    std::numeric_limits<double>::quiet_NaN());
                snap.wallBounds.push_back(bounds);
                continue;
            }

            std::optional<Eigen::Vector3d> orientedNormal =
                room->getWallNormalTowardRoom_World(wall);
            if (orientedNormal)
                snap.wallNormals.push_back(*orientedNormal);
            else
                snap.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));

            snap.wallCentroids.push_back(wall->getCentroid());
            snap.wallDistances.push_back(wall->getGlobalEquation().distance());
            const geometric::Plane::GeometrySnapshot geometry =
                wall->getGeometrySnapshot();
            bounds.minU_m = geometry.minPlaneU_m;
            bounds.maxU_m = geometry.maxPlaneU_m;
            bounds.minV_m = geometry.minPlaneV_m;
            bounds.maxV_m = geometry.maxPlaneV_m;
            bounds.valid =
                std::isfinite(bounds.minU_m) && std::isfinite(bounds.maxU_m) &&
                std::isfinite(bounds.minV_m) && std::isfinite(bounds.maxV_m) &&
                bounds.maxU_m > bounds.minU_m && bounds.maxV_m > bounds.minV_m;
            snap.wallBounds.push_back(bounds);
        }

        for (semantic::Passage *passage : room->getPassages())
        {
            if (!passage)
            {
                snap.passageCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snap.passageContexts.push_back(semantic::PassageContext());
                continue;
            }
            snap.passageCentroids.push_back(passage->getCentroid());
            semantic::PassageContext context;
            context.id       = passage->getId();
            context.passable = passage->isPassable();
            const std::optional<int> roomIdOfPassageObservationConnection =
                passage->getProspectiveRoomId();
            context.hasFarSideRoom =
                roomIdOfPassageObservationConnection.has_value();
            if (context.hasFarSideRoom)
                context.secondaryRoomId = *roomIdOfPassageObservationConnection;
            context.width_m       = passage->getWidth();
            context.height_m      = passage->getHeight();
            context.apertureValid = std::isfinite(context.width_m) &&
                                    std::isfinite(context.height_m) &&
                                    context.width_m > 0.0 &&
                                    context.height_m > 0.0;
            const semantic::Passage::KnownSideProvenance knownSide =
                passage->getKnownSideProvenance();
            context.hasKnownSideDirection = knownSide.hasDirection();
            if (context.hasKnownSideDirection)
            {
                context.knownSideDirection_World = knownSide.direction_World;
            }
            context.hasKnownSideRoom = knownSide.pRoom != nullptr;
            if (context.hasKnownSideRoom)
            {
                context.knownSideRoomId = knownSide.pRoom->getId();
            }
            context.traversalKnownToFarCount =
                passage->getTraversalKnownToFarCount();
            context.traversalFarToKnownCount =
                passage->getTraversalFarToKnownCount();
            context.traversalUnknownCount = passage->getTraversalUnknownCount();
            context.associatedWallCount   = passage->getAssociateWalls().size();
            context.hasBidirectionalTraversalEvidence =
                passage->hasBidirectionalTraversalEvidence();
            snap.passageContexts.push_back(context);
        }

        /* Assign persistent tag to old map rooms for merge trigger.
         * Tag format: "room_<id>" matches what matchRoomsToContext() assigns.
         */
        const std::string roomTag = "room_" + std::to_string(room->getId());
        snap.roomTag              = roomTag;
        if (!room->hasRoomTag())
        {
            room->setRoomTag(roomTag);
        }

        snapshots.push_back(snap);
    }

    const std::size_t exportedRoomCount = snapshots.size();
    {
        std::lock_guard<std::mutex> contextLock(mRoomContextMutex);
        mRoomContextHistory[mapId] = std::move(snapshots);
    }

    /* Record the departure room for mission-chain tracing: the room the UAV
     * was following when this map was stranded. */
    const int departureRoomId = getCurrentSemanticRoomIdentity();
    if (departureRoomId >= 0)
    {
        for (semantic::Room *room : rooms)
        {
            if (room != nullptr && !room->isBad() &&
                room->getId() == departureRoomId)
            {
                mpCurrentMap->setFinalRoom(room);
                break;
            }
        }
    }

    std::cout << "[Atlas] Exported room context: " << exportedRoomCount
              << " rooms (mapId: " << mapId << ")" << std::endl;
}

void Atlas::matchRoomsToContext(Map *pNewMap)
{
    if (!pNewMap)
        return;

    /* Candidate generation is intentionally separate from P4 verification.
     * This legacy method used cross-map centroids/normals and transferred live
     * walls before verification; it is retained as a disabled compatibility
     * entry point until the verified merge seam exists. */
    return;

    /* Snapshot Atlas membership before taking the room-context lock. Map
     * creation takes these locks in the opposite sequence by necessity. */
#if defined(VS_GRAPHS_ENABLE_ATLAS_LOCK_ORDER_TEST_HOOK)
    if (vsGraphsAtlasLockOrderBeforeMapSnapshot != nullptr)
    {
        vsGraphsAtlasLockOrderBeforeMapSnapshot();
    }
#endif
    const std::vector<Map *> allMaps = GetAllMaps();

    std::unique_lock<std::mutex> lock(mRoomContextMutex);

    if (mRoomContextHistory.empty())
        return;

    /* Match BOTH detected rooms AND candidate/prospective rooms.
     * Candidate rooms need identity tags for cross-restart continuity. */
    std::vector<semantic::Room *> newRooms       = pNewMap->GetAllDetectedMapRooms();
    std::vector<semantic::Room *> candidateRooms = pNewMap->GetAllCandidateMapRooms();
    newRooms.insert(newRooms.end(),
                    candidateRooms.begin(),
                    candidateRooms.end());

    if (newRooms.empty())
        return;

    std::vector<semantic::RoomContextSnapshot> allContext;
    for (const auto &entry : mRoomContextHistory)
        for (const auto &snap : entry.second)
            allContext.push_back(snap);

    if (allContext.empty())
        return;

    for (semantic::Room *room : newRooms)
    {
        if (!room || room->isBad())
            continue;

        if (room->hasRoomTag())
            continue;

        Eigen::Vector3d roomCentroid = room->getCentroid();
        double          bestDist     = std::numeric_limits<double>::max();
        const semantic::RoomContextSnapshot *bestMatch = nullptr;

        for (const semantic::RoomContextSnapshot &snap : allContext)
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
            std::vector<geometric::Plane *> roomWalls = room->getWalls();
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
            Map  *p_priorMap  = nullptr;
            semantic::Room *p_priorRoom = nullptr;
            for (Map *p_map : allMaps)
            {
                if (!p_map || p_map->IsBad() || p_map == pNewMap)
                    continue;
                for (semantic::Room *r : p_map->GetAllDetectedMapRooms())
                {
                    if (r && !r->isBad() && r->getId() == bestMatch->roomId)
                    {
                        p_priorRoom = r;
                        p_priorMap  = p_map;
                        break;
                    }
                }
                if (p_priorRoom)
                    break;
            }

            if (p_priorRoom && p_priorMap)
            {
                std::cout << "[Atlas] Prior semantic::Room#" << p_priorRoom->getId()
                          << " has " << p_priorRoom->getWalls().size()
                          << " walls" << std::endl;

                /* Transfer walls from prior room to current room */
                for (geometric::Plane *p_wall : p_priorRoom->getWalls())
                {
                    if (!p_wall || p_wall->isBad())
                        continue;

                    /* Re-associate wall to new room */
                    p_priorRoom->removeWall(p_wall);
                    room->setWalls(p_wall);

                    std::cout << "[Atlas] Transferred Wall#" << p_wall->getId()
                              << " from prior semantic::Room#" << p_priorRoom->getId()
                              << " to matched semantic::Room#" << room->getId()
                              << std::endl;
                }

                /* Passages will be re-associated by associatePassagesToRooms()
                 */

                std::cout << "[Atlas] semantic::Room#" << room->getId() << " now has "
                          << room->getWalls().size()
                          << " walls (continuing from prior semantic::Room#"
                          << bestMatch->roomId << ")" << std::endl;
            }
            else
            {
                std::cout
                    << "[Atlas] NO prior room found for match (p_priorRoom="
                    << p_priorRoom << ", p_priorMap=" << p_priorMap << ")"
                    << std::endl;
            }

            std::cout << "[Atlas] Matched room " << room->getId()
                      << " in new map, tagged with identity \""
                      << room->getRoomTag() << "\" (prior room "
                      << bestMatch->roomId << ", dist=" << bestDist << " m)"
                      << std::endl;
        }
    }
}

std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
    Atlas::copyRoomContextHistory() const
{
    std::lock_guard<std::mutex> contextLock(mRoomContextMutex);
    return mRoomContextHistory;
}

std::vector<semantic::RoomContextSnapshot> Atlas::copyRoomContextForMap(Map *p_map_in)
{
    std::vector<semantic::RoomContextSnapshot> snapshots;
    if (p_map_in == nullptr)
        return snapshots;
    std::vector<semantic::Room *> rooms          = p_map_in->GetAllDetectedMapRooms();
    std::vector<semantic::Room *> candidateRooms = p_map_in->GetAllCandidateMapRooms();
    rooms.insert(rooms.end(), candidateRooms.begin(), candidateRooms.end());
    for (semantic::Room *p_room : rooms)
    {
        if (p_room == nullptr || p_room->isBad())
            continue;
        semantic::RoomContextSnapshot snapshot;
        snapshot.roomId        = p_room->getId();
        semantic::Floor *p_snapshotFloor = p_room->getFloor();
        snapshot.floorId =
            p_snapshotFloor != nullptr ? p_snapshotFloor->getId() : -1;
        snapshot.centroid = p_room->getCentroid();
        snapshot.wasConfirmedRoom =
            p_room->getRoomVariant() == semantic::Room::roomVariant::ROOM;
        snapshot.boundaryStatus = static_cast<int>(p_room->getBoundaryStatus());
        snapshot.timestamp =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        snapshot.roomTag = p_room->hasRoomTag() ? p_room->getRoomTag() : "";
        for (geometric::Plane *p_wall : p_room->getWalls())
        {
            if (p_wall == nullptr || p_wall->isBad())
            {
                snapshot.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snapshot.wallCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snapshot.wallDistances.push_back(
                    std::numeric_limits<double>::quiet_NaN());
                snapshot.wallBounds.push_back(semantic::WallBounds());
                continue;
            }
            const std::optional<Eigen::Vector3d> normal =
                p_room->getWallNormalTowardRoom_World(p_wall);
            if (normal)
                snapshot.wallNormals.push_back(*normal);
            else
                snapshot.wallNormals.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
            snapshot.wallCentroids.push_back(p_wall->getCentroid());
            snapshot.wallDistances.push_back(
                p_wall->getGlobalEquation().distance());
            const geometric::Plane::GeometrySnapshot geometry =
                p_wall->getGeometrySnapshot();
            snapshot.wallBounds.push_back(
                {std::isfinite(geometry.minPlaneU_m) &&
                     std::isfinite(geometry.maxPlaneU_m) &&
                     std::isfinite(geometry.minPlaneV_m) &&
                     std::isfinite(geometry.maxPlaneV_m) &&
                     geometry.maxPlaneU_m > geometry.minPlaneU_m &&
                     geometry.maxPlaneV_m > geometry.minPlaneV_m,
                 geometry.minPlaneU_m,
                 geometry.maxPlaneU_m,
                 geometry.minPlaneV_m,
                 geometry.maxPlaneV_m});
        }
        for (semantic::Passage *p_passage : p_room->getPassages())
        {
            if (p_passage == nullptr)
            {
                snapshot.passageCentroids.push_back(Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()));
                snapshot.passageContexts.push_back(semantic::PassageContext());
                continue;
            }
            snapshot.passageCentroids.push_back(p_passage->getCentroid());
            semantic::PassageContext context;
            context.id       = p_passage->getId();
            context.passable = p_passage->isPassable();
            const std::optional<int> roomIdOfPassageObservationConnection =
                p_passage->getProspectiveRoomId();
            context.hasFarSideRoom =
                roomIdOfPassageObservationConnection.has_value();
            if (context.hasFarSideRoom)
                context.secondaryRoomId = *roomIdOfPassageObservationConnection;
            context.width_m       = p_passage->getWidth();
            context.height_m      = p_passage->getHeight();
            context.apertureValid = std::isfinite(context.width_m) &&
                                    std::isfinite(context.height_m) &&
                                    context.width_m > 0.0 &&
                                    context.height_m > 0.0;
            const semantic::Passage::KnownSideProvenance knownSide =
                p_passage->getKnownSideProvenance();
            context.hasKnownSideDirection = knownSide.hasDirection();
            if (context.hasKnownSideDirection)
            {
                context.knownSideDirection_World = knownSide.direction_World;
            }
            context.hasKnownSideRoom = knownSide.pRoom != nullptr;
            if (context.hasKnownSideRoom)
            {
                context.knownSideRoomId = knownSide.pRoom->getId();
            }
            context.traversalKnownToFarCount =
                p_passage->getTraversalKnownToFarCount();
            context.traversalFarToKnownCount =
                p_passage->getTraversalFarToKnownCount();
            context.traversalUnknownCount =
                p_passage->getTraversalUnknownCount();
            context.associatedWallCount = p_passage->getAssociateWalls().size();
            context.hasBidirectionalTraversalEvidence =
                p_passage->hasBidirectionalTraversalEvidence();
            snapshot.passageContexts.push_back(context);
        }
        snapshots.push_back(std::move(snapshot));
    }
    return snapshots;
}

std::optional<semantic::RoomContextSnapshot>
    Atlas::copyLatestRoomContext(const int roomId_in) const
{
    std::lock_guard<std::mutex>        contextLock(mRoomContextMutex);
    std::optional<semantic::RoomContextSnapshot> latestSnapshot;

    for (const std::pair<const long unsigned int,
                         std::vector<semantic::RoomContextSnapshot>> &historyEntry :
         mRoomContextHistory)
    {
        static_cast<void>(historyEntry.first);
        for (const semantic::RoomContextSnapshot &snapshot : historyEntry.second)
        {
            if (snapshot.roomId != roomId_in ||
                (latestSnapshot.has_value() &&
                 snapshot.timestamp <= latestSnapshot->timestamp))
            {
                continue;
            }
            latestSnapshot = snapshot;
        }
    }

    return latestSnapshot;
}

Atlas::SnapshotCopyResult
    Atlas::copyRoomContextForMapChecked(Map       *p_map_in,
                                        const bool callerOwnsSemanticLock)
{
    SnapshotCopyResult result;
    if (!callerOwnsSemanticLock)
        return result;

    result.snapshots = copyRoomContextForMap(p_map_in);
    result.status    = SnapshotCopyStatus::COMPLETE;
    return result;
}

bool Atlas::consumeNewMapCreatedEvent()
{
    std::lock_guard<std::mutex> contextLock(mRoomContextMutex);
    const bool                  wasCreated = newMapCreatedPending_;
    newMapCreatedPending_                  = false;
    return wasCreated;
}

bool Atlas::peekNewMapCreatedEvent() const
{
    std::lock_guard<std::mutex> contextLock(mRoomContextMutex);
    return newMapCreatedPending_;
}

void Atlas::acknowledgeNewMapCreatedEvent()
{
    std::lock_guard<std::mutex> contextLock(mRoomContextMutex);
    newMapCreatedPending_ = false;
}

const std::vector<semantic::RoomContextSnapshot> &
    Atlas::getRoomContextForMap(long unsigned int mapId) const
{
    /* Compatibility API: callers requiring synchronization must use the copy
     * API. The historical reference lifetime cannot be made lock-safe. */
    static const std::vector<semantic::RoomContextSnapshot> empty;
    auto it = mRoomContextHistory.find(mapId);
    if (it == mRoomContextHistory.end())
        return empty;
    return it->second;
}

} // namespace core
} // namespace vs_graphs
