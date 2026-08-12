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

#include "Map.h"

#include <algorithm>
#include <iterator>
#include <mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId = 0;

Map::Map() :
    mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
    mbFail(false),
    mbImuInitialized(false),
    mnMapChange(0),
    mnMapChangeNotified(0),
    mnWorldFrameEpoch(0U),
    mnMaxKFid(0),
    mnBigChangeIdx(0),
    mIsInUse(false),
    mHasTumbnail(false),
    mbBad(false),
    mbIsInertial(false),
    mbIMU_BA1(false),
    mbIMU_BA2(false)
{
    mnId       = nNextId++;
    mThumbnail = static_cast<GLubyte *>(NULL);
}

Map::Map(int initKFid) :
    mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
    mbFail(false),
    mbImuInitialized(false),
    mnMapChange(0),
    mnMapChangeNotified(0),
    mnWorldFrameEpoch(0U),
    mnInitKFid(initKFid),
    mnMaxKFid(initKFid),
    mnBigChangeIdx(0),
    mIsInUse(false),
    mHasTumbnail(false),
    mbBad(false),
    mbIsInertial(false),
    mbIMU_BA1(false),
    mbIMU_BA2(false)
{
    mnId       = nNextId++;
    mThumbnail = static_cast<GLubyte *>(NULL);
}

Map::~Map()
{
    // TODO: erase all points from memory
    mspMapPoints.clear();

    // TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    // Erase all markers from memory
    mspMarkers.clear();

    // Erase all semantic entities from memory
    mspFloors.clear();
    mspDoors.clear();
    mspPlanes.clear();
    mspPassages.clear();
    mspDetectedRooms.clear();
    mspMarkerBasedRooms.clear();

    if (mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte *>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);

    // Check if the KeyFrames are already in the map
    if (mspKeyFrames.empty())
    {
        std::cout << "\n[Mapping] Map initialized with initial KeyFrame #"
                  << mnInitKFid << "." << std::endl;
        mnInitKFid  = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }

    // Add the KeyFrame to the map
    mspKeyFrames.insert(pKF);

    // Update the maximum KeyFrame id
    if (pKF->mnId > mnMaxKFid)
        mnMaxKFid = pKF->mnId;

    if (pKF->mnId < mpKFlowerID->mnId)
        mpKFlowerID = pKF;

    mKFIndex[pKF->mnId] = pKF;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::AddMapMarker(Marker *pMarker)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMarkers.insert(pMarker);
    // Add the marker to the hashmap
    mMarkerIndex[pMarker->getId()] = pMarker;
}

void Map::AddMapPlane(Plane *pPlane)
{
    if (pPlane == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexMap);

    for (auto planeIterator = mPlaneIndex.begin();
         planeIterator != mPlaneIndex.end();)
    {
        planeIterator =
            planeIterator->second == pPlane &&
                    planeIterator->first != pPlane->getId()
                ? mPlaneIndex.erase(planeIterator)
                : std::next(planeIterator);
    }

    const auto existingPlaneIterator = mPlaneIndex.find(pPlane->getId());

    if (pPlane->getId() < 0 || (existingPlaneIterator != mPlaneIndex.end() &&
                                existingPlaneIterator->second != pPlane))
    {
        while (mPlaneIndex.count(nextAvailablePlaneId) > 0)
        {
            ++nextAvailablePlaneId;
        }

        const int replacementPlaneId = nextAvailablePlaneId++;

        std::cerr << "[Map] Plane ID collision for " << pPlane->getId()
                  << "; reassigned to " << replacementPlaneId << "."
                  << std::endl;

        pPlane->setId(replacementPlaneId);
    }
    else
    {
        nextAvailablePlaneId =
            std::max(nextAvailablePlaneId, pPlane->getId() + 1);
    }

    mspPlanes.insert(pPlane);
    mPlaneIndex.insert_or_assign(pPlane->getId(), pPlane);
}

void Map::AddRoomWallPlane(ORB_SLAM3::Plane *pPlane)
{
    unique_lock<mutex> lock(mMutexMap);
    // Add the plane to the hashmap
    mRoomWallPlaneIndex[pPlane->getId()] = pPlane;
}

void ORB_SLAM3::Map::AddMapPassage(ORB_SLAM3::Passage *pPassage)
{
    if (pPassage == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexMap);

    const auto existingPassage = mPassageIndex.find(pPassage->getId());
    if (pPassage->getId() < 0 ||
        (existingPassage != mPassageIndex.end() &&
         existingPassage->second != pPassage))
    {
        std::cerr << "[Map] Passage ID collision for " << pPassage->getId()
                  << "; caller must resolve it before destination insertion."
                  << std::endl;
        return;
    }

    mspPassages.insert(pPassage);
    mPassageIndex.insert_or_assign(pPassage->getId(), pPassage);
}

void Map::AddDetectedMapRoom(Room *pRoom)
{
    unique_lock<mutex> lock(mMutexMap);
    mspDetectedRooms.insert(pRoom);
}

void Map::AddCandidateMapRoom(Room *pRoom)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMarkerBasedRooms.insert(pRoom);
}

void Map::PromoteCandidateMapRoom(Room *pRoom)
{
    if (pRoom == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexMap);
    mspMarkerBasedRooms.erase(pRoom);
    mspDetectedRooms.insert(pRoom);
}

void Map::AddMapFloor(Floor *pFloor)
{
    if (pFloor == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexMap);

    for (auto floorIterator = mFloorIndex.begin();
         floorIterator != mFloorIndex.end();)
    {
        floorIterator =
            floorIterator->second == pFloor &&
                    floorIterator->first != pFloor->getId()
                ? mFloorIndex.erase(floorIterator)
                : std::next(floorIterator);
    }

    const auto existingFloorIterator = mFloorIndex.find(pFloor->getId());

    if (pFloor->getId() < 0 || (existingFloorIterator != mFloorIndex.end() &&
                                existingFloorIterator->second != pFloor))
    {
        while (mFloorIndex.count(nextAvailableFloorId) > 0)
        {
            ++nextAvailableFloorId;
        }

        const int replacementFloorId = nextAvailableFloorId++;

        std::cerr << "[Map] Floor ID collision for " << pFloor->getId()
                  << "; reassigned to " << replacementFloorId << "."
                  << std::endl;

        pFloor->setId(replacementFloorId);
    }
    else
    {
        nextAvailableFloorId =
            std::max(nextAvailableFloorId, pFloor->getId() + 1);
    }

    mspFloors.insert(pFloor);
    mFloorIndex.insert_or_assign(pFloor->getId(), pFloor);
}

int Map::reservePlaneId(void)
{
    unique_lock<mutex> lock(mMutexMap);

    while (mPlaneIndex.count(nextAvailablePlaneId) > 0)
    {
        ++nextAvailablePlaneId;
    }

    return nextAvailablePlaneId++;
}

int Map::reserveFloorId(void)
{
    unique_lock<mutex> lock(mMutexMap);

    while (mFloorIndex.count(nextAvailableFloorId) > 0)
    {
        ++nextAvailableFloorId;
    }

    return nextAvailableFloorId++;
}

void Map::AddMapDoor(Door *pDoor)
{
    unique_lock<mutex> lock(mMutexMap);
    mspDoors.insert(pDoor);
}

KeyFrame *Map::GetKeyFrameById(long unsigned int mnId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         keyFrameIterator = mKFIndex.find(mnId);
    return keyFrameIterator != mKFIndex.end() ? keyFrameIterator->second
                                              : nullptr;
}

ORB_SLAM3::Passage *Map::GetPassageById(int passageId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         passageIterator = mPassageIndex.find(passageId);
    return passageIterator != mPassageIndex.end() ? passageIterator->second
                                                  : nullptr;
}

Plane *Map::GetPlaneById(int planeId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         planeIterator = mPlaneIndex.find(planeId);
    return planeIterator != mPlaneIndex.end() ? planeIterator->second : nullptr;
}

ORB_SLAM3::Plane *Map::GetRoomWallPlaneById(int planeId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         wallIterator = mRoomWallPlaneIndex.find(planeId);
    return wallIterator != mRoomWallPlaneIndex.end() ? wallIterator->second
                                                     : nullptr;
}

Marker *Map::GetMarkerById(int markerId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         markerIterator = mMarkerIndex.find(markerId);
    return markerIterator != mMarkerIndex.end() ? markerIterator->second
                                                : nullptr;
}

Floor *Map::GetFloorById(int floorId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         floorIterator = mFloorIndex.find(floorId);
    return floorIterator != mFloorIndex.end() ? floorIterator->second : nullptr;
}

Door *Map::GetDoorById(int doorId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         doorIterator = mDoorIndex.find(doorId);
    return doorIterator != mDoorIndex.end() ? doorIterator->second : nullptr;
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    mvpReferenceMapPoints.erase(
        std::remove(mvpReferenceMapPoints.begin(),
                    mvpReferenceMapPoints.end(),
                    pMP),
        mvpReferenceMapPoints.end());

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseMapMarker(Marker *pMarker)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMarkers.erase(pMarker);

    for (auto markerIterator = mMarkerIndex.begin();
         markerIterator != mMarkerIndex.end();)
    {
        markerIterator = markerIterator->second == pMarker
                             ? mMarkerIndex.erase(markerIterator)
                             : std::next(markerIterator);
    }
}

void Map::EraseMapPlane(Plane *pPlane)
{
    unique_lock<mutex> lock(mMutexMap);
    mspPlanes.erase(pPlane);

    for (auto planeIterator = mPlaneIndex.begin();
         planeIterator != mPlaneIndex.end();)
    {
        planeIterator = planeIterator->second == pPlane
                            ? mPlaneIndex.erase(planeIterator)
                            : std::next(planeIterator);
    }
}

void Map::EraseRoomWallPlane(ORB_SLAM3::Plane *pPlane)
{
    unique_lock<mutex> lock(mMutexMap);

    for (auto wallIterator = mRoomWallPlaneIndex.begin();
         wallIterator != mRoomWallPlaneIndex.end();)
    {
        wallIterator = wallIterator->second == pPlane
                           ? mRoomWallPlaneIndex.erase(wallIterator)
                           : std::next(wallIterator);
    }
}

void Map::EraseMapPassage(ORB_SLAM3::Passage *pPassage)
{
    unique_lock<mutex> lock(mMutexMap);
    mspPassages.erase(pPassage);

    for (auto passageIterator = mPassageIndex.begin();
         passageIterator != mPassageIndex.end();)
    {
        passageIterator = passageIterator->second == pPassage
                              ? mPassageIndex.erase(passageIterator)
                              : std::next(passageIterator);
    }
}

void Map::EraseMapFloor(ORB_SLAM3::Floor *p_floor_in)
{
    unique_lock<mutex> lock(mMutexMap);
    mspFloors.erase(p_floor_in);

    for (auto floorIterator = mFloorIndex.begin();
         floorIterator != mFloorIndex.end();)
    {
        floorIterator = floorIterator->second == p_floor_in
                            ? mFloorIndex.erase(floorIterator)
                            : std::next(floorIterator);
    }
}

void Map::ClearTransferredEntityIndexes()
{
    unique_lock<mutex> lock(mMutexMap);
    mFloorIndex.clear();
    mPlaneIndex.clear();
    mMarkerIndex.clear();
    mKFIndex.clear();
    mPassageIndex.clear();
    mRoomWallPlaneIndex.clear();
}

void Map::EraseDetectedMapRoom(Room *pRoom)
{
    unique_lock<mutex> lock(mMutexMap);
    mspDetectedRooms.erase(pRoom);
}

void Map::EraseMarkerBasedMapRoom(Room *pRoom)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMarkerBasedRooms.erase(pRoom);
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    mKFIndex.erase(pKF->mnId);
    mvpKeyFrameOrigins.erase(
        std::remove(mvpKeyFrameOrigins.begin(),
                    mvpKeyFrameOrigins.end(),
                    pKF),
        mvpKeyFrameOrigins.end());

    if (mpFirstRegionKF == pKF)
    {
        mpFirstRegionKF = nullptr;
    }

    if (mpKFinitial == pKF)
    {
        mpKFinitial = nullptr;
    }

    if (mspKeyFrames.size() > 0)
    {
        if (pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame *> vpKFs =
                vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }

        if (mpKFinitial == nullptr)
        {
            mpKFinitial = mpKFlowerID;
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

std::vector<std::vector<Eigen::Vector3d>> Map::GetSkeletonClusterPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return skeletonClusterPoints;
}

void Map::SetSkeletonClusterPoints(
    const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints)
{
    unique_lock<mutex> lock(mMutexMap);
    skeletonClusterPoints = newClusterPoints;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
    Map::GetSkeletonEdges(void)
{
    /* Lock access to the map data */
    unique_lock<mutex> lock(mMutexMap);

    /* Return a copy of the latest connected skeleton edges */
    return mSkeletonEdges;
}

void Map::SetSkeletonEdges(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        &newSkeletonEdges)
{
    /* Lock access to the map data */
    unique_lock<mutex> lock(mMutexMap);

    /* Replace the previous connected skeleton edge collection */
    mSkeletonEdges = newSkeletonEdges;
}

vector<KeyFrame *> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
}

vector<MapPoint *> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
}

vector<Marker *> Map::GetAllMarkers()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Marker *>(mspMarkers.begin(), mspMarkers.end());
}

vector<Plane *> Map::GetAllPlanes()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Plane *>(mspPlanes.begin(), mspPlanes.end());
}

Plane *Map::GetBiggestGroundPlane()
{
    Plane *bestGroundPlane = nullptr;
    std::tuple<std::size_t, std::size_t, int> bestEvidence{0U, 0U, 0};
    bool hasBestEvidence = false;

    for (Plane *pPlane : GetAllPlanes())
    {
        if (pPlane == nullptr || pPlane->isBad() ||
            pPlane->getPlaneType() != Plane::planeVariant::GROUND)
        {
            continue;
        }

        const Plane::GeometrySnapshot geometry =
            pPlane->getGeometrySnapshot();
        const double normalNorm = geometry.equation_World.head<3>().norm();
        if (!geometry.equation_World.allFinite() ||
            !std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }

        if (geometry.cloudGeneration != geometry.successfulRefitGeneration ||
            geometry.finiteSupportCount == 0U ||
            std::abs(normalNorm - 1.0) > 1e-3)
        {
            continue;
        }

        const auto evidence =
            std::make_tuple(geometry.finiteSupportCount,
                            geometry.observationCount,
                            -pPlane->getId());
        if (!hasBestEvidence || evidence > bestEvidence)
        {
            bestEvidence = evidence;
            bestGroundPlane = pPlane;
            hasBestEvidence = true;
        }
    }
    return bestGroundPlane;
}

std::vector<ORB_SLAM3::Passage *> Map::GetAllPassages()
{
    unique_lock<mutex> lock(mMutexMap);
    return std::vector<ORB_SLAM3::Passage *>(mspPassages.begin(),
                                             mspPassages.end());
}

vector<Room *> Map::GetAllRooms()
{
    unique_lock<mutex> lock(mMutexMap);
    vector<Room *>     allRooms;
    allRooms.insert(allRooms.end(),
                    mspDetectedRooms.begin(),
                    mspDetectedRooms.end());
    allRooms.insert(allRooms.end(),
                    mspMarkerBasedRooms.begin(),
                    mspMarkerBasedRooms.end());
    return allRooms;
}

vector<Room *> Map::GetAllDetectedMapRooms()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Room *>(mspDetectedRooms.begin(), mspDetectedRooms.end());
}

vector<Room *> Map::GetAllMarkerBasedMapRooms()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Room *>(mspMarkerBasedRooms.begin(),
                          mspMarkerBasedRooms.end());
}

vector<Room *> Map::GetAllCandidateMapRooms()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Room *>(mspMarkerBasedRooms.begin(),
                          mspMarkerBasedRooms.end());
}

vector<Floor *> Map::GetAllFloors()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Floor *>(mspFloors.begin(), mspFloors.end());
}

vector<Door *> Map::GetAllDoors()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Door *>(mspDoors.begin(), mspDoors.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::MarkersInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMarkers.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint *> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}

long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame *Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
    for (set<KeyFrame *>::iterator sit  = mspKeyFrames.begin(),
                                   send = mspKeyFrames.end();
         sit != send;
         sit++)
    {
        KeyFrame *pKF = *sit;
        pKF->UpdateMap(static_cast<Map *>(NULL));
    }

    mspPlanes.clear();
    mspMarkers.clear();
    mspPassages.clear();
    mspFloors.clear();
    mspDoors.clear();
    mspMapPoints.clear();
    mspKeyFrames.clear();

    mPlaneIndex.clear();
    mMarkerIndex.clear();
    mPassageIndex.clear();
    mFloorIndex.clear();
    mDoorIndex.clear();
    mKFIndex.clear();
    mRoomWallPlaneIndex.clear();

    skeletonClusterPoints.clear();
    mSkeletonEdges.clear();

    mnMaxKFid        = mnInitKFid;
    mbImuInitialized = false;
    mspDetectedRooms.clear();
    mspMarkerBasedRooms.clear();
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad.store(true, std::memory_order_release);
}

bool Map::IsBad()
{
    return mbBad.load(std::memory_order_acquire);
}

void Map::ApplyScaledRotation(const Sophus::SE3f &T,
                              const float         s,
                              const bool          bScaledVel)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f    Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    const g2o::Sim3 transform_oldWorldToNewWorld(Ryw.cast<double>(),
                                                 tyw.cast<double>(),
                                                 static_cast<double>(s));

    for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin();
         sit != mspKeyFrames.end();
         sit++)
    {
        KeyFrame    *pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw * Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if (!bScaledVel)
            pKF->SetVelocity(Ryw * Vw);
        else
            pKF->SetVelocity(Ryw * Vw * s);
    }

    for (set<MapPoint *>::iterator sit = mspMapPoints.begin();
         sit != mspMapPoints.end();
         sit++)
    {
        MapPoint *pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }

    for (Plane *p_plane : mspPlanes)
    {
        if (p_plane != nullptr && !p_plane->isBad())
        {
            p_plane->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (Marker *p_marker : mspMarkers)
    {
        if (p_marker != nullptr)
        {
            p_marker->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (ORB_SLAM3::Passage *p_passage : mspPassages)
    {
        if (p_passage != nullptr)
        {
            p_passage->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (Room *p_room : mspDetectedRooms)
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_room->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (Room *p_room : mspMarkerBasedRooms)
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_room->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (Floor *p_floor : mspFloors)
    {
        if (p_floor != nullptr)
        {
            p_floor->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (std::vector<Eigen::Vector3d> &cluster_world : skeletonClusterPoints)
    {
        for (Eigen::Vector3d &point_world_m : cluster_world)
        {
            point_world_m = transform_oldWorldToNewWorld.map(point_world_m);
        }
    }

    for (auto &skeletonEdge_world : mSkeletonEdges)
    {
        skeletonEdge_world.first =
            transform_oldWorldToNewWorld.map(skeletonEdge_world.first);
        skeletonEdge_world.second =
            transform_oldWorldToNewWorld.map(skeletonEdge_world.second);
    }

    mnMapChange++;
    mnWorldFrameEpoch++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID)
    {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

std::uint64_t Map::GetWorldFrameEpoch()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnWorldFrameEpoch;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera *> &spCams)
{
    int nMPWithoutObs = 0;

    std::set<MapPoint *> tmp_mspMapPoints1;
    tmp_mspMapPoints1.insert(mspMapPoints.begin(), mspMapPoints.end());

    for (MapPoint *pMPi : tmp_mspMapPoints1)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        if (pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame *, std::tuple<int, int>> mpObs = pMPi->GetObservations();
        for (map<KeyFrame *, std::tuple<int, int>>::iterator it = mpObs.begin(),
                                                             end = mpObs.end();
             it != end;
             ++it)
        {
            if (it->first->GetMap() != this || it->first->isBad())
            {
                pMPi->EraseObservation(it->first);
            }
        }
    }

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.clear();
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for (int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }

    // Backup of MapPoints
    mvpBackupMapPoints.clear();

    std::set<MapPoint *> tmp_mspMapPoints2;
    tmp_mspMapPoints2.insert(mspMapPoints.begin(), mspMapPoints.end());

    for (MapPoint *pMPi : tmp_mspMapPoints2)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames, mspMapPoints);
    }

    // Backup of KeyFrames
    mvpBackupKeyFrames.clear();
    for (KeyFrame *pKFi : mspKeyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames, mspMapPoints, spCams);
    }

    mnBackupKFinitialID = -1;
    if (mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if (mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }
}

void Map::PostLoad(
    KeyFrameDatabase *pKFDB,
    ORBVocabulary
        *pORBVoc /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/,
    map<unsigned int, GeometricCamera *> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(),
              mvpBackupMapPoints.end(),
              std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(),
              mvpBackupKeyFrames.end(),
              std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    map<long unsigned int, MapPoint *> mpMapPointId;
    for (MapPoint *pMPi : mspMapPoints)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    map<long unsigned int, KeyFrame *> mpKeyFrameId;
    for (KeyFrame *pKFi : mspKeyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    // References reconstruction between different instances
    for (MapPoint *pMPi : mspMapPoints)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }

    for (KeyFrame *pKFi : mspKeyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }

    if (mnBackupKFinitialID != -1)
    {
        mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
    }

    if (mnBackupKFlowerID != -1)
    {
        mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    }

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for (int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        mvpKeyFrameOrigins.push_back(
            mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
}

} // namespace ORB_SLAM3
