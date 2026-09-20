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

#include "Map.h"

#include <algorithm>
#include <iterator>
#include <mutex>

namespace vs_graphs
{
namespace core
{

long unsigned int Map::nNextId = 0;

Map::Map() :
    p_firstRegionKeyFrame(static_cast<KeyFrame *>(nullptr)),
    fail(false),
    hasImuInitialization(false),
    mapChange(0),
    mapChangeNotified(0),
    worldFrameEpoch(0U),
    maxKeyFrameId(0),
    bigChangeIndex(0),
    inUse(false),
    hasThumbnail(false),
    mbBad(false),
    isInertialMode(false),
    hasInertialBA1(false),
    hasInertialBA2(false)
{
    mnId        = nNextId++;
    p_thumbnail = static_cast<GLubyte *>(nullptr);
}

Map::Map(int initKFid) :
    p_firstRegionKeyFrame(static_cast<KeyFrame *>(nullptr)),
    fail(false),
    hasImuInitialization(false),
    mapChange(0),
    mapChangeNotified(0),
    worldFrameEpoch(0U),
    initKeyFrameId(initKFid),
    maxKeyFrameId(initKFid),
    bigChangeIndex(0),
    inUse(false),
    hasThumbnail(false),
    mbBad(false),
    isInertialMode(false),
    hasInertialBA1(false),
    hasInertialBA2(false)
{
    mnId        = nNextId++;
    p_thumbnail = static_cast<GLubyte *>(nullptr);
}

Map::~Map()
{
    // NOTE: map elements are intentionally not freed here; the destructor
    // only drops the set references (ownership lives in the atlas/optimizer).
    mapPoints.clear();
    keyFrames.clear();

    // Erase all markers from memory
    markers.clear();

    // Erase all semantic entities from memory
    floors.clear();
    doors.clear();
    planes.clear();
    passages.clear();
    detectedRooms.clear();
    markerBasedRooms.clear();

    if (p_thumbnail)
        delete p_thumbnail;
    p_thumbnail = static_cast<GLubyte *>(nullptr);

    referenceMapPoints.clear();
    keyFrameOrigins.clear();
}

void Map::addKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);

    // First keyframe seeds the map (origin and lowest-id keyframe references);
    // later keyframes are inserted with no id-duplicate check.
    if (keyFrames.empty())
    {
        std::cout << "\n[Mapping] Map initialized with initial KeyFrame #"
                  << initKeyFrameId << "." << std::endl;
        initKeyFrameId    = pKF->mnId;
        p_initialKeyFrame = pKF;
        p_lowerIdKeyFrame = pKF;
    }

    // Add the KeyFrame to the map
    keyFrames.insert(pKF);

    // Update the maximum KeyFrame id
    if (pKF->mnId > maxKeyFrameId)
        maxKeyFrameId = pKF->mnId;

    if (pKF->mnId < p_lowerIdKeyFrame->mnId)
        p_lowerIdKeyFrame = pKF;

    keyFrameIndex[pKF->mnId] = pKF;
}

void Map::addMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mapPoints.insert(pMP);
}

void Map::addMapMarker(semantic::Marker *pMarker)
{
    unique_lock<mutex> lock(mMutexMap);
    markers.insert(pMarker);
    // Add the marker to the hashmap
    markerIndex[pMarker->getId()] = pMarker;
}

void Map::addMapPlane(geometric::Plane *pPlane)
{
    if (pPlane == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexMap);

    for (auto planeIterator = planeIndex.begin();
         planeIterator != planeIndex.end();)
    {
        planeIterator = planeIterator->second == pPlane &&
                                planeIterator->first != pPlane->getId()
                            ? planeIndex.erase(planeIterator)
                            : std::next(planeIterator);
    }

    const auto existingPlaneIterator = planeIndex.find(pPlane->getId());

    if (pPlane->getId() < 0 || (existingPlaneIterator != planeIndex.end() &&
                                existingPlaneIterator->second != pPlane))
    {
        while (planeIndex.count(nextAvailablePlaneId) > 0)
        {
            ++nextAvailablePlaneId;
        }

        const int replacementPlaneId = nextAvailablePlaneId++;

        std::cerr << "[Map] geometric::Plane ID collision for "
                  << pPlane->getId() << "; reassigned to " << replacementPlaneId
                  << "." << std::endl;

        pPlane->setId(replacementPlaneId);
    }
    else
    {
        nextAvailablePlaneId =
            std::max(nextAvailablePlaneId, pPlane->getId() + 1);
    }

    planes.insert(pPlane);
    planeIndex.insert_or_assign(pPlane->getId(), pPlane);
}

void Map::addRoomWallPlane(vs_graphs::core::geometric::Plane *pPlane)
{
    unique_lock<mutex> lock(mMutexMap);
    // Add the plane to the hashmap
    roomWallPlaneIndex[pPlane->getId()] = pPlane;
}

void vs_graphs::core::Map::addMapPassage(
    vs_graphs::core::semantic::Passage *pPassage)
{
    if (pPassage == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexMap);

    const auto existingPassage = passageIndex.find(pPassage->getId());
    if (pPassage->getId() < 0 || (existingPassage != passageIndex.end() &&
                                  existingPassage->second != pPassage))
    {
        std::cerr << "[Map] semantic::Passage ID collision for "
                  << pPassage->getId()
                  << "; caller must resolve it before destination insertion."
                  << std::endl;
        return;
    }

    passages.insert(pPassage);
    passageIndex.insert_or_assign(pPassage->getId(), pPassage);
}

void Map::addDetectedMapRoom(semantic::Room *pRoom)
{
    unique_lock<mutex> lock(mMutexMap);
    detectedRooms.insert(pRoom);
}

void Map::addCandidateMapRoom(semantic::Room *pRoom)
{
    unique_lock<mutex> lock(mMutexMap);
    markerBasedRooms.insert(pRoom);
}

void Map::promoteCandidateMapRoom(semantic::Room *pRoom)
{
    if (pRoom == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexMap);
    markerBasedRooms.erase(pRoom);
    detectedRooms.insert(pRoom);
}

void Map::addMapFloor(semantic::Floor *pFloor)
{
    if (pFloor == nullptr)
    {
        return;
    }

    unique_lock<mutex> lock(mMutexMap);

    for (auto floorIterator = floorIndex.begin();
         floorIterator != floorIndex.end();)
    {
        floorIterator = floorIterator->second == pFloor &&
                                floorIterator->first != pFloor->getId()
                            ? floorIndex.erase(floorIterator)
                            : std::next(floorIterator);
    }

    const auto existingFloorIterator = floorIndex.find(pFloor->getId());

    if (pFloor->getId() < 0 || (existingFloorIterator != floorIndex.end() &&
                                existingFloorIterator->second != pFloor))
    {
        while (floorIndex.count(nextAvailableFloorId) > 0)
        {
            ++nextAvailableFloorId;
        }

        const int replacementFloorId = nextAvailableFloorId++;

        std::cerr << "[Map] semantic::Floor ID collision for "
                  << pFloor->getId() << "; reassigned to " << replacementFloorId
                  << "." << std::endl;

        pFloor->setId(replacementFloorId);
    }
    else
    {
        nextAvailableFloorId =
            std::max(nextAvailableFloorId, pFloor->getId() + 1);
    }

    floors.insert(pFloor);
    floorIndex.insert_or_assign(pFloor->getId(), pFloor);
}

int Map::reservePlaneId(void)
{
    unique_lock<mutex> lock(mMutexMap);

    while (planeIndex.count(nextAvailablePlaneId) > 0)
    {
        ++nextAvailablePlaneId;
    }

    return nextAvailablePlaneId++;
}

int Map::reserveFloorId(void)
{
    unique_lock<mutex> lock(mMutexMap);

    while (floorIndex.count(nextAvailableFloorId) > 0)
    {
        ++nextAvailableFloorId;
    }

    return nextAvailableFloorId++;
}

void Map::addMapDoor(Door *pDoor)
{
    unique_lock<mutex> lock(mMutexMap);
    doors.insert(pDoor);
}

KeyFrame *Map::getKeyFrameById(long unsigned int mnId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         keyFrameIterator = keyFrameIndex.find(mnId);
    return keyFrameIterator != keyFrameIndex.end() ? keyFrameIterator->second
                                                   : nullptr;
}

vs_graphs::core::semantic::Passage *Map::getPassageById(int passageId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         passageIterator = passageIndex.find(passageId);
    return passageIterator != passageIndex.end() ? passageIterator->second
                                                 : nullptr;
}

geometric::Plane *Map::getPlaneById(int planeId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         planeIterator = planeIndex.find(planeId);
    return planeIterator != planeIndex.end() ? planeIterator->second : nullptr;
}

vs_graphs::core::geometric::Plane *Map::getRoomWallPlaneById(int planeId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         wallIterator = roomWallPlaneIndex.find(planeId);
    return wallIterator != roomWallPlaneIndex.end() ? wallIterator->second
                                                    : nullptr;
}

semantic::Marker *Map::getMarkerById(int markerId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         markerIterator = markerIndex.find(markerId);
    return markerIterator != markerIndex.end() ? markerIterator->second
                                               : nullptr;
}

semantic::Floor *Map::getFloorById(int floorId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         floorIterator = floorIndex.find(floorId);
    return floorIterator != floorIndex.end() ? floorIterator->second : nullptr;
}

Door *Map::getDoorById(int doorId)
{
    unique_lock<mutex> lock(mMutexMap);
    const auto         doorIterator = doorIndex.find(doorId);
    return doorIterator != doorIndex.end() ? doorIterator->second : nullptr;
}

void Map::setImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    hasImuInitialization = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return hasImuInitialization;
}

void Map::eraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mapPoints.erase(pMP);
    referenceMapPoints.erase(
        std::remove(referenceMapPoints.begin(), referenceMapPoints.end(), pMP),
        referenceMapPoints.end());

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::eraseMapMarker(semantic::Marker *pMarker)
{
    unique_lock<mutex> lock(mMutexMap);
    markers.erase(pMarker);

    for (auto markerIterator = markerIndex.begin();
         markerIterator != markerIndex.end();)
    {
        markerIterator = markerIterator->second == pMarker
                             ? markerIndex.erase(markerIterator)
                             : std::next(markerIterator);
    }
}

void Map::eraseMapPlane(geometric::Plane *pPlane)
{
    unique_lock<mutex> lock(mMutexMap);
    planes.erase(pPlane);

    for (auto planeIterator = planeIndex.begin();
         planeIterator != planeIndex.end();)
    {
        planeIterator = planeIterator->second == pPlane
                            ? planeIndex.erase(planeIterator)
                            : std::next(planeIterator);
    }
}

void Map::eraseRoomWallPlane(vs_graphs::core::geometric::Plane *pPlane)
{
    unique_lock<mutex> lock(mMutexMap);

    for (auto wallIterator = roomWallPlaneIndex.begin();
         wallIterator != roomWallPlaneIndex.end();)
    {
        wallIterator = wallIterator->second == pPlane
                           ? roomWallPlaneIndex.erase(wallIterator)
                           : std::next(wallIterator);
    }
}

void Map::eraseMapPassage(vs_graphs::core::semantic::Passage *pPassage)
{
    unique_lock<mutex> lock(mMutexMap);
    passages.erase(pPassage);

    for (auto passageIterator = passageIndex.begin();
         passageIterator != passageIndex.end();)
    {
        passageIterator = passageIterator->second == pPassage
                              ? passageIndex.erase(passageIterator)
                              : std::next(passageIterator);
    }
}

void Map::eraseMapFloor(vs_graphs::core::semantic::Floor *p_floor_in)
{
    unique_lock<mutex> lock(mMutexMap);
    floors.erase(p_floor_in);

    for (auto floorIterator = floorIndex.begin();
         floorIterator != floorIndex.end();)
    {
        floorIterator = floorIterator->second == p_floor_in
                            ? floorIndex.erase(floorIterator)
                            : std::next(floorIterator);
    }
}

void Map::clearTransferredEntityIndexes()
{
    unique_lock<mutex> lock(mMutexMap);
    floorIndex.clear();
    planeIndex.clear();
    markerIndex.clear();
    keyFrameIndex.clear();
    passageIndex.clear();
    roomWallPlaneIndex.clear();
}

void Map::eraseDetectedMapRoom(semantic::Room *pRoom)
{
    unique_lock<mutex> lock(mMutexMap);
    detectedRooms.erase(pRoom);
}

void Map::eraseMarkerBasedMapRoom(semantic::Room *pRoom)
{
    unique_lock<mutex> lock(mMutexMap);
    markerBasedRooms.erase(pRoom);
}

void Map::eraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    keyFrames.erase(pKF);
    keyFrameIndex.erase(pKF->mnId);
    keyFrameOrigins.erase(
        std::remove(keyFrameOrigins.begin(), keyFrameOrigins.end(), pKF),
        keyFrameOrigins.end());

    if (p_firstRegionKeyFrame == pKF)
    {
        p_firstRegionKeyFrame = nullptr;
    }

    if (p_initialKeyFrame == pKF)
    {
        p_initialKeyFrame = nullptr;
    }

    if (keyFrames.size() > 0)
    {
        if (pKF->mnId == p_lowerIdKeyFrame->mnId)
        {
            vector<KeyFrame *> vpKFs =
                vector<KeyFrame *>(keyFrames.begin(), keyFrames.end());
            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
            p_lowerIdKeyFrame = vpKFs[0];
        }

        if (p_initialKeyFrame == nullptr)
        {
            p_initialKeyFrame = p_lowerIdKeyFrame;
        }
    }
    else
    {
        p_lowerIdKeyFrame = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::setReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    referenceMapPoints = vpMPs;
}

void Map::setStartingRoom(semantic::Room *p_room_in)
{
    unique_lock<mutex> lock(mMutexMap);
    p_startingRoom = p_room_in;
}

semantic::Room *Map::getStartingRoom()
{
    unique_lock<mutex> lock(mMutexMap);
    return p_startingRoom;
}

void Map::setFinalRoom(semantic::Room *p_room_in)
{
    unique_lock<mutex> lock(mMutexMap);
    p_finalRoom = p_room_in;
}

semantic::Room *Map::getFinalRoom()
{
    unique_lock<mutex> lock(mMutexMap);
    return p_finalRoom;
}

void Map::setFollowingMap(Map *p_map_in)
{
    unique_lock<mutex> lock(mMutexMap);
    p_followingMap = p_map_in;
}

Map *Map::getFollowingMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return p_followingMap;
}

void Map::informNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    bigChangeIndex++;
}

int Map::getLastBigChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return bigChangeIndex;
}

std::vector<std::vector<Eigen::Vector3d>> Map::getSkeletonClusterPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return skeletonClusterPoints;
}

void Map::setSkeletonClusterPoints(
    const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints)
{
    unique_lock<mutex> lock(mMutexMap);
    skeletonClusterPoints = newClusterPoints;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
    Map::getSkeletonEdges(void)
{
    /* Lock access to the map data */
    unique_lock<mutex> lock(mMutexMap);

    /* Return a copy of the latest connected skeleton edges */
    return skeletonEdges;
}

void Map::setSkeletonEdges(
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        &newSkeletonEdges)
{
    /* Lock access to the map data */
    unique_lock<mutex> lock(mMutexMap);

    /* Replace the previous connected skeleton edge collection */
    skeletonEdges = newSkeletonEdges;
}

vector<KeyFrame *> Map::getAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame *>(keyFrames.begin(), keyFrames.end());
}

vector<MapPoint *> Map::getAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint *>(mapPoints.begin(), mapPoints.end());
}

vector<semantic::Marker *> Map::getAllMarkers()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<semantic::Marker *>(markers.begin(), markers.end());
}

vector<geometric::Plane *> Map::getAllPlanes()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<geometric::Plane *>(planes.begin(), planes.end());
}

geometric::Plane *Map::getBiggestGroundPlane()
{
    geometric::Plane                         *bestGroundPlane = nullptr;
    std::tuple<std::size_t, std::size_t, int> bestEvidence{0U, 0U, 0};
    bool                                      hasBestEvidence = false;

    for (geometric::Plane *pPlane : getAllPlanes())
    {
        if (pPlane == nullptr || pPlane->isBad() ||
            pPlane->getPlaneType() != geometric::Plane::PlaneVariant::GROUND)
        {
            continue;
        }

        const geometric::Plane::GeometrySnapshot geometry =
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

        const auto evidence = std::make_tuple(geometry.finiteSupportCount,
                                              geometry.observationCount,
                                              -pPlane->getId());
        if (!hasBestEvidence || evidence > bestEvidence)
        {
            bestEvidence    = evidence;
            bestGroundPlane = pPlane;
            hasBestEvidence = true;
        }
    }
    return bestGroundPlane;
}

std::vector<vs_graphs::core::semantic::Passage *> Map::getAllPassages()
{
    unique_lock<mutex> lock(mMutexMap);
    return std::vector<vs_graphs::core::semantic::Passage *>(passages.begin(),
                                                             passages.end());
}

vector<semantic::Room *> Map::getAllRooms()
{
    unique_lock<mutex>       lock(mMutexMap);
    vector<semantic::Room *> allRooms;
    allRooms.insert(allRooms.end(), detectedRooms.begin(), detectedRooms.end());
    allRooms.insert(allRooms.end(),
                    markerBasedRooms.begin(),
                    markerBasedRooms.end());
    return allRooms;
}

vector<semantic::Room *> Map::getAllDetectedMapRooms()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<semantic::Room *>(detectedRooms.begin(), detectedRooms.end());
}

vector<semantic::Room *> Map::getAllMarkerBasedMapRooms()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<semantic::Room *>(markerBasedRooms.begin(),
                                    markerBasedRooms.end());
}

vector<semantic::Room *> Map::getAllCandidateMapRooms()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<semantic::Room *>(markerBasedRooms.begin(),
                                    markerBasedRooms.end());
}

vector<semantic::Floor *> Map::getAllFloors()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<semantic::Floor *>(floors.begin(), floors.end());
}

vector<Door *> Map::getAllDoors()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<Door *>(doors.begin(), doors.end());
}

long unsigned int Map::getMapPointCount()
{
    unique_lock<mutex> lock(mMutexMap);
    return mapPoints.size();
}

long unsigned int Map::getMarkerCount()
{
    unique_lock<mutex> lock(mMutexMap);
    return markers.size();
}

long unsigned int Map::getKeyFrameCount()
{
    unique_lock<mutex> lock(mMutexMap);
    return keyFrames.size();
}

vector<MapPoint *> Map::getReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return referenceMapPoints;
}

long unsigned int Map::getId()
{
    return mnId;
}

long unsigned int Map::getInitKeyFrameId()
{
    unique_lock<mutex> lock(mMutexMap);
    return initKeyFrameId;
}

void Map::setInitKeyFrameId(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    initKeyFrameId = initKFif;
}

long unsigned int Map::getMaxKeyFrameId()
{
    unique_lock<mutex> lock(mMutexMap);
    return maxKeyFrameId;
}

KeyFrame *Map::getOriginKeyFrame()
{
    return p_initialKeyFrame;
}

void Map::setCurrentMap()
{
    inUse = true;
}

void Map::setStoredMap()
{
    inUse = false;
}

void Map::clear()
{
    for (set<KeyFrame *>::iterator sit  = keyFrames.begin(),
                                   send = keyFrames.end();
         sit != send;
         sit++)
    {
        KeyFrame *pKF = *sit;
        pKF->updateMap(static_cast<Map *>(nullptr));
    }

    planes.clear();
    markers.clear();
    passages.clear();
    floors.clear();
    doors.clear();
    mapPoints.clear();
    keyFrames.clear();

    planeIndex.clear();
    markerIndex.clear();
    passageIndex.clear();
    floorIndex.clear();
    doorIndex.clear();
    keyFrameIndex.clear();
    roomWallPlaneIndex.clear();

    skeletonClusterPoints.clear();
    skeletonEdges.clear();

    maxKeyFrameId        = initKeyFrameId;
    hasImuInitialization = false;
    detectedRooms.clear();
    markerBasedRooms.clear();
    referenceMapPoints.clear();
    keyFrameOrigins.clear();
    hasInertialBA1 = false;
    hasInertialBA2 = false;
}

bool Map::isInUse()
{
    return inUse;
}

void Map::setBad()
{
    mbBad.store(true, std::memory_order_release);
}

bool Map::isBad()
{
    return mbBad.load(std::memory_order_acquire);
}

void Map::applyScaledRotation(const Sophus::SE3f &T,
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

    for (set<KeyFrame *>::iterator sit = keyFrames.begin();
         sit != keyFrames.end();
         sit++)
    {
        KeyFrame    *pKF = *sit;
        Sophus::SE3f Twc = pKF->getPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw * Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->setPose(Tcy);
        Eigen::Vector3f Vw = pKF->getVelocity();
        if (!bScaledVel)
            pKF->setVelocity(Ryw * Vw);
        else
            pKF->setVelocity(Ryw * Vw * s);
    }

    for (set<MapPoint *>::iterator sit = mapPoints.begin();
         sit != mapPoints.end();
         sit++)
    {
        MapPoint *pMP = *sit;
        pMP->setWorldPos(s * Ryw * pMP->getWorldPos() + tyw);
        pMP->updateNormalAndDepth();
    }

    for (geometric::Plane *p_plane : planes)
    {
        if (p_plane != nullptr && !p_plane->isBad())
        {
            p_plane->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (semantic::Marker *p_marker : markers)
    {
        if (p_marker != nullptr)
        {
            p_marker->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (vs_graphs::core::semantic::Passage *p_passage : passages)
    {
        if (p_passage != nullptr)
        {
            p_passage->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (semantic::Room *p_room : detectedRooms)
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_room->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (semantic::Room *p_room : markerBasedRooms)
    {
        if (p_room != nullptr && !p_room->isBad())
        {
            p_room->applyTransform(transform_oldWorldToNewWorld);
        }
    }

    for (semantic::Floor *p_floor : floors)
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

    for (auto &skeletonEdge_world : skeletonEdges)
    {
        skeletonEdge_world.first =
            transform_oldWorldToNewWorld.map(skeletonEdge_world.first);
        skeletonEdge_world.second =
            transform_oldWorldToNewWorld.map(skeletonEdge_world.second);
    }

    mapChange++;
    worldFrameEpoch++;
}

void Map::setInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    isInertialMode = true;
}

bool Map::isInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return isInertialMode;
}

void Map::setInertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    hasInertialBA1 = true;
}

void Map::setInertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    hasInertialBA2 = true;
}

bool Map::getInertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return hasInertialBA1;
}

bool Map::getInertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return hasInertialBA2;
}

void Map::changeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::getLowerKeyFrameId()
{
    unique_lock<mutex> lock(mMutexMap);
    if (p_lowerIdKeyFrame)
    {
        return p_lowerIdKeyFrame->mnId;
    }
    return 0;
}

int Map::getMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mapChange;
}

std::uint64_t Map::getWorldFrameEpoch()
{
    unique_lock<mutex> lock(mMutexMap);
    return worldFrameEpoch;
}

void Map::increaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mapChange++;
}

int Map::getLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mapChangeNotified;
}

void Map::setLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<camera_models::GeometricCamera *> &spCams)
{
    int nMPWithoutObs = 0;

    std::set<MapPoint *> tmp_mspMapPoints1;
    tmp_mspMapPoints1.insert(mapPoints.begin(), mapPoints.end());

    for (MapPoint *pMPi : tmp_mspMapPoints1)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        if (pMPi->getObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame *, std::tuple<int, int>> observations =
            pMPi->getObservations();
        for (map<KeyFrame *, std::tuple<int, int>>::iterator
                 it  = observations.begin(),
                 end = observations.end();
             it != end;
             ++it)
        {
            if (it->first->getMap() != this || it->first->isBad())
            {
                pMPi->eraseObservation(it->first);
            }
        }
    }

    // Saves the id of KF origins
    backupKeyFrameOriginIds.clear();
    backupKeyFrameOriginIds.reserve(keyFrameOrigins.size());
    for (int i = 0, numEl = keyFrameOrigins.size(); i < numEl; ++i)
    {
        backupKeyFrameOriginIds.push_back(keyFrameOrigins[i]->mnId);
    }

    // Backup of MapPoints
    backupMapPoints.clear();

    std::set<MapPoint *> tmp_mspMapPoints2;
    tmp_mspMapPoints2.insert(mapPoints.begin(), mapPoints.end());

    for (MapPoint *pMPi : tmp_mspMapPoints2)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        backupMapPoints.push_back(pMPi);
        pMPi->PreSave(keyFrames, mapPoints);
    }

    // Backup of KeyFrames
    backupKeyFrames.clear();
    for (KeyFrame *pKFi : keyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        backupKeyFrames.push_back(pKFi);
        pKFi->PreSave(keyFrames, mapPoints, spCams);
    }

    backupInitialKeyFrameId = -1;
    if (p_initialKeyFrame)
    {
        backupInitialKeyFrameId = p_initialKeyFrame->mnId;
    }

    backupLowerKeyFrameId = -1;
    if (p_lowerIdKeyFrame)
    {
        backupLowerKeyFrameId = p_lowerIdKeyFrame->mnId;
    }
}

void Map::PostLoad(
    KeyFrameDatabase *pKFDB,
    ORBVocabulary
        *pORBVoc /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/,
    map<unsigned int, camera_models::GeometricCamera *> &mpCams)
{
    std::copy(backupMapPoints.begin(),
              backupMapPoints.end(),
              std::inserter(mapPoints, mapPoints.begin()));
    std::copy(backupKeyFrames.begin(),
              backupKeyFrames.end(),
              std::inserter(keyFrames, keyFrames.begin()));

    map<long unsigned int, MapPoint *> mpMapPointId;
    for (MapPoint *pMPi : mapPoints)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        pMPi->updateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    map<long unsigned int, KeyFrame *> mpKeyFrameId;
    for (KeyFrame *pKFi : keyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->updateMap(this);
        pKFi->setORBVocabulary(pORBVoc);
        pKFi->setKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    // References reconstruction between different instances
    for (MapPoint *pMPi : mapPoints)
    {
        if (!pMPi || pMPi->isBad())
            continue;

        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }

    for (KeyFrame *pKFi : keyFrames)
    {
        if (!pKFi || pKFi->isBad())
            continue;

        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }

    if (backupInitialKeyFrameId != -1)
    {
        p_initialKeyFrame = mpKeyFrameId[backupInitialKeyFrameId];
    }

    if (backupLowerKeyFrameId != -1)
    {
        p_lowerIdKeyFrame = mpKeyFrameId[backupLowerKeyFrameId];
    }

    keyFrameOrigins.clear();
    keyFrameOrigins.reserve(backupKeyFrameOriginIds.size());
    for (int i = 0; i < backupKeyFrameOriginIds.size(); ++i)
    {
        keyFrameOrigins.push_back(mpKeyFrameId[backupKeyFrameOriginIds[i]]);
    }

    backupMapPoints.clear();
}

} // namespace core
} // namespace vs_graphs
