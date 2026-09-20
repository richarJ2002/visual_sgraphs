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

#ifndef MAP_H
#define MAP_H

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Semantic/Floor.h"
#include "Semantic/Marker.h"
#include "Semantic/Room.h"

#include <atomic>
#include <boost/serialization/base_object.hpp>
#include <mutex>
#include <pangolin/pangolin.h>
#include <set>
#include <unordered_map>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
class Room;
}
class Atlas;
namespace geometric
{
class Plane;
}
class Door;
namespace semantic
{
class Floor;
}
namespace semantic
{
class Marker;
}
namespace semantic
{
class Passage;
}
class MapPoint;
class KeyFrame;
class KeyFrameDatabase;

class Map
{
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & initKeyFrameId;
        ar & maxKeyFrameId;
        ar & bigChangeIndex;

        // Save/load a set structure, the set structure is broken in
        // libboost 1.58 for ubuntu 16.04, a vector is serializated ar &
        // mspKeyFrames; ar & mspMapPoints;
        ar & backupKeyFrames;
        ar & backupMapPoints;

        ar & backupKeyFrameOriginIds;

        ar & backupInitialKeyFrameId;
        ar & backupLowerKeyFrameId;

        ar & hasImuInitialization;
        ar & isInertialMode;
        ar & hasInertialBA1;
        ar & hasInertialBA2;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Map();
    Map(int initKFid);
    ~Map();

    void addKeyFrame(KeyFrame *pKF);
    void addMapPoint(MapPoint *pMP);
    void addMapPlane(geometric::Plane *pPlane);
    void addMapMarker(semantic::Marker *pMarker);
    void addDetectedMapRoom(semantic::Room *room);
    void addCandidateMapRoom(semantic::Room *room);
    /*! Atomically moves a room from candidate to detected storage. */
    void promoteCandidateMapRoom(semantic::Room *room);
    void addMapFloor(vs_graphs::core::semantic::Floor *pFloor);
    void addMapDoor(vs_graphs::core::Door *pDoor);
    void addRoomWallPlane(vs_graphs::core::geometric::Plane *pPlane);
    void addMapPassage(vs_graphs::core::semantic::Passage *pPassage);

    /*!
     * @brief Reserves a plane identifier that will not be reused by this map.
     *
     * @return Unique plane identifier for a subsequently added plane.
     */
    int reservePlaneId(void);

    /*!
     * @brief Reserves a floor identifier that will not be reused by this map.
     *
     * @return Unique floor identifier for a subsequently added floor.
     */
    int reserveFloorId(void);

    void eraseMapPoint(MapPoint *pMP);
    void eraseKeyFrame(KeyFrame *pKF);
    void eraseMapPlane(geometric::Plane *pPlane);
    void eraseMapMarker(semantic::Marker *pMarker);
    void eraseDetectedMapRoom(semantic::Room *pRoom);
    void eraseMarkerBasedMapRoom(semantic::Room *pRoom);
    void eraseRoomWallPlane(vs_graphs::core::geometric::Plane *pPlane);

    /*!
     * @brief Records the room this map started with (bootstrap entry room).
     *        Set once; non-owning, owned by this map.
     */
    void setStartingRoom(vs_graphs::core::semantic::Room *p_room_in);

    /*! Returns the room this map started with, if any. */
    vs_graphs::core::semantic::Room *getStartingRoom();

    /*!
     * @brief Records the last current room at departure (reset/export).
     *        Set on map transitions; non-owning, owned by this map.
     */
    void setFinalRoom(vs_graphs::core::semantic::Room *p_room_in);

    /*! Returns the last current room at departure, if any. */
    vs_graphs::core::semantic::Room *getFinalRoom();

    /*!
     * @brief Links the next map in the mission chain. Left null on
     *        same-map clears. Non-owning; valid only while the Atlas
     *        retains both maps.
     */
    void setFollowingMap(Map *p_map_in);

    /*! Returns the next map in the mission chain, if any. */
    Map *getFollowingMap();

    void eraseMapPassage(vs_graphs::core::semantic::Passage *pPassage);
    void eraseMapFloor(vs_graphs::core::semantic::Floor *p_floor_in);

    /*! Clears lookup-only state after every indexed entity was transferred. */
    void clearTransferredEntityIndexes();

    void informNewBigChange();
    int  getLastBigChangeIndex();
    void setReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

    std::vector<semantic::Room *>                   getAllRooms();
    std::vector<geometric::Plane *>                 getAllPlanes();
    std::vector<semantic::Marker *>                 getAllMarkers();
    std::vector<KeyFrame *>                         getAllKeyFrames();
    std::vector<MapPoint *>                         getAllMapPoints();
    std::vector<semantic::Room *>                   getAllDetectedMapRooms();
    std::vector<vs_graphs::core::Door *>            getAllDoors();
    std::vector<vs_graphs::core::semantic::Floor *> getAllFloors();
    std::vector<semantic::Room *>                   getAllMarkerBasedMapRooms();
    std::vector<semantic::Room *>                   getAllCandidateMapRooms();
    std::vector<MapPoint *>                         getReferenceMapPoints();
    std::vector<vs_graphs::core::semantic::Passage *> getAllPassages();

    /*!
     * @brief Get the cluster points of the map set by `voxblox_skeleton`
     */
    std::vector<std::vector<Eigen::Vector3d>> getSkeletonClusterPoints(void);

    /*!
     * @brief       Set the cluster points of the map set by `voxblox_skeleton`
     *
     * @param[in]   newClusterPoints
     *              The new cluster points to set
     */
    void setSkeletonClusterPoints(
        const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints);

    /*!
     * @brief       Gets the latest connected Voxblox skeleton edges.
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        getSkeletonEdges(void);

    /*!
     * @brief       Stores the latest connected Voxblox skeleton edges.
     */
    void setSkeletonEdges(
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            &newSkeletonEdges);

    long unsigned     getKeyFrameCount();
    long unsigned int getMarkerCount();
    long unsigned int getMapPointCount();

    long unsigned int getId();
    long unsigned int getMaxKeyFrameId();
    long unsigned int getInitKeyFrameId();
    void              setInitKeyFrameId(long unsigned int initKFif);

    KeyFrame                           *getOriginKeyFrame();
    semantic::Floor                    *getFloorById(int floorId);
    Door                               *getDoorById(int doorId);
    geometric::Plane                   *getPlaneById(int planeId);
    semantic::Marker                   *getMarkerById(int markerId);
    KeyFrame                           *getKeyFrameById(long unsigned int mnId);
    vs_graphs::core::semantic::Passage *getPassageById(int passageId);
    vs_graphs::core::geometric::Plane  *getRoomWallPlaneById(int planeId);

    geometric::Plane *getBiggestGroundPlane();

    void setStoredMap();
    void setCurrentMap();

    bool isInUse();

    bool isBad();
    void setBad();

    void clear();

    int           getLastMapChange();
    int           getMapChangeIndex();
    /*! Returns the epoch of the coordinate frame containing this map. */
    std::uint64_t getWorldFrameEpoch();
    void          increaseChangeIndex();
    void          setLastMapChange(int currentChangeId);

    bool isImuInitialized();
    void setImuInitialized();

    void applyScaledRotation(const Sophus::SE3f &T,
                             const float         s,
                             const bool          bScaledVel = false);

    bool isInertial();
    void setInertialBA1();
    void setInertialBA2();
    bool getInertialBA1();
    bool getInertialBA2();
    void setInertialSensor();

    void changeId(long unsigned int nId);

    unsigned int getLowerKeyFrameId();

    void PreSave(std::set<camera_models::GeometricCamera *> &spCams);
    void PostLoad(KeyFrameDatabase                                    *pKFDB,
                  ORBVocabulary                                       *pORBVoc,
                  map<unsigned int, camera_models::GeometricCamera *> &mpCams);

    KeyFrame                 *p_firstRegionKeyFrame;
    std::mutex                mMutexMapUpdate;
    vector<KeyFrame *>        keyFrameOrigins;
    vector<unsigned long int> backupKeyFrameOriginIds;

    // This avoid that two points are created simultaneously in separate threads
    // (id conflict)
    std::mutex mMutexPointCreation;

    bool fail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH  = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    std::set<long unsigned int> optKeyFrameIds;
    std::set<long unsigned int> fixedKeyFrameIds;

  protected:
    long unsigned int mnId;

    std::set<semantic::Floor *>                    floors;
    std::set<Door *>                               doors;
    std::set<geometric::Plane *>                   planes;
    std::set<semantic::Marker *>                   markers;
    std::set<MapPoint *>                           mapPoints;
    std::set<KeyFrame *>                           keyFrames;
    std::set<semantic::Room *>                     detectedRooms;
    std::set<semantic::Room *>                     markerBasedRooms;
    std::set<vs_graphs::core::semantic::Passage *> passages;

    // Skeleton cluster points of the map set by `voxblox_skeleton`
    std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

    /*!
     * @brief       Latest connected Voxblox skeleton graph edges.
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> skeletonEdges;

    // Hashmaps and indices for fetching elements
    std::unordered_map<int, semantic::Floor *>                    floorIndex;
    std::unordered_map<int, Door *>                               doorIndex;
    std::unordered_map<int, geometric::Plane *>                   planeIndex;
    std::unordered_map<int, semantic::Marker *>                   markerIndex;
    std::unordered_map<long unsigned int, KeyFrame *>             keyFrameIndex;
    std::unordered_map<int, vs_graphs::core::semantic::Passage *> passageIndex;
    std::unordered_map<int, vs_graphs::core::geometric::Plane *>
        roomWallPlaneIndex;

    /* Monotonic semantic IDs remain unique after fusion creates index gaps. */
    int nextAvailablePlaneId{0};
    int nextAvailableFloorId{0};

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04,
    // a vector is serializated
    std::vector<MapPoint *> backupMapPoints;
    std::vector<KeyFrame *> backupKeyFrames;

    KeyFrame *p_initialKeyFrame;
    KeyFrame *p_lowerIdKeyFrame;

    /* Mission-chain trace links. Rooms are owned by this map (shared
     * lifetime); the following map is Atlas-owned (see setters). */
    vs_graphs::core::semantic::Room *p_startingRoom{nullptr};
    vs_graphs::core::semantic::Room *p_finalRoom{nullptr};
    Map                             *p_followingMap{nullptr};

    unsigned long int backupLowerKeyFrameId;
    unsigned long int backupInitialKeyFrameId;

    std::vector<MapPoint *> referenceMapPoints;

    bool hasImuInitialization;

    int mapChange;
    int mapChangeNotified;

    /*!
     * Monotonic coordinate-frame epoch. Unlike mnMapChange, ordinary local BA
     * and content updates do not increment it; whole-map rebases do.
     */
    std::uint64_t worldFrameEpoch;

    long unsigned int initKeyFrameId;
    long unsigned int maxKeyFrameId;

    // Index related to a big change in the map (loop closure, global BA)
    int bigChangeIndex;

    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte *p_thumbnail;

    bool             inUse;
    bool             hasThumbnail;
    std::atomic_bool mbBad{false};

    bool isInertialMode;
    bool hasInertialBA1;
    bool hasInertialBA2;

    // Mutex
    std::mutex mMutexMap;
};

} // namespace core
} // namespace vs_graphs

#endif
