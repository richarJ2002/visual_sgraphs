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

#include "ORBVocabulary.h"
#include "Thirdparty/Sophus/sophus/se3.hpp"
#include <Eigen/Core>
#include <atomic>
#include <boost/serialization/access.hpp>
#include <map>
#include <mutex>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace geometriccamera
{
class GeometricCamera;
} // namespace geometriccamera
} // namespace camera_models
} // namespace core
} // namespace vs_graphs

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
    void serialize(Archive &ar, const unsigned int version);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Map();
    Map(int initialKeyFrameId_in);
    ~Map();

    void addKeyFrame(KeyFrame *p_keyFrame_inout);
    void addMapPoint(MapPoint *p_mapPoint_in);
    void addMapPlane(geometric::Plane *p_plane_inout);
    void addMapMarker(semantic::Marker *p_marker_in);
    void addDetectedMapRoom(semantic::Room *p_room_in);
    void addCandidateMapRoom(semantic::Room *p_room_in);
    /*! Atomically moves a room from candidate to detected storage. */
    void promoteCandidateMapRoom(semantic::Room *p_room_in);
    void addMapFloor(vs_graphs::core::semantic::Floor *p_floor_inout);
    void addMapDoor(vs_graphs::core::Door *p_door_in);
    void addRoomWallPlane(vs_graphs::core::geometric::Plane *p_plane_in);
    void addMapPassage(vs_graphs::core::semantic::Passage *p_passage_inout);

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

    void eraseMapPoint(MapPoint *p_mapPoint_in);
    void eraseKeyFrame(KeyFrame *p_keyFrame_inout);
    void eraseMapPlane(geometric::Plane *p_plane_in);
    void eraseMapMarker(semantic::Marker *p_marker_in);
    void eraseDetectedMapRoom(semantic::Room *p_room_in);
    void eraseMarkerBasedMapRoom(semantic::Room *p_room_in);
    void eraseRoomWallPlane(vs_graphs::core::geometric::Plane *p_plane_in);

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

    void eraseMapPassage(vs_graphs::core::semantic::Passage *p_passage_in);
    void eraseMapFloor(vs_graphs::core::semantic::Floor *p_floor_in);

    /*! Clears lookup-only state after every indexed entity was transferred. */
    void clearTransferredEntityIndexes();

    void informNewBigChange();
    int  getLastBigChangeIndex();
    void setReferenceMapPoints(const std::vector<MapPoint *> &mapPoints_in);

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
     * @param[in]   newClusterPoints_in
     *              The new cluster points to set
     */
    void setSkeletonClusterPoints(
        const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints_in);

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
            &newSkeletonEdges_in);

    long unsigned     getKeyFrameCount();
    long unsigned int getMarkerCount();
    long unsigned int getMapPointCount();

    long unsigned int getId();
    long unsigned int getMaxKeyFrameId();
    long unsigned int getInitKeyFrameId();
    void              setInitKeyFrameId(long unsigned int initialKFif_in);

    KeyFrame         *getOriginKeyFrame();
    semantic::Floor  *getFloorById(int floorId_in);
    Door             *getDoorById(int doorId_in);
    geometric::Plane *getPlaneById(int planeId_in);
    semantic::Marker *getMarkerById(int markerId_in);
    KeyFrame         *getKeyFrameById(long unsigned int idCount_in);
    vs_graphs::core::semantic::Passage *getPassageById(int passageId_in);
    vs_graphs::core::geometric::Plane  *getRoomWallPlaneById(int planeId_in);

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
    void          setLastMapChange(int currentChangeId_in);

    bool isImuInitialized();
    void setImuInitialized();

    void applyScaledRotation(const Sophus::SE3f &T_in,
                             const float         s_in,
                             const bool          isScaledVelocity_in = false);

    bool isInertial();
    void setInertialBA1();
    void setInertialBA2();
    bool getInertialBA1();
    bool getInertialBA2();
    void setInertialSensor();

    void changeId(long unsigned int idCount_in);

    unsigned int getLowerKeyFrameId();

    void preSave(std::set<camera_models::geometriccamera::GeometricCamera *>
                     &cams_inout);
    void postLoad(KeyFrameDatabase *p_keyFrameDatabase_inout,
                  ORBVocabulary    *p_orbVocabulary_in,
                  std::map<unsigned int,
                           camera_models::geometriccamera::GeometricCamera *>
                      &cams_inout);

    KeyFrame                      *p_firstRegionKeyFrame;
    std::mutex                     mapUpdateMutex;
    std::vector<KeyFrame *>        keyFrameOrigins;
    std::vector<unsigned long int> backupKeyFrameOriginIds;

    // This avoid that two points are created simultaneously in separate threads
    // (id conflict)
    std::mutex pointCreationMutex;

    bool hasFailed;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH  = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nextId;

    std::set<long unsigned int> optKeyFrameIds;
    std::set<long unsigned int> fixedKeyFrameIds;

  protected:
    long unsigned int id;

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
     * Monotonic coordinate-frame epoch. Unlike mapChange, ordinary local BA
     * and content updates do not increment it; whole-map rebases do.
     */
    std::uint64_t worldFrameEpoch;

    long unsigned int initKeyFrameId;
    long unsigned int maxKeyFrameId;

    // Index related to a big change in the map (loop closure, global BA)
    int bigChangeIndex;

    // View of the map in aerial sight (for the AtlasViewer)
    unsigned char *p_thumbnail;

    bool             isMapInUse;
    bool             hasThumbnail;
    std::atomic_bool isFlaggedBad{false};

    bool isInertialMode;
    bool hasInertialBA1;
    bool hasInertialBA2;

    // Mutex
    std::mutex mapMutex;
};

} // namespace core
} // namespace vs_graphs

#endif
