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

#include "MapStatus.h"
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

    [[nodiscard]] MapStatus addKeyFrame(KeyFrame *p_keyFrame_inout);
    [[nodiscard]] MapStatus addMapPoint(MapPoint *p_mapPoint_in);
    [[nodiscard]] MapStatus addMapPlane(geometric::Plane *p_plane_inout);
    [[nodiscard]] MapStatus addMapMarker(semantic::Marker *p_marker_in);
    [[nodiscard]] MapStatus addDetectedMapRoom(semantic::Room *p_room_in);
    [[nodiscard]] MapStatus addCandidateMapRoom(semantic::Room *p_room_in);
    /*! Atomically moves a room from candidate to detected storage. */
    [[nodiscard]] MapStatus promoteCandidateMapRoom(semantic::Room *p_room_in);
    [[nodiscard]] MapStatus
        addMapFloor(vs_graphs::core::semantic::Floor *p_floor_inout);
    [[nodiscard]] MapStatus addMapDoor(vs_graphs::core::Door *p_door_in);
    [[nodiscard]] MapStatus
        addRoomWallPlane(vs_graphs::core::geometric::Plane *p_plane_in);
    [[nodiscard]] MapStatus
        addMapPassage(vs_graphs::core::semantic::Passage *p_passage_inout);

    /*!
     * @brief Reserves a plane identifier that will not be reused by this map.
     *
     * @param[out] planeId_out Unique plane identifier for a subsequently added
     * plane.
     * @return MAP_STATUS_SUCCESS.
     */
    [[nodiscard]] MapStatus reservePlaneId(int &planeId_out);

    /*!
     * @brief Reserves a floor identifier that will not be reused by this map.
     *
     * @param[out] floorId_out Unique floor identifier for a subsequently added
     * floor.
     * @return MAP_STATUS_SUCCESS.
     */
    [[nodiscard]] MapStatus reserveFloorId(int &floorId_out);

    [[nodiscard]] MapStatus eraseMapPoint(MapPoint *p_mapPoint_in);
    [[nodiscard]] MapStatus eraseKeyFrame(KeyFrame *p_keyFrame_inout);
    [[nodiscard]] MapStatus eraseMapPlane(geometric::Plane *p_plane_in);
    [[nodiscard]] MapStatus eraseMapMarker(semantic::Marker *p_marker_in);
    [[nodiscard]] MapStatus eraseDetectedMapRoom(semantic::Room *p_room_in);
    [[nodiscard]] MapStatus eraseMarkerBasedMapRoom(semantic::Room *p_room_in);
    [[nodiscard]] MapStatus
        eraseRoomWallPlane(vs_graphs::core::geometric::Plane *p_plane_in);

    /*!
     * @brief Records the room this map started with (bootstrap entry room).
     *        Set once; non-owning, owned by this map.
     */
    [[nodiscard]] MapStatus
        setStartingRoom(vs_graphs::core::semantic::Room *p_room_in);

    /*! Returns the room this map started with, if any. */
    [[nodiscard]] MapStatus
        getStartingRoom(semantic::Room *&p_startingRoom_out);

    /*!
     * @brief Records the last current room at departure (reset/export).
     *        Set on map transitions; non-owning, owned by this map.
     */
    [[nodiscard]] MapStatus
        setFinalRoom(vs_graphs::core::semantic::Room *p_room_in);

    /*! Returns the last current room at departure, if any. */
    [[nodiscard]] MapStatus getFinalRoom(semantic::Room *&p_finalRoom_out);

    /*!
     * @brief Links the next map in the mission chain. Left null on
     *        same-map clears. Non-owning; valid only while the Atlas
     *        retains both maps.
     */
    [[nodiscard]] MapStatus setFollowingMap(Map *p_map_in);

    /*! Returns the next map in the mission chain, if any. */
    [[nodiscard]] MapStatus getFollowingMap(Map *&p_followingMap_out);

    [[nodiscard]] MapStatus
        eraseMapPassage(vs_graphs::core::semantic::Passage *p_passage_in);
    [[nodiscard]] MapStatus
        eraseMapFloor(vs_graphs::core::semantic::Floor *p_floor_in);

    /*! Clears lookup-only state after every indexed entity was transferred. */
    [[nodiscard]] MapStatus clearTransferredEntityIndexes();

    [[nodiscard]] MapStatus informNewBigChange();
    [[nodiscard]] MapStatus getLastBigChangeIndex(int &lastBigChangeIndex_out);
    [[nodiscard]] MapStatus
        setReferenceMapPoints(const std::vector<MapPoint *> &mapPoints_in);

    [[nodiscard]] MapStatus
        getAllRooms(std::vector<semantic::Room *> &allRooms_out);
    [[nodiscard]] MapStatus
        getAllPlanes(std::vector<geometric::Plane *> &allPlanes_out);
    [[nodiscard]] MapStatus
        getAllMarkers(std::vector<semantic::Marker *> &allMarkers_out);
    [[nodiscard]] MapStatus
        getAllKeyFrames(std::vector<KeyFrame *> &allKeyFrames_out);
    [[nodiscard]] MapStatus
        getAllMapPoints(std::vector<MapPoint *> &allMapPoints_out);
    [[nodiscard]] MapStatus getAllDetectedMapRooms(
        std::vector<semantic::Room *> &allDetectedMapRooms_out);
    [[nodiscard]] MapStatus getAllDoors(std::vector<Door *> &allDoors_out);
    [[nodiscard]] MapStatus
        getAllFloors(std::vector<semantic::Floor *> &allFloors_out);
    [[nodiscard]] MapStatus getAllMarkerBasedMapRooms(
        std::vector<semantic::Room *> &allMarkerBasedMapRooms_out);
    [[nodiscard]] MapStatus getAllCandidateMapRooms(
        std::vector<semantic::Room *> &allCandidateMapRooms_out);
    [[nodiscard]] MapStatus
        getReferenceMapPoints(std::vector<MapPoint *> &referenceMapPoints_out);
    [[nodiscard]] MapStatus getAllPassages(
        std::vector<vs_graphs::core::semantic::Passage *> &allPassages_out);

    /*!
     * @brief Get the cluster points of the map set by `voxblox_skeleton`
     */
    [[nodiscard]] MapStatus getSkeletonClusterPoints(
        std::vector<std::vector<Eigen::Vector3d>> &skeletonClusterPoints_out);

    /*!
     * @brief       Set the cluster points of the map set by `voxblox_skeleton`
     *
     * @param[in]   newClusterPoints_in
     *              The new cluster points to set
     */
    [[nodiscard]] MapStatus setSkeletonClusterPoints(
        const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints_in);

    /*!
     * @brief       Gets the latest connected Voxblox skeleton edges.
     */
    [[nodiscard]] MapStatus getSkeletonEdges(
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            &skeletonEdges_out);

    /*!
     * @brief       Stores the latest connected Voxblox skeleton edges.
     */
    [[nodiscard]] MapStatus setSkeletonEdges(
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            &newSkeletonEdges_in);

    [[nodiscard]] MapStatus getKeyFrameCount(unsigned long &keyFrameCount_out);
    [[nodiscard]] MapStatus getMarkerCount(unsigned long &markerCount_out);
    [[nodiscard]] MapStatus getMapPointCount(unsigned long &mapPointCount_out);

    [[nodiscard]] MapStatus getId(unsigned long &id_out);
    [[nodiscard]] MapStatus getMaxKeyFrameId(unsigned long &maxKeyFrameId_out);
    [[nodiscard]] MapStatus
        getInitKeyFrameId(unsigned long &initKeyFrameId_out);
    [[nodiscard]] MapStatus setInitKeyFrameId(long unsigned int initialKFif_in);

    [[nodiscard]] MapStatus getOriginKeyFrame(KeyFrame *&p_originKeyFrame_out);
    [[nodiscard]] MapStatus getFloorById(int               floorId_in,
                                         semantic::Floor *&p_floorById_out);
    [[nodiscard]] MapStatus getDoorById(int doorId_in, Door *&p_doorById_out);
    [[nodiscard]] MapStatus getPlaneById(int                planeId_in,
                                         geometric::Plane *&p_planeById_out);
    [[nodiscard]] MapStatus getMarkerById(int                markerId_in,
                                          semantic::Marker *&p_markerById_out);
    [[nodiscard]] MapStatus getKeyFrameById(long unsigned int idCount_in,
                                            KeyFrame *&p_keyFrameById_out);
    [[nodiscard]] MapStatus
                            getPassageById(int                                  passageId_in,
                                           vs_graphs::core::semantic::Passage *&p_passageById_out);
    [[nodiscard]] MapStatus getRoomWallPlaneById(
        int                                 planeId_in,
        vs_graphs::core::geometric::Plane *&p_roomWallPlaneById_out);

    [[nodiscard]] MapStatus
        getBiggestGroundPlane(geometric::Plane *&p_biggestGroundPlane_out);

    [[nodiscard]] MapStatus setStoredMap();
    [[nodiscard]] MapStatus setCurrentMap();

    [[nodiscard]] MapStatus isInUse(bool &isInUse_out);

    [[nodiscard]] MapStatus isBad(bool &isBad_out);
    [[nodiscard]] MapStatus setBad();

    [[nodiscard]] MapStatus clear();

    [[nodiscard]] MapStatus getLastMapChange(int &lastMapChange_out);
    [[nodiscard]] MapStatus getMapChangeIndex(int &mapChangeIndex_out);
    /*! Returns the epoch of the coordinate frame containing this map. */
    [[nodiscard]] MapStatus
        getWorldFrameEpoch(std::uint64_t &worldFrameEpoch_out);
    [[nodiscard]] MapStatus increaseChangeIndex();
    [[nodiscard]] MapStatus setLastMapChange(int currentChangeId_in);

    [[nodiscard]] MapStatus isImuInitialized(bool &isImuInitialized_out);
    [[nodiscard]] MapStatus setImuInitialized();

    [[nodiscard]] MapStatus
        applyScaledRotation(const Sophus::SE3f &T_in,
                            const float         s_in,
                            const bool          isScaledVelocity_in = false);

    [[nodiscard]] MapStatus isInertial(bool &isInertial_out);
    [[nodiscard]] MapStatus setInertialBA1();
    [[nodiscard]] MapStatus setInertialBA2();
    [[nodiscard]] MapStatus getInertialBA1(bool &inertialBA1_out);
    [[nodiscard]] MapStatus getInertialBA2(bool &inertialBA2_out);
    [[nodiscard]] MapStatus setInertialSensor();

    [[nodiscard]] MapStatus changeId(long unsigned int idCount_in);

    [[nodiscard]] MapStatus
        getLowerKeyFrameId(unsigned int &lowerKeyFrameId_out);

    [[nodiscard]] MapStatus
        preSave(std::set<camera_models::geometriccamera::GeometricCamera *>
                    &cams_inout);
    [[nodiscard]] MapStatus
        postLoad(KeyFrameDatabase *p_keyFrameDatabase_inout,
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
