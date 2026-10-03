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
 * @file            Map.h
 *
 * @brief           Declares Map, one map of the atlas: its key frames, map
 *                  points and the semantic entities (planes, walls, rooms,
 *                  floors, passages, markers) built on them.
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

/*!
 * @brief        One map of the atlas: the key frames, map points and semantic
 *               entities (planes, rooms, floors, doors, markers, passages) that
 *               share one world frame. The pointers it holds are borrowed:
 *               destroying or clearing the map only drops its references. Most
 *               methods take mapMutex, as each one says.
 */
class Map
{
    friend class boost::serialization::access;

    /*!
     * @brief        Boost serialisation hook: reads or writes the map's
     *               persistent fields (map id, key frame id bounds, big change
     *               counter, backup key frame and map point lists, origin ids,
     *               first and lowest-id key frame ids, IMU and inertial flags).
     *               Call preSave() before saving and postLoad() after loading.
     *
     * @param[in,out] ar
     *               Boost archive being written or read.
     * @param[in]    version
     *               Archive format version; unused.
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Map();
    /*!
     * @brief        Creates an empty map that takes the next free map id and
     *               starts counting key frame ids at the given value.
     *
     * @param[in]    initialKeyFrameId_in
     *               Id of the first key frame this map will hold; also the
     *               starting value of the highest key frame id.
     */
    Map(int initialKeyFrameId_in);
    ~Map();
    /*!
     * @brief        Copying is forbidden: the map owns its thumbnail
     *               image and deletes it when destroyed.
     */
    Map(const Map &otherMap_in)            = delete;
    Map &operator=(const Map &otherMap_in) = delete;

    /*!
     * @brief        Adds a key frame to this map and indexes it by its id. The
     *               first key frame added becomes the origin and lowest-id key
     *               frame; later ones update the highest id and the lowest-id
     *               key frame. Takes mapMutex.
     *
     * @param[in,out] p_keyFrame_inout
     *               Key frame to add; borrowed, must not be null. Only its id
     *               is read.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus addKeyFrame(KeyFrame *p_keyFrame_inout);
    /*!
     * @brief        Adds a map point to this map. Takes mapMutex.
     *
     * @param[in]    p_mapPoint_in
     *               Map point to add; borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus addMapPoint(MapPoint *p_mapPoint_in);
    /*!
     * @brief        Adds a plane to this map and indexes it by plane id. If the
     *               plane id is negative or already used by another plane, the
     *               plane is given a fresh id. Does nothing for a null plane.
     *               Takes mapMutex.
     *
     * @param[in,out] p_plane_inout
     *               Plane to add; borrowed. Its id may be changed to resolve a
     *               collision.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus addMapPlane(geometric::Plane *p_plane_inout);
    /*!
     * @brief        Adds a marker to this map and indexes it by marker id.
     *               Takes mapMutex.
     *
     * @param[in]    p_marker_in
     *               Marker to add; borrowed, must not be null.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus addMapMarker(semantic::Marker *p_marker_in);
    /*!
     * @brief        Adds a room to the detected rooms of this map. Takes
     *               mapMutex.
     *
     * @param[in]    p_room_in
     *               Room to add; borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus addDetectedMapRoom(semantic::Room *p_room_in);
    /*!
     * @brief        Adds a room to the candidate (marker-based) rooms of this
     *               map, which are rooms not yet promoted to detected rooms.
     *               Takes mapMutex.
     *
     * @param[in]    p_room_in
     *               Room to add; borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus addCandidateMapRoom(semantic::Room *p_room_in);
    /*! Atomically moves a room from candidate to detected storage. */
    [[nodiscard]] MapStatus promoteCandidateMapRoom(semantic::Room *p_room_in);
    /*!
     * @brief        Adds a floor to this map and indexes it by floor id. If the
     *               floor id is negative or already used by another floor, the
     *               floor is given a fresh id. Does nothing for a null floor.
     *               Takes mapMutex.
     *
     * @param[in,out] p_floor_inout
     *               Floor to add; borrowed. Its id may be changed to resolve a
     *               collision.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        addMapFloor(vs_graphs::core::semantic::Floor *p_floor_inout);
    /*!
     * @brief        Adds a door to this map. The door is not entered in the id
     *               lookup used by getDoorById(). Takes mapMutex.
     *
     * @param[in]    p_door_in
     *               Door to add; borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus addMapDoor(vs_graphs::core::Door *p_door_in);
    /*!
     * @brief        Indexes a wall plane of a room by its plane id, so
     *               getRoomWallPlaneById() can find it. The plane is not added
     *               to the map's plane set. Takes mapMutex.
     *
     * @param[in]    p_plane_in
     *               Wall plane to index; borrowed, must not be null.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        addRoomWallPlane(vs_graphs::core::geometric::Plane *p_plane_in);
    /*!
     * @brief        Adds a passage to this map and indexes it by passage id. A
     *               passage whose id is negative or already used by another
     *               passage is rejected: it is not added, a message is printed
     *               and the status is still success, so the caller must resolve
     *               the id first. Does nothing for a null passage. Takes
     *               mapMutex.
     *
     * @param[in,out] p_passage_inout
     *               Passage to add; borrowed. Only its id is read.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
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

    /*!
     * @brief        Removes a map point from this map and from its reference
     *               map points. The point itself is not deleted. Takes
     *               mapMutex.
     *
     * @param[in]    p_mapPoint_in
     *               Map point to remove.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus eraseMapPoint(MapPoint *p_mapPoint_in);
    /*!
     * @brief        Removes a key frame from this map and from its id index and
     *               origin list, and re-picks the origin and lowest-id key
     *               frame if the removed one held that role. The key frame
     *               itself is not deleted. Takes mapMutex.
     *
     * @param[in,out] p_keyFrame_inout
     *               Key frame to remove, must not be null. Only its id is read.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus eraseKeyFrame(KeyFrame *p_keyFrame_inout);
    /*!
     * @brief        Removes a plane from this map and from the plane id index.
     *               The plane itself is not deleted. Takes mapMutex.
     *
     * @param[in]    p_plane_in
     *               Plane to remove.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus eraseMapPlane(geometric::Plane *p_plane_in);
    /*!
     * @brief        Removes a marker from this map and from the marker id
     *               index. The marker itself is not deleted. Takes mapMutex.
     *
     * @param[in]    p_marker_in
     *               Marker to remove.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus eraseMapMarker(semantic::Marker *p_marker_in);
    /*!
     * @brief        Removes a room from the detected rooms of this map. The
     *               room itself is not deleted. Takes mapMutex.
     *
     * @param[in]    p_room_in
     *               Room to remove.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus eraseDetectedMapRoom(semantic::Room *p_room_in);
    /*!
     * @brief        Removes a room from the candidate (marker-based) rooms of
     *               this map. The room itself is not deleted. Takes mapMutex.
     *
     * @param[in]    p_room_in
     *               Room to remove.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus eraseMarkerBasedMapRoom(semantic::Room *p_room_in);
    /*!
     * @brief        Removes a room wall plane from the wall plane id index. The
     *               plane itself is not deleted. Takes mapMutex.
     *
     * @param[in]    p_plane_in
     *               Wall plane to remove from the index.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
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

    /*!
     * @brief        Removes a passage from this map and from the passage id
     *               index. The passage itself is not deleted. Takes mapMutex.
     *
     * @param[in]    p_passage_in
     *               Passage to remove.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        eraseMapPassage(vs_graphs::core::semantic::Passage *p_passage_in);
    /*!
     * @brief        Removes a floor from this map and from the floor id index.
     *               The floor itself is not deleted. Takes mapMutex.
     *
     * @param[in]    p_floor_in
     *               Floor to remove.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        eraseMapFloor(vs_graphs::core::semantic::Floor *p_floor_in);

    /*! Clears lookup-only state after every indexed entity was transferred. */
    [[nodiscard]] MapStatus clearTransferredEntityIndexes();

    /*!
     * @brief        Counts one more big change of the map (loop closure or
     *               global bundle adjustment). Takes mapMutex.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus informNewBigChange();
    /*!
     * @brief        Returns how many big changes (loop closures, global bundle
     *               adjustments) this map has seen. Takes mapMutex.
     *
     * @param[out]   lastBigChangeIndex_out
     *               Big change counter.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getLastBigChangeIndex(int &lastBigChangeIndex_out);
    /*!
     * @brief        Replaces the reference map points, the local map points the
     *               tracker draws and uses as its current reference. Takes
     *               mapMutex.
     *
     * @param[in]    mapPoints_in
     *               New reference map points, copied; the points are borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        setReferenceMapPoints(const std::vector<MapPoint *> &mapPoints_in);

    /*!
     * @brief        Returns a snapshot copy of all rooms of this map, detected
     *               rooms first and then candidate rooms. Takes mapMutex.
     *
     * @param[out]   allRooms_out
     *               Receives the rooms; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getAllRooms(std::vector<semantic::Room *> &allRooms_out);
    /*!
     * @brief        Returns a snapshot copy of all planes of this map. Takes
     *               mapMutex.
     *
     * @param[out]   allPlanes_out
     *               Receives the planes; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getAllPlanes(std::vector<geometric::Plane *> &allPlanes_out);
    /*!
     * @brief        Returns a snapshot copy of all markers of this map. Takes
     *               mapMutex.
     *
     * @param[out]   allMarkers_out
     *               Receives the markers; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getAllMarkers(std::vector<semantic::Marker *> &allMarkers_out);
    /*!
     * @brief        Returns a snapshot copy of all key frames of this map.
     *               Takes mapMutex.
     *
     * @param[out]   allKeyFrames_out
     *               Receives the key frames; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getAllKeyFrames(std::vector<KeyFrame *> &allKeyFrames_out);
    /*!
     * @brief        Returns a snapshot copy of all map points of this map.
     *               Takes mapMutex.
     *
     * @param[out]   allMapPoints_out
     *               Receives the map points; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getAllMapPoints(std::vector<MapPoint *> &allMapPoints_out);
    /*!
     * @brief        Returns a snapshot copy of the detected rooms of this map.
     *               Takes mapMutex.
     *
     * @param[out]   allDetectedMapRooms_out
     *               Receives the detected rooms; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getAllDetectedMapRooms(
        std::vector<semantic::Room *> &allDetectedMapRooms_out);
    /*!
     * @brief        Returns a snapshot copy of all doors of this map. Takes
     *               mapMutex.
     *
     * @param[out]   allDoors_out
     *               Receives the doors; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getAllDoors(std::vector<Door *> &allDoors_out);
    /*!
     * @brief        Returns a snapshot copy of all floors of this map. Takes
     *               mapMutex.
     *
     * @param[out]   allFloors_out
     *               Receives the floors; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getAllFloors(std::vector<semantic::Floor *> &allFloors_out);
    /*!
     * @brief        Returns a snapshot copy of the candidate (marker-based)
     *               rooms of this map. Takes mapMutex.
     *
     * @param[out]   allMarkerBasedMapRooms_out
     *               Receives the candidate rooms; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getAllMarkerBasedMapRooms(
        std::vector<semantic::Room *> &allMarkerBasedMapRooms_out);
    /*!
     * @brief        Returns a snapshot copy of the candidate (marker-based)
     *               rooms of this map, the same set as
     *               getAllMarkerBasedMapRooms(). Takes mapMutex.
     *
     * @param[out]   allCandidateMapRooms_out
     *               Receives the candidate rooms; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getAllCandidateMapRooms(
        std::vector<semantic::Room *> &allCandidateMapRooms_out);
    /*!
     * @brief        Returns a snapshot copy of the reference map points set by
     *               setReferenceMapPoints(). Takes mapMutex.
     *
     * @param[out]   referenceMapPoints_out
     *               Receives the reference map points; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getReferenceMapPoints(std::vector<MapPoint *> &referenceMapPoints_out);
    /*!
     * @brief        Returns a snapshot copy of all passages of this map. Takes
     *               mapMutex.
     *
     * @param[out]   allPassages_out
     *               Receives the passages; borrowed pointers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
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

    /*!
     * @brief        Returns the number of key frames in this map. Takes
     *               mapMutex.
     *
     * @param[out]   keyFrameCount_out
     *               Number of key frames.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getKeyFrameCount(unsigned long &keyFrameCount_out);
    /*!
     * @brief        Returns the number of markers in this map. Takes mapMutex.
     *
     * @param[out]   markerCount_out
     *               Number of markers.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getMarkerCount(unsigned long &markerCount_out);
    /*!
     * @brief        Returns the number of map points in this map. Takes
     *               mapMutex.
     *
     * @param[out]   mapPointCount_out
     *               Number of map points.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getMapPointCount(unsigned long &mapPointCount_out);

    /*!
     * @brief        Returns the id this map was given at creation (or by
     *               changeId()). Does not lock.
     *
     * @param[out]   id_out
     *               Map id.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getId(unsigned long &id_out) const;
    /*!
     * @brief        Returns the highest key frame id added to this map; erasing
     *               key frames does not lower it. Takes mapMutex.
     *
     * @param[out]   maxKeyFrameId_out
     *               Highest key frame id.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getMaxKeyFrameId(unsigned long &maxKeyFrameId_out);
    /*!
     * @brief        Returns the id of the first key frame of this map. Takes
     *               mapMutex.
     *
     * @param[out]   initKeyFrameId_out
     *               Id of the first key frame.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getInitKeyFrameId(unsigned long &initKeyFrameId_out);
    /*!
     * @brief        Sets the id of the first key frame of this map. Takes
     *               mapMutex.
     *
     * @param[in]    initialKFif_in
     *               New id of the first key frame.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setInitKeyFrameId(long unsigned int initialKFif_in);

    /*!
     * @brief        Returns the origin key frame of this map, the one whose
     *               pose anchors the map. Does not lock.
     *
     * @param[out]   p_originKeyFrame_out
     *               Receives the origin key frame, or nullptr when it was
     *               erased and no key frame remains; borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getOriginKeyFrame(KeyFrame *&p_originKeyFrame_out);
    /*!
     * @brief        Looks up a floor by id. Takes mapMutex.
     *
     * @param[in]    floorId_in
     *               Id of the floor.
     * @param[out]   p_floorById_out
     *               Receives the floor, or nullptr when no floor has that id;
     *               borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getFloorById(int               floorId_in,
                                         semantic::Floor *&p_floorById_out);
    /*!
     * @brief        Looks up a door by id. The id lookup is never filled by
     *               addMapDoor(), so this returns nullptr for every id. Takes
     *               mapMutex.
     *
     * @param[in]    doorId_in
     *               Id of the door.
     * @param[out]   p_doorById_out
     *               Receives the door, or nullptr when no door has that id;
     *               borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getDoorById(int doorId_in, Door *&p_doorById_out);
    /*!
     * @brief        Looks up a plane by id. Takes mapMutex.
     *
     * @param[in]    planeId_in
     *               Id of the plane.
     * @param[out]   p_planeById_out
     *               Receives the plane, or nullptr when no plane has that id;
     *               borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getPlaneById(int                planeId_in,
                                         geometric::Plane *&p_planeById_out);
    /*!
     * @brief        Looks up a marker by id. Takes mapMutex.
     *
     * @param[in]    markerId_in
     *               Id of the marker.
     * @param[out]   p_markerById_out
     *               Receives the marker, or nullptr when no marker has that id;
     *               borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getMarkerById(int                markerId_in,
                                          semantic::Marker *&p_markerById_out);
    /*!
     * @brief        Looks up a key frame by id. Takes mapMutex.
     *
     * @param[in]    idCount_in
     *               Id of the key frame.
     * @param[out]   p_keyFrameById_out
     *               Receives the key frame, or nullptr when no key frame has
     *               that id; borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getKeyFrameById(long unsigned int idCount_in,
                                            KeyFrame *&p_keyFrameById_out);
    /*!
     * @brief        Looks up a passage by id. Takes mapMutex.
     *
     * @param[in]    passageId_in
     *               Id of the passage.
     * @param[out]   p_passageById_out
     *               Receives the passage, or nullptr when no passage has that
     *               id; borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
                            getPassageById(int                                  passageId_in,
                                           vs_graphs::core::semantic::Passage *&p_passageById_out);
    /*!
     * @brief        Looks up a room wall plane by plane id among the planes
     *               indexed by addRoomWallPlane(). Takes mapMutex.
     *
     * @param[in]    planeId_in
     *               Plane id of the wall.
     * @param[out]   p_roomWallPlaneById_out
     *               Receives the wall plane, or nullptr when none has that id;
     *               borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getRoomWallPlaneById(
        int                                 planeId_in,
        vs_graphs::core::geometric::Plane *&p_roomWallPlaneById_out);

    /*!
     * @brief        Picks the best-supported ground plane of this map: the one
     *               with the most finite support points, then the most
     *               observations, then the lowest plane id. Planes that are
     *               bad, not ground, have a non-unit or non-finite normal, or
     *               whose cloud was not refitted are skipped. Takes mapMutex
     *               through getAllPlanes().
     *
     * @param[out]   p_biggestGroundPlane_out
     *               Receives the ground plane, or nullptr when no plane
     *               qualifies; borrowed.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getBiggestGroundPlane(geometric::Plane *&p_biggestGroundPlane_out);

    /*!
     * @brief        Marks this map as stored, that is not the atlas's active
     *               map. Does not lock.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setStoredMap();
    /*!
     * @brief        Marks this map as the one currently used, the atlas's
     *               active map. Does not lock.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setCurrentMap();

    /*!
     * @brief        Tells whether this map is the atlas's active map. Does not
     *               lock.
     *
     * @param[out]   isInUse_out
     *               True when the map is in use.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus isInUse(bool &isInUse_out) const;

    /*!
     * @brief        Tells whether this map was flagged bad. Lock-free atomic
     *               read.
     *
     * @param[out]   isBad_out
     *               True when the map is flagged bad.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus isBad(bool &isBad_out);
    /*!
     * @brief        Flags this map bad; the flag is never cleared. Lock-free
     *               atomic write.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setBad();

    /*!
     * @brief        Empties this map: key frames (their map link is reset to
     *               null), map points, semantic entities, id indexes, reference
     *               points, origins and skeleton data are dropped, the highest
     *               key frame id goes back to the first key frame id and the
     *               IMU initialisation and inertial bundle adjustment flags are
     *               reset. The objects are not deleted. Does not lock mapMutex.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus clear();

    /*!
     * @brief        Returns the change index last recorded with
     *               setLastMapChange(), so a consumer can tell whether the map
     *               changed since it looked. Takes mapMutex.
     *
     * @param[out]   lastMapChange_out
     *               Last recorded change index.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getLastMapChange(int &lastMapChange_out);
    /*!
     * @brief        Returns the current change counter, which grows with every
     *               map change. Takes mapMutex.
     *
     * @param[out]   mapChangeIndex_out
     *               Current change counter.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getMapChangeIndex(int &mapChangeIndex_out);
    /*! Returns the epoch of the coordinate frame containing this map. */
    [[nodiscard]] MapStatus
        getWorldFrameEpoch(std::uint64_t &worldFrameEpoch_out);
    /*!
     * @brief        Counts one more map change. Takes mapMutex.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus increaseChangeIndex();
    /*!
     * @brief        Records the change index a consumer has already handled.
     *               Takes mapMutex.
     *
     * @param[in]    currentChangeId_in
     *               Change index to record.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setLastMapChange(int currentChangeId_in);

    /*!
     * @brief        Tells whether the IMU of this map has been initialised.
     *               Takes mapMutex.
     *
     * @param[out]   isImuInitialized_out
     *               True when the IMU is initialised.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus isImuInitialized(bool &isImuInitialized_out);
    /*!
     * @brief        Records that the IMU of this map is initialised. Takes
     *               mapMutex.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setImuInitialized();

    /*!
     * @brief        Moves the whole map into a new world frame: key frame
     *               poses, key frame velocities, map points, planes, markers,
     *               passages, rooms, floors and skeleton data are scaled and
     *               transformed; bad planes and rooms are skipped and doors are
     *               left untouched. Afterwards the change counter and the world
     *               frame epoch each grow by one. Takes mapMutex.
     *
     * @param[in]    alignmentPose_oldWorldToNewWorld_in
     *               Pose mapping points of the old world frame into the new
     *               world frame; its translation is in the new world frame,
     *               metres.
     * @param[in]    alignmentScale_in
     *               Scale factor from old to new world units.
     * @param[in]    isScaledVelocity_in
     *               True to also multiply the key frame velocities by the
     *               scale; false to only rotate them.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus applyScaledRotation(
        const Sophus::SE3f &alignmentPose_oldWorldToNewWorld_in,
        const float         alignmentScale_in,
        const bool          isScaledVelocity_in = false);

    /*!
     * @brief        Tells whether this map uses an inertial sensor. Takes
     *               mapMutex.
     *
     * @param[out]   isInertial_out
     *               True when the map is inertial.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus isInertial(bool &isInertial_out);
    /*!
     * @brief        Records that the first inertial bundle adjustment of this
     *               map has run. Takes mapMutex.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setInertialBA1();
    /*!
     * @brief        Records that the second inertial bundle adjustment of this
     *               map has run. Takes mapMutex.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setInertialBA2();
    /*!
     * @brief        Tells whether the first inertial bundle adjustment has run.
     *               Takes mapMutex.
     *
     * @param[out]   inertialBA1_out
     *               True when it has run.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getInertialBA1(bool &inertialBA1_out);
    /*!
     * @brief        Tells whether the second inertial bundle adjustment has
     *               run. Takes mapMutex.
     *
     * @param[out]   inertialBA2_out
     *               True when it has run.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus getInertialBA2(bool &inertialBA2_out);
    /*!
     * @brief        Marks this map as using an inertial sensor. Takes mapMutex.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus setInertialSensor();

    /*!
     * @brief        Overwrites the id of this map. Does not lock.
     *
     * @param[in]    idCount_in
     *               New map id.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus changeId(long unsigned int idCount_in);

    /*!
     * @brief        Returns the lowest key frame id in this map. Takes
     *               mapMutex.
     *
     * @param[out]   lowerKeyFrameId_out
     *               Lowest key frame id, or 0 when the map has no key frame.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        getLowerKeyFrameId(unsigned int &lowerKeyFrameId_out);

    /*!
     * @brief        Prepares the map for serialisation: drops observations that
     *               come from other maps or bad key frames, then copies the
     *               good map points and key frames, the origin ids and the
     *               first and lowest-id key frame ids into the backup fields
     *               that serialize() writes. Does not lock mapMutex.
     *
     * @param[in,out] cams_inout
     *               Set that collects the cameras used by the saved key frames.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        preSave(std::set<camera_models::geometriccamera::GeometricCamera *>
                    &cams_inout);
    /*!
     * @brief        Rebuilds the map after loading: restores the key frame and
     *               map point sets from the backup lists, links them to this
     *               map and re-creates references by id, adds the key frames to
     *               the database and restores the first, lowest-id and origin
     *               key frames. Does not lock mapMutex.
     *
     * @param[in,out] p_keyFrameDatabase_inout
     *               Key frame database the loaded key frames are added to;
     *               borrowed.
     * @param[in]    p_orbVocabulary_in
     *               ORB vocabulary given to the loaded key frames; borrowed.
     * @param[in,out] cams_inout
     *               Cameras by id used to re-link key frames to their camera.
     *
     * @return       MAP_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapStatus
        postLoad(KeyFrameDatabase *p_keyFrameDatabase_inout,
                 ORBVocabulary    *p_orbVocabulary_in,
                 std::map<unsigned int,
                          camera_models::geometriccamera::GeometricCamera *>
                     &cams_inout);

    /*!
     * @brief        Key frame kept only for map merges; borrowed. The
     *               constructors set it to null, eraseKeyFrame() clears it when
     *               that key frame is erased, and Atlas::mergeMapPair() takes
     *               the absorbed map's value when this map has none and the key
     *               frame now belongs to this map. Nothing else reads it.
     */
    KeyFrame                      *p_firstRegionKeyFrame;
    /*!
     * @brief        Held by tracking and the optimizers while they update this
     *               map, so the map cannot change under them.
     */
    std::mutex                     mapUpdateMutex;
    /*!
     * @brief        Key frames this map started from: the initial key frame of
     *               each initialisation, plus those imported by map merges.
     *               Erased key frames are removed.
     */
    std::vector<KeyFrame *>        keyFrameOrigins;
    /*!
     * @brief        Ids of keyFrameOrigins, filled by preSave() and read back
     *               by postLoad() because pointers cannot be saved.
     */
    std::vector<unsigned long int> backupKeyFrameOriginIds;

    /*!
     * @brief        Held while a map point is created, so two threads cannot
     *               create points with conflicting ids.
     */
    std::mutex pointCreationMutex;

    /*!
     * @brief        Failure flag; set to false at construction and never
     *               changed or read afterwards.
     */
    bool hasFailed;

    /*!
     * @brief        Width in pixels of the map thumbnail, a power of two; not
     *               used by any code yet.
     */
    static const int THUMB_WIDTH = 512;
    /*!
     * @brief        Height in pixels of the map thumbnail, a power of two; not
     *               used by any code yet.
     */
    static const int THUMB_HEIGHT = 512;

    /*!
     * @brief        Id given to the next map created; shared by all maps.
     */
    static long unsigned int nextId;

    /*!
     * @brief        Ids of the key frames the last local bundle adjustment
     *               optimised; used by the map drawer to colour them.
     */
    std::set<long unsigned int> optKeyFrameIds;
    /*!
     * @brief        Ids of the key frames the last local bundle adjustment held
     *               fixed; used by the map drawer to colour them.
     */
    std::set<long unsigned int> fixedKeyFrameIds;

  protected:
    /*!
     * @brief        Id of this map, taken from nextId at construction.
     */
    long unsigned int id;

    /*!
     * @brief        Floors of this map; borrowed.
     */
    std::set<semantic::Floor *>                    floors;
    /*!
     * @brief        Doors of this map; borrowed.
     */
    std::set<Door *>                               doors;
    /*!
     * @brief        Planes of this map; borrowed.
     */
    std::set<geometric::Plane *>                   planes;
    /*!
     * @brief        Markers of this map; borrowed.
     */
    std::set<semantic::Marker *>                   markers;
    /*!
     * @brief        Map points of this map; borrowed.
     */
    std::set<MapPoint *>                           mapPoints;
    /*!
     * @brief        Key frames of this map; borrowed.
     */
    std::set<KeyFrame *>                           keyFrames;
    /*!
     * @brief        Rooms detected in this map; borrowed.
     */
    std::set<semantic::Room *>                     detectedRooms;
    /*!
     * @brief        Candidate rooms built from markers that are not yet
     *               promoted to detected rooms; borrowed.
     */
    std::set<semantic::Room *>                     markerBasedRooms;
    /*!
     * @brief        Passages of this map; borrowed.
     */
    std::set<vs_graphs::core::semantic::Passage *> passages;

    /*!
     * @brief        Cluster points of the map set by `voxblox_skeleton`, in the
     *               world frame, metres.
     */
    std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

    /*!
     * @brief       Latest connected Voxblox skeleton graph edges.
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> skeletonEdges;

    // Hashmaps and indices for fetching elements

    /*!
     * @brief        Floors by floor id.
     */
    std::unordered_map<int, semantic::Floor *>                    floorIndex;
    /*!
     * @brief        Doors by door id; never filled, so lookups find nothing.
     */
    std::unordered_map<int, Door *>                               doorIndex;
    /*!
     * @brief        Planes by plane id.
     */
    std::unordered_map<int, geometric::Plane *>                   planeIndex;
    /*!
     * @brief        Markers by marker id.
     */
    std::unordered_map<int, semantic::Marker *>                   markerIndex;
    /*!
     * @brief        Key frames by key frame id.
     */
    std::unordered_map<long unsigned int, KeyFrame *>             keyFrameIndex;
    /*!
     * @brief        Passages by passage id.
     */
    std::unordered_map<int, vs_graphs::core::semantic::Passage *> passageIndex;
    std::unordered_map<int, vs_graphs::core::geometric::Plane *>
        /*!
         * @brief        Room wall planes by plane id.
         */
        roomWallPlaneIndex;

    /*!
     * @brief        Next plane id to hand out; ids stay unique even when fusion
     *               leaves gaps in planeIndex.
     */
    int nextAvailablePlaneId{0};
    /*!
     * @brief        Next floor id to hand out; ids stay unique even when fusion
     *               leaves gaps in floorIndex.
     */
    int nextAvailableFloorId{0};

    /*!
     * @brief        Map points copied by preSave() for saving, as a vector
     *               because the boost 1.58 set serialisation was broken;
     *               emptied by postLoad().
     */
    std::vector<MapPoint *> backupMapPoints;
    /*!
     * @brief        Key frames copied by preSave() for saving, as a vector
     *               because the boost 1.58 set serialisation was broken.
     */
    std::vector<KeyFrame *> backupKeyFrames;

    /*!
     * @brief        Origin key frame of this map, the one that anchors it;
     *               borrowed, set by the first addKeyFrame().
     */
    KeyFrame *p_initialKeyFrame;
    /*!
     * @brief        Key frame with the lowest id in this map; borrowed.
     */
    KeyFrame *p_lowerIdKeyFrame;

    /*!
     * @brief        Room this map started in, or nullptr until
     *               setStartingRoom() is called; borrowed.
     */
    vs_graphs::core::semantic::Room *p_startingRoom{nullptr};
    /*!
     * @brief        Room the robot was in when this map was left, or nullptr
     *               until setFinalRoom() is called; borrowed.
     */
    vs_graphs::core::semantic::Room *p_finalRoom{nullptr};
    /*!
     * @brief        Next map in the mission chain, or nullptr; borrowed from
     *               the atlas.
     */
    Map                             *p_followingMap{nullptr};

    /*!
     * @brief        Id of p_lowerIdKeyFrame saved by preSave(); NO_SAVED_ID
     *               when there was none.
     */
    unsigned long int backupLowerKeyFrameId;
    /*!
     * @brief        Id of p_initialKeyFrame saved by preSave(); NO_SAVED_ID
     *               when there was none.
     */
    unsigned long int backupInitialKeyFrameId;

    /*!
     * @brief        Reference map points set by setReferenceMapPoints();
     *               borrowed.
     */
    std::vector<MapPoint *> referenceMapPoints;

    /*!
     * @brief        True once the IMU of this map is initialised.
     */
    bool hasImuInitialization;

    /*!
     * @brief        Change counter: grows on increaseChangeIndex() and on
     *               applyScaledRotation().
     */
    int mapChange;
    /*!
     * @brief        Change index a consumer last recorded with
     *               setLastMapChange().
     */
    int mapChangeNotified;

    /*!
     * Monotonic coordinate-frame epoch. Unlike mapChange, ordinary local BA
     * and content updates do not increment it; whole-map rebases do.
     */
    std::uint64_t worldFrameEpoch;

    /*!
     * @brief        Id of the first key frame of this map.
     */
    long unsigned int initKeyFrameId;
    /*!
     * @brief        Highest key frame id added to this map so far.
     */
    long unsigned int maxKeyFrameId;

    /*!
     * @brief        Number of big changes (loop closure, global bundle
     *               adjustment) this map has seen.
     */
    int bigChangeIndex;

    /*!
     * @brief        Top-down thumbnail image of the map; deleted by the
     *               destructor, nullptr because nothing allocates it yet.
     */
    unsigned char *p_thumbnail;

    /*!
     * @brief        True while this map is the atlas's active map.
     */
    bool             isMapInUse;
    /*!
     * @brief        True when a thumbnail exists; always false because nothing
     *               creates one yet.
     */
    bool             hasThumbnail;
    /*!
     * @brief        True once setBad() flagged this map bad; atomic so isBad()
     *               needs no lock.
     */
    std::atomic_bool isFlaggedBad{false};

    /*!
     * @brief        True when this map uses an inertial sensor.
     */
    bool isInertialMode;
    /*!
     * @brief        True once the first inertial bundle adjustment has run.
     */
    bool hasInertialBA1;
    /*!
     * @brief        True once the second inertial bundle adjustment has run.
     */
    bool hasInertialBA2;

    /*!
     * @brief        Guards the map contents; taken by most methods.
     */
    std::mutex mapMutex;
};

} // namespace core
} // namespace vs_graphs

#endif
