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
 * @file            Atlas.h
 *
 * @brief           Declares Atlas, the set of maps built during a session,
 *                  including the bookkeeping for merging and retiring maps.
 */

#ifndef ATLAS_H
#define ATLAS_H

#include "AtlasCurrentMapStatus.h"
#include "AtlasStatus.h"
#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"
#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"
#include "CameraModels/Pinhole/objects/Pinhole.h"
#include "Geometric/Plane.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Semantic/Floor.h"
#include "Semantic/Marker.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "Semantic/RoomContextSnapshot.h"

#include <Eigen/Core>
#include <atomic>
#include <boost/serialization/access.hpp>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <vector>

namespace vs_graphs
{
namespace core
{
class Map;
namespace semantic
{
class Room;
}
class Frame;
namespace geometric
{
class Plane;
}
namespace semantic
{
class Floor;
}
class Viewer;
namespace semantic
{
class Marker;
}
namespace camera_models
{
namespace pinhole
{
class Pinhole;
} // namespace pinhole
} // namespace camera_models
namespace semantic
{
class Passage;
}
class MapPoint;
class KeyFrame;
namespace camera_models
{
namespace kannalabrandt8
{
class KannalaBrandt8;
} // namespace kannalabrandt8
} // namespace camera_models
class KeyFrameDatabase;

/*!
 * @brief           Per-old-map bookkeeping for continuous consecutive-map
 *                  matching.
 *
 *                  Lives in the Atlas (not in either map) because it describes
 *                  a map PAIR relationship across semantic cycles: when the
 *                  pair was last attempted and what the content looked like
 *                  then, so unchanged pairs are not re-attempted every cycle.
 */
struct MergeAttemptState
{
    /*!
     * @brief           Steady-clock time of the latest merge attempt for this
     *                  old map. Only meaningful once hasEverAttempted is true.
     */
    std::chrono::steady_clock::time_point lastAttemptTime{};
    /*!
     * @brief           Content hash (see consecutiveContentHash) of the old and
     *                  current map at the latest attempt; an unchanged hash
     *                  lets the scheduler skip the pair.
     */
    std::size_t                           contentHashAtLastAttempt{0U};
    /*!
     * @brief           True once an attempt has been recorded for this old map;
     *                  false means no cooldown or hash check applies yet.
     */
    bool                                  hasEverAttempted{false};
};

/*!
 * @brief           Serialisable snapshot of a room's geometric context at the
 *                  moment a
 *                         map is abandoned.
 *
 *                  Captured before the old map is stranded, these snapshots let
 *                  the Atlas re-identify the corresponding physical room when a
 *                  fresh map starts filling in from skeleton clustering. Wall
 *                  normals, centroids, and plane distances are stored so that a
 *                  best-match comparison can verify the room identity rather
 *                  than relying on a fragile centroid-only heuristic.
 */
class Atlas
{
    friend class boost::serialization::access;

    /*!
     * @brief           Saves or loads the Atlas for map save and load: the maps
     *                  (as backupMaps), the cameras, the static id counters and
     *                  lastInitKeyFrameId.
     *
     * @param[in,out]   ar
     *                  Boost archive that is written to or read from.
     *
     * @param[in]       version
     *                  Archive class version; unused.
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version);

  public:
    /*!
     * @brief           Whether copyRoomContextForMapChecked could copy the room
     *                  snapshots: COMPLETE, or UNAVAILABLE_LIVE_GENERATION when
     *                  the caller did not own the semantic update lock.
     */
    enum class SnapshotCopyStatus
    {
        COMPLETE,
        UNAVAILABLE_LIVE_GENERATION
    };

    /*!
     * @brief           Outcome of copyRoomContextForMapChecked: a status plus
     *                  the snapshots it copied.
     */
    struct SnapshotCopyResult
    {
        /*!
         * @brief           COMPLETE once snapshots is filled;
         *                  UNAVAILABLE_LIVE_GENERATION (the default) when the
         *                  copy was refused.
         */
        SnapshotCopyStatus status{
            SnapshotCopyStatus::UNAVAILABLE_LIVE_GENERATION};
        /*!
         * @brief           Room snapshots copied from a live map; empty unless
         *                  status is COMPLETE.
         */
        std::vector<semantic::RoomContextSnapshot> snapshots;
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Atlas();
    /*!
     * @brief           Creates an Atlas and immediately creates its first map.
     *
     * @param[in]       initialKeyFrameId_in
     *                  Key frame id at which the first map starts; stored as
     *                  lastInitKeyFrameId.
     */
    Atlas(int initialKeyFrameId_in);
    ~Atlas();
    /*!
     * @brief           Copying is forbidden: the atlas owns its maps and
     *                  deletes them when destroyed.
     */
    Atlas(const Atlas &otherAtlas_in)            = delete;
    Atlas &operator=(const Atlas &otherAtlas_in) = delete;

    /*!
     * @brief           Creates a new empty map and makes it the current one.
     *                  The previous current map is stored and linked to its
     *                  successor, its room geometry is recorded for identity
     *                  matching, and the new-map event becomes pending. Takes
     *                  atlasMutex.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus createNewMap();
    /*!
     * @brief           Makes an existing map the current one; the previous
     *                  current map is marked as stored. Takes atlasMutex.
     *
     * @param[in]       p_map_in
     *                  Map to make current; must not be null.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus changeMap(Map *p_map_in);

    /*!
     * @brief           Returns the key frame id at which the newest map was
     *                  initialised. createNewMap and preSave move it past the
     *                  active map's highest key frame id.
     *
     * @param[out]      lastInitKeyFrameId_out
     *                  Initial key frame id of the newest map.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getLastInitKeyFrameId(unsigned long &lastInitKeyFrameId_out);

    /*!
     * @brief           Remembers the viewer and records that one is attached.
     *
     * @param[in]       p_viewer_in
     *                  Viewer to remember; borrowed, the Atlas never deletes
     *                  it.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus setViewer(Viewer *p_viewer_in);

    // Methods for adding new components in the current map
    /*!
     * @brief           Registers a floor with the map the floor belongs to and
     *                  moves the floor identity allocator past its id. A null
     *                  pointer is ignored.
     *
     * @param[in]       p_floor_in
     *                  Floor to register; borrowed, the owning map keeps it.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addMapFloor(semantic::Floor *p_floor_in);
    /*!
     * @brief           Registers a plane with the map the plane belongs to.
     *
     * @param[in]       p_plane_in
     *                  Plane to register; must not be null.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addMapPlane(geometric::Plane *p_plane_in);
    /*!
     * @brief           Registers a key frame with the map the key frame belongs
     *                  to.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to register; must not be null.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addKeyFrame(KeyFrame *p_keyFrame_in);
    /*!
     * @brief           Registers a map point with the map the map point belongs
     *                  to.
     *
     * @param[in]       p_mapPoint_in
     *                  Map point to register; must not be null.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addMapPoint(MapPoint *p_mapPoint_in);
    /*!
     * @brief           Registers a marker with the map the marker belongs to.
     *
     * @param[in]       p_marker_in
     *                  Marker to register; must not be null.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addMapMarker(semantic::Marker *p_marker_in);
    /*!
     * @brief           Registers a detected room with the map the room belongs
     *                  to and moves the room identity allocator past its id. A
     *                  null pointer is ignored.
     *
     * @param[in]       p_room_in
     *                  Detected room to register; borrowed, the owning map
     *                  keeps it.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addDetectedMapRoom(semantic::Room *p_room_in);
    /*!
     * @brief           Registers a candidate room with the map the room belongs
     *                  to and moves the room identity allocator past its id. A
     *                  null pointer is ignored.
     *
     * @param[in]       p_room_in
     *                  Candidate room to register; borrowed, the owning map
     *                  keeps it.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addCandidateMapRoom(semantic::Room *p_room_in);
    /*!
     * @brief           Registers a passage with the map the passage belongs to
     *                  and moves the passage identity allocator past its id. A
     *                  null pointer is ignored.
     *
     * @param[in]       p_passage_in
     *                  Passage to register; borrowed, the owning map keeps it.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        addMapPassage(vs_graphs::core::semantic::Passage *p_passage_in);
    /*!
     * @brief           Registers a room wall plane with the map the plane
     *                  belongs to.
     *
     * @param[in]       p_plane_in
     *                  Wall plane to register; must not be null.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        addRoomWallPlane(vs_graphs::core::geometric::Plane *p_plane_in);

    /*!
     * @brief           Reserves the next mission-stable room identity,
     *                  beginning at zero.
     */
    [[nodiscard]] AtlasStatus reserveRoomIdentity(int &roomId_out);

    /*!
     * @brief           Reserves the next mission-stable passage identity,
     *                  beginning at zero.
     */
    [[nodiscard]] AtlasStatus reservePassageIdentity(int &passageId_out);

    /*!
     * @brief           Reserves the next mission-stable floor identity,
     *                  beginning at zero.
     */
    [[nodiscard]] AtlasStatus reserveFloorIdentity(int &floorId_out);

    /*!
     * @brief           Advances the room allocator past an explicitly restored
     *                  identity.
     */
    [[nodiscard]] AtlasStatus observeRoomIdentity(int roomId_in);

    /*!
     * @brief           Advances the passage allocator past an explicitly
     *                  restored identity.
     */
    [[nodiscard]] AtlasStatus observePassageIdentity(int passageId_in);

    /*!
     * @brief           Advances the floor allocator past an explicitly restored
     *                  identity.
     */
    [[nodiscard]] AtlasStatus observeFloorIdentity(int floorId_in);

    /*!
     * @brief           Stores the mission-stable identity occupied by the
     *                  camera.
     */
    [[nodiscard]] AtlasStatus setCurrentSemanticRoomIdentity(int roomId_in);

    /*!
     * @brief           Returns the last mission-stable room identity, or -1
     *                  before bootstrap.
     */
    [[nodiscard]] AtlasStatus getCurrentSemanticRoomIdentity(
        int &getCurrentSemanticRoomIdentity_out) const;

    /*!
     * @brief           Returns every camera model registered with the Atlas.
     *                  Does not take atlasMutex.
     *
     * @param[out]      allCameras_out
     *                  Copy of the camera pointer list; the pointers are
     *                  borrowed from the Atlas.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAllCameras(
        std::vector<camera_models::geometriccamera::GeometricCamera *>
            &allCameras_out);
    /*!
     * @brief           Registers a camera model, reusing an equal one that is
     *                  already registered. Does not take atlasMutex.
     *
     * @param[in]       p_camera_in
     *                  Camera model to register; must not be null.
     *
     * @param[out]      p_camera_out
     *                  The equal camera that was already registered, or
     *                  p_camera_in itself when it was new.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addCamera(
        camera_models::geometriccamera::GeometricCamera  *p_camera_in,
        camera_models::geometriccamera::GeometricCamera *&p_camera_out);

    /* All methods without Map pointer work on current map */
    /*!
     * @brief           Tells the active map that a large change happened (for
     *                  example a loop-closure correction) by advancing its
     *                  big-change index. Takes atlasMutex.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus informNewBigChange();
    /*!
     * @brief           Returns the active map's big-change index (see
     *                  informNewBigChange). Takes atlasMutex.
     *
     * @param[out]      lastBigChangeIndex_out
     *                  Number of big changes recorded by the active map.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getLastBigChangeIndex(int &lastBigChangeIndex_out);
    /*!
     * @brief           Stores the reference map points of the active map. Takes
     *                  atlasMutex.
     *
     * @param[in]       mapPoints_in
     *                  Map points to store; the points are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        setReferenceMapPoints(const std::vector<MapPoint *> &mapPoints_in);

    /*!
     * @brief           Returns the number of markers in the active map. Takes
     *                  atlasMutex.
     *
     * @param[out]      markerCount_out
     *                  Marker count of the active map.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getMarkerCount(unsigned long &markerCount_out);
    /*!
     * @brief           Returns the number of key frames in the active map.
     *                  Takes atlasMutex.
     *
     * @param[out]      keyFrameCount_out
     *                  Key frame count of the active map.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getKeyFrameCount(unsigned long &keyFrameCount_out);
    /*!
     * @brief           Returns the number of map points in the active map.
     *                  Takes atlasMutex.
     *
     * @param[out]      mapPointCount_out
     *                  Map point count of the active map.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getMapPointCount(unsigned long &mapPointCount_out);

    /*!
     * @brief           Ids of the markers that sit on planes detected so far.
     */
    std::vector<int> visitedPlanesMarkerIds;

    // Method for get data in current map
    /*!
     * @brief           Returns the rooms of the active map. Takes atlasMutex.
     *
     * @param[out]      allRooms_out
     *                  Copy of the active map's room pointer list; the pointers
     *                  are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getAllRooms(std::vector<semantic::Room *> &allRooms_out);
    /*!
     * @brief           Returns the floors of the active map. Takes atlasMutex.
     *
     * @param[out]      allFloors_out
     *                  Copy of the active map's floor pointer list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getAllFloors(std::vector<semantic::Floor *> &allFloors_out);
    /*!
     * @brief           Returns the markers of the active map. Takes atlasMutex.
     *
     * @param[out]      allMarkers_out
     *                  Copy of the active map's marker pointer list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getAllMarkers(std::vector<semantic::Marker *> &allMarkers_out);
    /*!
     * @brief           Returns the key frames of the active map. Takes
     *                  atlasMutex.
     *
     * @param[out]      allKeyFrames_out
     *                  Copy of the active map's key frame pointer list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getAllKeyFrames(std::vector<KeyFrame *> &allKeyFrames_out);
    /*!
     * @brief           Returns the map points of the active map. Takes
     *                  atlasMutex.
     *
     * @param[out]      allMapPoints_out
     *                  Copy of the active map's map point pointer list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getAllMapPoints(std::vector<MapPoint *> &allMapPoints_out);
    /*!
     * @brief           Returns the detected rooms of the active map. Takes
     *                  atlasMutex.
     *
     * @param[out]      allDetectedMapRooms_out
     *                  Copy of the active map's detected room pointer list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAllDetectedMapRooms(
        std::vector<semantic::Room *> &allDetectedMapRooms_out);
    /*!
     * @brief           Returns the planes of the active map. Takes atlasMutex.
     *
     * @param[out]      allPlanes_out
     *                  Copy of the active map's plane pointer list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAllPlanes(
        std::vector<vs_graphs::core::geometric::Plane *> &allPlanes_out);
    /*!
     * @brief           Returns the marker-based rooms of the active map. Takes
     *                  atlasMutex.
     *
     * @param[out]      allMarkerBasedMapRooms_out
     *                  Copy of the active map's marker-based room pointer list;
     *                  the pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAllMarkerBasedMapRooms(
        std::vector<semantic::Room *> &allMarkerBasedMapRooms_out);
    /*!
     * @brief           Returns the candidate rooms of the active map. Takes
     *                  atlasMutex.
     *
     * @param[out]      allCandidateMapRooms_out
     *                  Copy of the active map's candidate room pointer list;
     *                  the pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAllCandidateMapRooms(
        std::vector<semantic::Room *> &allCandidateMapRooms_out);
    /*!
     * @brief           Returns the reference map points of the active map.
     *                  Takes atlasMutex.
     *
     * @param[out]      referenceMapPoints_out
     *                  Copy of the active map's reference map point list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getReferenceMapPoints(std::vector<MapPoint *> &referenceMapPoints_out);
    /*!
     * @brief           Returns the passages of the active map. Takes
     *                  atlasMutex.
     *
     * @param[out]      allPassages_out
     *                  Copy of the active map's passage pointer list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAllPassages(
        std::vector<vs_graphs::core::semantic::Passage *> &allPassages_out);

    /*!
     * @brief           Get the cluster points of the map set by
     *                  `voxblox_skeleton`
     */
    [[nodiscard]] AtlasStatus getSkeletonClusterPoints(
        std::vector<std::vector<Eigen::Vector3d>> &skeletonClusterPoints_out);

    /*!
     * @brief           Gets the latest connected Voxblox skeleton edges.
     */
    [[nodiscard]] AtlasStatus getSkeletonEdges(
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            &skeletonEdges_out);

    /*!
     * @brief           Stores the latest connected Voxblox skeleton edges.
     */
    [[nodiscard]] AtlasStatus setSkeletonEdges(
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            &newSkeletonEdges_in);

    /*!
     * @brief           Set the cluster points of the map set by
     *                  `voxblox_skeleton`
     *
     * @param[in]       newClusterPoints_in
     *                  The new cluster points to set
     */
    [[nodiscard]] AtlasStatus setSkeletonClusterPoints(
        const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints_in);

    /*!
     * @brief           Returns the biggest ground plane of the active map.
     *                  Takes atlasMutex.
     *
     * @param[out]      p_biggestGroundPlane_out
     *                  Borrowed plane pointer as reported by the active map;
     *                  nullptr when the Atlas has no active map.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getBiggestGroundPlane(geometric::Plane *&p_biggestGroundPlane_out);

    /*!
     * @brief           Returns every active, non-bad map, sorted by ascending
     *                  map id. Takes atlasMutex.
     *
     * @param[out]      allMaps_out
     *                  Borrowed map pointers; the Atlas owns the maps.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAllMaps(std::vector<Map *> &allMaps_out);

    /*!
     * @brief           Checks whether a map is still an active, non-retired
     *                  Atlas map.
     *
     * @param[in]       p_map_in
     *                  Map pointer to validate.
     *
     * @param[out]      isActiveMap_out
     *                  True only while the map is active and not marked bad.
     *
     * @return          ATLAS_STATUS_SUCCESS.
     */
    [[nodiscard]] AtlasStatus isActiveMap(Map *p_map_in, bool &isActiveMap_out);

    /*!
     * @brief           Returns how many maps are in the active set. Takes
     *                  atlasMutex.
     *
     * @param[out]      maps_out
     *                  Number of active maps.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus countMaps(int &maps_out);

    /*!
     * @brief           Wipes the content of the current map in place without
     *                  creating a new map. The room geometry is recorded first
     *                  so the rooms can be re-identified later, and the map's
     *                  big-change index advances. Takes atlasMutex.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus clearMap();

    /*!
     * @brief           Empties the active map set, forgets the current map and
     *                  resets lastInitKeyFrameId to 0. The Map objects are not
     *                  deleted. Takes atlasMutex.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus clearAtlas();

    /*!
     * @brief           Returns the current map. Creates the first map when
     *                  there is none, and while the current map is marked bad
     *                  (a merge is in progress) polls every 3 ms until
     *                  changeMap installs a replacement. Takes atlasMutex.
     *
     * @param[out]      p_currentMap_out
     *                  Borrowed pointer to the current map; never null on
     *                  return.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getCurrentMap(Map *&p_currentMap_out);

    /*!
     * @brief           Returns the current map's id, its truthful status
     *                  relative to the active map set, and every active Atlas
     *                  map, as one coherent, non-mutating value view.
     *
     *                  Unlike GetCurrentMap(), this method never creates a map:
     *                  an Atlas with no current map (e.g. immediately after
     *                  clearAtlas(), or before the first CreateNewMap())
     *                  reports an empty \p currentMapId_out,
     *                  \p currentMapStatus_out ==
     *                  AtlasCurrentMapStatus::NO_CURRENT_MAP, and an empty
     *                  returned vector, rather than fabricating a new map as a
     *                  read side effect. All three facts are read under one
     *                  atlasMutex critical section, so no concurrent
     *                  ChangeMap()/SetMapBad()/map add or remove can produce a
     *                  torn read across them -- the two-call race possible with
     *                  separate GetCurrentMap() and GetAllMaps() calls cannot
     *                  occur here. This is a narrower guarantee than "the
     *                  current map always names an active map": Atlas's own
     *                  state can already be transiently incoherent independent
     *                  of any race, because SetMapBad(p_map_in) erases
     *                  \p p_map_in from the active set and marks it bad without
     *                  clearing p_activeMap -- a later ChangeMap() call is what
     *                  eventually installs a replacement current map. When
     *                  \p p_map_in == the current map, every call to this
     *                  method between those two events truthfully reports
     *                  \p currentMapId_out naming a map absent from the
     *                  returned vector, via \p currentMapStatus_out ==
     *                  AtlasCurrentMapStatus::CURRENT_MAP_NOT_ACTIVE, rather
     *                  than silently claiming a consistency invariant that does
     *                  not hold at that instant. This method never busy-waits,
     *                  unlike GetCurrentMap().
     *
     * @param[in,out]   currentMapId_inout
     *                  Set to the current map's id, or left empty when
     *                  p_activeMap is null at the moment of the read.
     *
     * @param[out]      currentMapStatus_out
     *                  Set to NO_CURRENT_MAP, CURRENT_MAP_ACTIVE, or
     *                  CURRENT_MAP_NOT_ACTIVE; see AtlasCurrentMapStatus.h.
     *
     * @param[out]      coherentMapView_out
     *                  Every active Atlas map, sorted by id -- identical
     *                  content and order to GetAllMaps().
     *
     * @return          ATLAS_STATUS_SUCCESS.
     */
    [[nodiscard]] AtlasStatus
        getCoherentMapView(std::optional<long unsigned int> &currentMapId_inout,
                           AtlasCurrentMapStatus &currentMapStatus_out,
                           std::vector<Map *>    &coherentMapView_out);

    /*!
     * @brief           Acquires exclusive access to semantic-map mutations.
     *
     *                  Loop closing holds this lock while semantic entities are
     *                  transformed, transferred, and fused. Semantic worker
     *                  threads use the same lock before changing the graph,
     *                  which prevents guarded writers from interleaving
     *                  mutations.
     *
     * @param[out]      semanticUpdateLock_out
     *                  Movable lock which releases the semantic transaction
     *                  when it leaves scope.
     *
     * @return          ATLAS_STATUS_SUCCESS.
     */
    [[nodiscard]] AtlasStatus acquireSemanticUpdateLock(
        std::unique_lock<std::mutex> &semanticUpdateLock_out);

    /*!
     * @brief           Removes a map from the active Atlas and marks it
     *                  invalid.
     *
     * @param[in,out]   p_map_inout
     *                  Map whose merge lifecycle has completed.
     */
    [[nodiscard]] AtlasStatus setMapBad(Map *p_map_inout);

    /*!
     * @brief           Merges the semantic graph of \p p_otherMap_in into
     *                  \p p_currentMap_in.
     *
     *                  The current map survives and keeps its authoritative
     *                  frame. The other map's keyframes, map points, planes,
     *                  markers, passages, detected rooms, marker-based rooms,
     *                  and floors are transformed into the current map's frame
     *                  using Horn's closed-form solution on corresponding wall
     *                  normals and centroids, transferred into the current map,
     *                  fused, and re-associated. The other map is then marked
     *                  bad.
     *
     *                  Deterministic: Horn's closed-form method, no iteration.
     *
     *                  The caller must already hold the semantic-update lock
     *                  (see \ref acquireSemanticUpdateLock); this method does
     *                  not acquire it.
     *
     * @param[in,out]   p_currentMap_inout
     *                  Map that survives the merge and remains active.
     *
     * @param[in,out]   p_otherMap_inout
     *                  Map to be transformed, absorbed, then marked bad.
     */
    [[nodiscard]] AtlasStatus mergeMapPair(Map *p_currentMap_inout,
                                           Map *p_otherMap_inout);

    /*!
     * @brief           Attempts validated consecutive-map merges, old into
     *                  current.
     *
     *                  Runs the continuous shape-matching scheduler over every
     *                  non-bad, non-current map: room-prior seed gate,
     *                  minimum-shape gate, cooldown/change gate, shared-Horn
     *                  transform estimation, then the consecutive merge gate.
     *                  On ACCEPT the old map is fused via MergeMapPair (which
     *                  retires it); on anything else nothing mutates. At most
     *                  one merge commits per call.
     *
     *                  The caller must already hold the semantic-update lock
     *                  (see \ref acquireSemanticUpdateLock); this method does
     *                  not acquire it.
     */
    [[nodiscard]] AtlasStatus attemptConsecutiveMergeIfGated(void);

    /*!
     * @brief           Captures the room geometry of the current map before it
     *                  is
     *                         stranded by a restart.
     *
     *                  Called internally from
     *                  \ref createNewMapWhileAtlasLocked, clearMap() and
     *                  clearAtlas() so that room identity survives map
     *                  transitions.
     */
    [[nodiscard]] AtlasStatus exportRoomContextFromCurrentMap();

    /*!
     * @brief           Transfers room identity tags from accumulated context
     *                  snapshots
     *                         to untagged rooms in \p pNewMap.
     *
     *                  Uses a best-match strategy: the room whose centroid is
     *                  closest to a snapshot centroid — provided the distance
     *                  is below \ref kRoomContextMatchThreshold_m and wall
     *                  normals agree — inherits that snapshot's room identity
     *                  tag.
     *
     * @param[in]       p_newMap_in
     *                  Map whose rooms should be examined/tagged.
     */
    [[nodiscard]] AtlasStatus matchRoomsToContext(Map *p_newMap_in);

    /*!
     * @brief           Copies room history without exposing references beyond
     *                  the lock scope.
     */
    [[nodiscard]] AtlasStatus copyRoomContextHistory(
        std::map<unsigned long, std::vector<semantic::RoomContextSnapshot>>
            &copyRoomContextHistory_out) const;

    /*!
     * @brief           Copies invariant context from a live map under the
     *                  semantic transaction.
     */
    [[nodiscard]] AtlasStatus copyRoomContextForMap(
        Map                                        *p_map_in,
        std::vector<semantic::RoomContextSnapshot> &roomContexts_out);

    /*!
     * @brief           Copies the newest departed-map snapshot for one stable
     *                  room identity.
     */
    [[nodiscard]] AtlasStatus copyLatestRoomContext(
        int                                           roomId_in,
        std::optional<semantic::RoomContextSnapshot> &roomContext_out) const;

    /*!
     * @brief           Copies live entities only when the caller owns the
     *                  semantic transaction.
     */
    [[nodiscard]] AtlasStatus
        copyRoomContextForMapChecked(Map *p_map_inout,
                                     bool callerOwnsSemanticLock_in,
                                     Atlas::SnapshotCopyResult &copyResult_out);

    /*!
     * @brief           Returns and clears the map-created lifecycle event.
     */
    [[nodiscard]] AtlasStatus
        consumeNewMapCreatedEvent(bool &wasEventPending_out);

    /*!
     * @brief           Reads the lifecycle event without acknowledging it.
     */
    [[nodiscard]] AtlasStatus
        peekNewMapCreatedEvent(bool &isEventPending_out) const;

    /*!
     * @brief           Acknowledges the pending lifecycle event after verified
     *                  handling.
     */
    [[nodiscard]] AtlasStatus acknowledgeNewMapCreatedEvent();

    /*!
     * @brief           Returns the vector of context snapshots stored for
     *                  \p mapId.
     */
    [[nodiscard]] AtlasStatus
        getRoomContextForMap(long unsigned int mapId_in,
                             const std::vector<semantic::RoomContextSnapshot> *
                                 &p_roomContextForMap_out) const;

    /*!
     * @brief           Moves invalid maps out of the transient retirement
     *                  queue.
     *
     *                  Retired maps remain owned by the Atlas until shutdown.
     *                  Delayed reclamation avoids invalidating raw map pointers
     *                  which can still be held by tracking or visualization
     *                  readers after a merge.
     */
    [[nodiscard]] AtlasStatus removeBadMaps();

    /*!
     * @brief           Reports whether the active map uses an inertial sensor.
     *                  Takes atlasMutex.
     *
     * @param[out]      isInertial_out
     *                  True when the active map is inertial.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus isInertial(bool &isInertial_out);
    /*!
     * @brief           Marks the active map as using an inertial sensor. Takes
     *                  atlasMutex.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus setInertialSensor();
    /*!
     * @brief           Marks the IMU of the active map as initialised. Takes
     *                  atlasMutex.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus setImuInitialized();
    /*!
     * @brief           Reports whether the IMU of the active map is
     *                  initialised. Takes atlasMutex.
     *
     * @param[out]      isImuInitialized_out
     *                  True when the active map's IMU is initialised.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus isImuInitialized(bool &isImuInitialized_out);

    // Function for garantee the correction of serialization of this object
    /*!
     * @brief           Prepares the Atlas for saving: advances
     *                  lastInitKeyFrameId past the active map's highest key
     *                  frame id, copies the maps sorted by id into backupMaps,
     *                  marks empty maps bad, lets each remaining map prepare
     *                  itself, then calls removeBadMaps.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus preSave();
    /*!
     * @brief           Restores run-time state after loading: refills the
     *                  active map set from backupMaps, lets each map re-link
     *                  its key frames to the key frame database, the vocabulary
     *                  and the cameras (looked up by id), then empties
     *                  backupMaps.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus postLoad();

    /*!
     * @brief           Returns the key frames of every map in backupMaps, keyed
     *                  by key frame id. backupMaps is only filled between
     *                  preSave and postLoad.
     *
     * @param[out]      atlasKeyFrames_out
     *                  Borrowed key frame pointers keyed by key frame id.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAtlasKeyFrames(
        std::map<unsigned long, KeyFrame *> &atlasKeyFrames_out);

    // Functions for getting the entities
    /*!
     * @brief           Looks up a plane of the active map by id. Takes
     *                  atlasMutex.
     *
     * @param[in]       planeId_in
     *                  Id of the plane to find.
     *
     * @param[out]      p_planeById_out
     *                  Borrowed pointer; nullptr when there is no active map or
     *                  no such plane.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getPlaneById(int                planeId_in,
                                           geometric::Plane *&p_planeById_out);
    /*!
     * @brief           Looks up a floor of the active map by id. Takes
     *                  atlasMutex.
     *
     * @param[in]       floorId_in
     *                  Id of the floor to find.
     *
     * @param[out]      p_floorById_out
     *                  Borrowed pointer; nullptr when there is no active map or
     *                  no such floor.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getFloorById(int               floorId_in,
                                           semantic::Floor *&p_floorById_out);
    /*!
     * @brief           Looks up a marker of the active map by id. Takes
     *                  atlasMutex.
     *
     * @param[in]       markerId_in
     *                  Id of the marker to find.
     *
     * @param[out]      p_markerById_out
     *                  Borrowed pointer; nullptr when there is no active map or
     *                  no such marker.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getMarkerById(int markerId_in, semantic::Marker *&p_markerById_out);
    /*!
     * @brief           Looks up a key frame of the active map by id. Takes
     *                  atlasMutex.
     *
     * @param[in]       idCount_in
     *                  Id of the key frame to find.
     *
     * @param[out]      p_keyFrameById_out
     *                  Borrowed pointer; nullptr when there is no active map or
     *                  no such key frame.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getKeyFrameById(long unsigned int idCount_in,
                                              KeyFrame *&p_keyFrameById_out);
    /*!
     * @brief           Looks up a passage of the active map by id. Takes
     *                  atlasMutex.
     *
     * @param[in]       passageId_in
     *                  Id of the passage to find.
     *
     * @param[out]      p_passageById_out
     *                  Borrowed pointer; nullptr when there is no active map or
     *                  no such passage.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
                              getPassageById(int                                  passageId_in,
                                             vs_graphs::core::semantic::Passage *&p_passageById_out);
    /*!
     * @brief           Looks up a room wall plane of the active map by id.
     *                  Takes atlasMutex.
     *
     * @param[in]       planeId_in
     *                  Id of the wall plane to find.
     *
     * @param[out]      p_roomWallPlaneById_out
     *                  Borrowed pointer; nullptr when there is no active map or
     *                  no such wall plane.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getRoomWallPlaneById(
        int                                 planeId_in,
        vs_graphs::core::geometric::Plane *&p_roomWallPlaneById_out);

    /*!
     * @brief           Returns the key frame database that postLoad uses to
     *                  re-link key frames.
     *
     * @param[out]      p_keyFrameDatabase_out
     *                  Borrowed pointer; whatever setKeyFrameDatabase stored.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getKeyFrameDatabase(KeyFrameDatabase *&p_keyFrameDatabase_out);
    /*!
     * @brief           Stores the key frame database that postLoad uses to
     *                  re-link key frames.
     *
     * @param[in]       p_keyFrameDatabase_in
     *                  Database to remember; borrowed, the Atlas never deletes
     *                  it.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        setKeyFrameDatabase(KeyFrameDatabase *p_keyFrameDatabase_in);

    /*!
     * @brief           Returns the ORB vocabulary that postLoad uses to re-link
     *                  key frames.
     *
     * @param[out]      p_oRBVocabulary_out
     *                  Borrowed pointer; whatever setORBVocabulary stored.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getORBVocabulary(ORBVocabulary *&p_oRBVocabulary_out);
    /*!
     * @brief           Stores the ORB vocabulary that postLoad uses to re-link
     *                  key frames.
     *
     * @param[in]       p_orbVocabulary_in
     *                  Vocabulary to remember; borrowed, the Atlas never
     *                  deletes it.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        setORBVocabulary(ORBVocabulary *p_orbVocabulary_in);

    /*!
     * @brief           Counts the key frames of every active map, not only the
     *                  current one. Takes atlasMutex.
     *
     * @param[out]      livedKeyFrameCount_out
     *                  Total key frame count over all active maps.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getLivedKeyFrameCount(unsigned long &livedKeyFrameCount_out);
    /*!
     * @brief           Counts the map points of every active map, not only the
     *                  current one. Takes atlasMutex.
     *
     * @param[out]      livedMapPointCount_out
     *                  Total map point count over all active maps.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getLivedMapPointCount(unsigned long &livedMapPointCount_out);

  protected:
    /*!
     * @brief           Creates the next map while the caller owns atlasMutex.
     */
    [[nodiscard]] AtlasStatus createNewMapWhileAtlasLocked();

    /*!
     * @brief           Active maps. The Atlas owns them and deletes them in its
     *                  destructor.
     */
    std::set<Map *> maps;
    /*!
     * @brief           Maps marked bad by setMapBad that removeBadMaps has not
     *                  yet moved to retiredMaps; still owned by the Atlas.
     */
    std::set<Map *> badMaps;

    /*!
     * @brief           Maps retired from active use but still owned until Atlas
     *                  destruction.
     */
    std::set<Map *> retiredMaps;

    /*!
     * @brief           Maps sorted by id, filled by preSave for serialisation
     *                  and emptied by postLoad. A vector rather than a set
     *                  because libboost 1.58 on Ubuntu 16.04 fails on a
     *                  serialised set.
     */
    std::vector<Map *> backupMaps;

    /*!
     * @brief           The current map, or nullptr before the first map exists
     *                  and after clearAtlas. Borrowed from maps; guarded by
     *                  atlasMutex.
     */
    Map *p_activeMap;

    /*!
     * @brief           Distinct camera models used by the key frames;
     *                  serialised. The Atlas does not delete them.
     */
    std::vector<camera_models::geometriccamera::GeometricCamera *> cameras;

    /*!
     * @brief           Key frame id at which the newest map was initialised;
     *                  serialised. createNewMap and preSave advance it.
     */
    unsigned long int lastInitKeyFrameId;

    /*!
     * @brief           Viewer given to setViewer; borrowed and only meaningful
     *                  once hasViewer is true.
     */
    Viewer *p_viewer;
    /*!
     * @brief           True once setViewer has stored a viewer.
     */
    bool    hasViewer;

    /*!
     * @brief           Key frame database that postLoad hands to the loaded
     *                  maps; borrowed, set by setKeyFrameDatabase.
     */
    KeyFrameDatabase *p_keyFrameDatabase;
    /*!
     * @brief           ORB vocabulary that postLoad hands to the loaded maps;
     *                  borrowed, set by setORBVocabulary.
     */
    ORBVocabulary    *p_orbVocabulary;

    /*!
     * @brief           Guards the map sets, p_activeMap and lastInitKeyFrameId.
     */
    std::mutex atlasMutex;

    /*!
     * @brief           Serialises semantic graph updates with map-merge
     *                  transactions.
     */
    std::mutex semanticUpdateMutex;

    /*!
     * @brief           Best-match centroid distance (m) below which a room in a
     *                  new map is considered the same physical room as a
     *                  prior-map snapshot.
     */
    static constexpr double kRoomContextMatchThreshold_m = 2.0;

    /*!
     * @brief           Minimum |cos θ| between wall normals for two wall
     *                  hypotheses to be considered the same physical surface
     *                  during context matching.
     */
    static constexpr double kWallNormalAlignmentCosTheta = 0.85;

    /*!
     * @brief           Snapshots of departed maps' room geometry, keyed by
     *                  Map::GetId().
     */
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
        roomContextHistory;

    /*!
     * @brief           Continuous-match bookkeeping per old map ID. Guarded by
     *                  atlasMutex like the other Atlas-owned indexes.
     */
    std::map<long unsigned int, MergeAttemptState> consecutiveMergeState;

    /*!
     * @brief           Protects roomContextHistory against concurrent access
     *                  from tracking and the semantic worker thread.
     */
    mutable std::mutex roomContextMutex;

    /*!
     * @brief           Mission-wide semantic allocators survive active-map
     *                  replacement.
     */
    std::atomic<int> nextRoomIdentity{0};
    /*!
     * @brief           Next passage identity that reservePassageIdentity hands
     *                  out.
     */
    std::atomic<int> nextPassageIdentity{0};
    /*!
     * @brief           Next floor identity that reserveFloorIdentity hands out.
     */
    std::atomic<int> nextFloorIdentity{0};

    /*!
     * @brief           Last occupied semantic identity; map-local temporal
     *                  resets do not clear it.
     */
    std::atomic<int> currentSemanticRoomIdentity{-1};

    /*!
     * @brief           Pending lifecycle event, protected by roomContextMutex.
     */
    bool isNewMapCreatedPending{false};
};

} // namespace core
} // namespace vs_graphs

#endif
