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
 * @file            AtlasClass.h
 *
 * @brief           Declares Atlas, the set of maps built during a session,
 *                  including the bookkeeping for merging and retiring maps.
 */

#ifndef VS_GRAPHS_CORE_SRC_ATLAS_OBJECTS_ATLAS_CLASS_H
#define VS_GRAPHS_CORE_SRC_ATLAS_OBJECTS_ATLAS_CLASS_H

#include "map_state/atlas/objects/AtlasCurrentMapStatusEnum.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/atlas/objects/RoomContextExportStruct.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/map/map.h"
#include "map_state/map_point/map_point.h"
#include "mapping/geometric/Plane/Plane.h"
#include "scene_understanding/scene_graph/floor/Floor.h"
#include "scene_understanding/scene_graph/passage/Passage.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include "sensor_models/camera_models/geometric_camera/geometric_camera.h"
#include "sensor_models/camera_models/kannala_brandt8/kannala_brandt8.h"
#include "sensor_models/camera_models/pinhole/pinhole.h"

#include <Eigen/Core>
#include <atomic>
#include <boost/serialization/access.hpp>
#include <chrono>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <vector>

namespace vs_graphs::core::frame
{
class Frame;
}
namespace vs_graphs::core::keyframe
{
class KeyFrame;
}
namespace vs_graphs::core::keyframe_database
{
class KeyFrameDatabase;
}
namespace vs_graphs::core::map
{
class Map;
}
namespace vs_graphs::core::map_point
{
class MapPoint;
}

namespace vs_graphs
{
namespace core
{

class SemanticsManager;

namespace semantic
{
class Room;
}

namespace geometric
{
class Plane;
}
namespace semantic
{
class Floor;
}
class Viewer;
namespace camera_models
{
namespace pinhole
{
class Pinhole;
} /* namespace pinhole */
} /* namespace camera_models */
namespace semantic
{
class Passage;
}

namespace camera_models
{
namespace kannala_brandt8
{
class KannalaBrandt8;
} /* namespace kannala_brandt8 */
} /* namespace camera_models */

namespace map
{
class Map;
} /* namespace map */
namespace keyframe
{
class KeyFrame;
} /* namespace keyframe */
namespace map_point
{
class MapPoint;
} /* namespace map_point */
namespace frame
{
class Frame;
} /* namespace frame */
namespace atlas
{

/*!
 * @brief           Serialisable snapshot of a room's geometric context at the
 *                  moment a map is abandoned.
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
     *                  lastInitKeyFrameId. When loading, the cameras Boost
     *                  creates are added to loadedCameras, which owns them.
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Creates an Atlas with no map, no viewer, no key frame
     *                  database and no vocabulary; lastInitKeyFrameId is 0.
     *                  Boost builds an Atlas this way before loading a saved
     *                  one.
     */
    Atlas()
    {
        /* Init map and set current active map to null */
        p_activeMap = static_cast<map::Map *>(nullptr);
    }

    /*!
     * @brief           Creates an Atlas and immediately creates its first map.
     *
     * @param[in]       initialKeyFrameId_in
     *                  Key frame id at which the first map starts; stored as
     *                  lastInitKeyFrameId.
     */
    Atlas(int initialKeyFrameId_in) :
        Atlas()
    {
        /* Store the value of the keyframe id */
        lastInitKeyFrameId = initialKeyFrameId_in;

        /* Create a new map */
        if (createNewMap() != AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: createNewMap returned a failure status although it "
                "cannot fail; continuing as before.",
                __func__);
        }
    }

    /*!
     * @brief           Deletes every map the atlas still holds, each exactly
     *                  once, after the maps leave the semantics manager's
     *                  registry.
     */
    ~Atlas()
    {
        /* The maps own their S-Graphs and the semantics manager keeps a
         * registry of them, so every map leaves the registry before it is
         * deleted. */
        std::vector<map::Map *> mapsToUnregister{};
        for (const std::pair<const unsigned long, std::unique_ptr<map::Map>>
                 &mapEntry : maps)
        {
            mapsToUnregister.push_back(mapEntry.second.get());
        }
        for (const std::unique_ptr<map::Map> &p_badMap : badMaps)
        {
            mapsToUnregister.push_back(p_badMap.get());
        }
        for (const std::unique_ptr<map::Map> &p_retiredMap : retiredMaps)
        {
            mapsToUnregister.push_back(p_retiredMap.get());
        }
        if (unregisterSceneGraphs(mapsToUnregister) !=
            AtlasStatus::ATLAS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: unregisterSceneGraphs returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        maps.clear();
        badMaps.clear();
        retiredMaps.clear();
    }
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
     *                  map::Map to make current; must not be null.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus changeMap(map::Map *p_map_in);

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

    /* ---------------------------------------------------------------------- *
     * METHODS FOR ADDING NEW COMPONENTS IN THE CURRENT MAP
     * ---------------------------------------------------------------------- */

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
    [[nodiscard]] AtlasStatus addKeyFrame(keyframe::KeyFrame *p_keyFrame_in);

    /*!
     * @brief           Registers a map point with the map the map point belongs
     *                  to.
     *
     * @param[in]       p_mapPoint_in
     *                  map::Map point to register; must not be null.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addMapPoint(map_point::MapPoint *p_mapPoint_in);

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
     *                  Takes atlasMutex while it copies the list and takes no
     *                  other lock under it, so the caller must not hold
     *                  atlasMutex (no Atlas method calls this).
     *
     * @param[out]      allCameras_out
     *                  Copy of the camera pointer list; the pointers are
     *                  borrowed, never deleted by the caller (see cameras for
     *                  their owners).
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getAllCameras(
        std::vector<camera_models::geometric_camera::GeometricCamera *>
            &allCameras_out);

    /*!
     * @brief           Registers a camera model, reusing an equal one that is
     *                  already registered. Takes atlasMutex for the whole
     *                  search and insertion. Lock order: atlasMutex is the only
     *                  lock held, because the comparison calls camera methods,
     *                  which take no lock; the caller must not hold atlasMutex
     *                  (no Atlas method calls this).
     *
     * @param[in]       p_camera_in
     *                  Camera model to register; must not be null. Borrowed,
     *                  never deleted by the Atlas: it stays with its creator
     *                  (Settings or Tracking), which must keep it alive while
     *                  the Atlas or a key frame may use it, also when an equal
     *                  camera is returned instead.
     *
     * @param[out]      p_camera_out
     *                  The equal camera that was already registered, or
     *                  p_camera_in itself when it was new; borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus addCamera(
        camera_models::geometric_camera::GeometricCamera  *p_camera_in,
        camera_models::geometric_camera::GeometricCamera *&p_camera_out);

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
     *                  map::Map points to store; the points are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus setReferenceMapPoints(
        const std::vector<map_point::MapPoint *> &mapPoints_in);

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
     *                  map::Map point count of the active map.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getMapPointCount(unsigned long &mapPointCount_out);

    /* ---------------------------------------------------------------------- *
     * METHODS FOR GETTING DATA IN CURRENT MAP
     * ---------------------------------------------------------------------- */

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
        getAllKeyFrames(std::vector<keyframe::KeyFrame *> &allKeyFrames_out);

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
        getAllMapPoints(std::vector<map_point::MapPoint *> &allMapPoints_out);

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
     * @brief           Returns the reference map points of the active map.
     *                  Takes atlasMutex.
     *
     * @param[out]      referenceMapPoints_out
     *                  Copy of the active map's reference map point list; the
     *                  pointers are borrowed.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus getReferenceMapPoints(
        std::vector<map_point::MapPoint *> &referenceMapPoints_out);

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
    [[nodiscard]] AtlasStatus getAllMaps(std::vector<map::Map *> &allMaps_out);

    /*!
     * @brief           Checks whether a map is still an active, non-retired
     *                  Atlas map.
     *
     * @param[in]       p_map_in
     *                  map::Map pointer to validate.
     *
     * @param[out]      isActiveMap_out
     *                  True only while the map is active and not marked bad.
     *
     * @return          ATLAS_STATUS_SUCCESS.
     */
    [[nodiscard]] AtlasStatus isActiveMap(map::Map *p_map_in,
                                          bool     &isActiveMap_out);

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
     *                  big-change index advances. Takes semanticUpdateMutex,
     *                  then atlasMutex; the caller must hold neither.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus clearMap();

    /*!
     * @brief           Empties the active map set, forgets the current map and
     *                  resets lastInitKeyFrameId to 0. The maps are moved to
     *                  retiredMaps, not deleted, because key frames, map points
     *                  and other threads can still hold pointers to them; the
     *                  destructor deletes them. Takes atlasMutex.
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
    [[nodiscard]] AtlasStatus getCurrentMap(map::Map *&p_currentMap_out);

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
                           AtlasCurrentMapStatus   &currentMapStatus_out,
                           std::vector<map::Map *> &coherentMapView_out);

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
     * @brief           Returns the semantics manager that serves the S-Graph of
     *                  the maps this atlas holds. No lock is taken: the manager
     *                  guards its graph with its own graph mutex.
     *
     * @param[out]      p_semanticsManager_out
     *                  Borrowed pointer to the semantics manager; null only
     *                  before setSemanticsManager() has been called.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getSemanticsManager(SemanticsManager *&p_semanticsManager_out);

    /*!
     * @brief           Attaches the semantics manager that serves the S-Graph
     *                  of the maps and registers the maps the atlas already
     *                  holds (and its current map as the manager's active map),
     *                  because the first map is created before the manager
     *                  exists. Takes atlasMutex, then the manager's graph lock.
     *
     * @param[in]       p_semanticsManager_in
     *                  Semantics manager to attach; borrowed, never deleted.
     *                  Must outlive the atlas.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        setSemanticsManager(SemanticsManager *p_semanticsManager_in);

    /*!
     * @brief           Removes a map from the active Atlas and marks it
     *                  invalid.
     *
     * @param[in,out]   p_map_inout
     *                  map::Map whose merge lifecycle has completed.
     */
    [[nodiscard]] AtlasStatus setMapBad(map::Map *p_map_inout);

    /*!
     * @brief           Captures the room geometry of the current map before it
     *                  is stranded by a restart, and records its final room
     *                  link.
     *
     *                  Runs on the tracking thread, which never waits for a
     *                  graph lock: it try-locks the map's graph shared and,
     *                  while the semantic cycle holds it, exports the context
     *                  the last cycle cached (see refreshRoomContextCache())
     *                  instead. Called internally from
     *                  \ref createNewMapWhileAtlasLocked, clearMap() and
     *                  clearAtlas() so that room identity survives map
     *                  transitions.
     */
    [[nodiscard]] AtlasStatus exportRoomContextFromCurrentMap();

    /*!
     * @brief           Caches the room context of the current map as plain
     *                  data, for exportRoomContextFromCurrentMap() to use while
     *                  a cycle holds the graph lock. The semantics manager
     *                  calls it at the end of every cycle, still holding the
     *                  graph locks. Takes the graph lock shared (re-entrant),
     *                  then roomContextMutex.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus refreshRoomContextCache();

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
        map::Map                                   *p_map_in,
        std::vector<semantic::RoomContextSnapshot> &roomContexts_out);

    /*!
     * @brief           Copies the newest departed-map snapshot for one stable
     *                  room identity.
     */
    [[nodiscard]] AtlasStatus copyLatestRoomContext(
        int                                           roomId_in,
        std::optional<semantic::RoomContextSnapshot> &roomContext_out) const;

    /*!
     * @brief           Returns and clears the map-created lifecycle event.
     */
    [[nodiscard]] AtlasStatus
        consumeNewMapCreatedEvent(bool &wasEventPending_out);

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

    /*!
     * @brief           Returns the world "up" direction of the active map, the
     *                  one axis every plane orientation test is judged
     *                  against. Takes atlasMutex.
     *
     *                  Plain RGB-D (never reads IMU state): -Y of the camera
     *                  that started the map, since the map frame is that
     *                  camera's frame and camera Y points down. This is only
     *                  vertical if that camera was level. Inertial: the active
     *                  map's own up (Map::getUpDirection()), which starts as
     *                  that -Y, follows every whole-map rotation and is set
     *                  from the IMU gravity estimate at IMU initialisation and
     *                  scale refinement. The IMU-initialised flag alone is not
     *                  used: an inertial merge sets it without a gravity
     *                  rotation.
     *
     * @param[out]      upDirection_world_out
     *                  Unit vector pointing up, in the active map's world
     *                  frame.
     *
     * @param[out]      isFromImuGravity_out
     *                  True when the direction comes from an IMU gravity
     *                  estimate, false when it is the map-start camera's -Y.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus
        getUpDirection(Eigen::Vector3d &upDirection_world_out,
                       bool            &isFromImuGravity_out);

    /* ---------------------------------------------------------------------- *
     * FUNCTIONS TO GAURANTEE THE CORRECTION OF SERIALIZATION OF ATLAS
     * ---------------------------------------------------------------------- */

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

    /* ---------------------------------------------------------------------- *
     * FUNCTIONS FOR GETTING THE ENTITIES
     * ---------------------------------------------------------------------- */

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
    [[nodiscard]] AtlasStatus
        getKeyFrameById(long unsigned int    idCount_in,
                        keyframe::KeyFrame *&p_keyFrameById_out);

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
     * @brief           Stores the key frame database that postLoad uses to
     *                  re-link key frames.
     *
     * @param[in]       p_keyFrameDatabase_in
     *                  Database to remember; borrowed, the Atlas never deletes
     *                  it.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus setKeyFrameDatabase(
        keyframe_database::KeyFrameDatabase *p_keyFrameDatabase_in);

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

  protected:
    /*!
     * @brief           Creates the next map while the caller owns atlasMutex.
     */
    [[nodiscard]] AtlasStatus createNewMapWhileAtlasLocked();

    /*!
     * @brief           Removes the scene graph registry entry of every given
     *                  map from the semantics manager; a no-op while no
     *                  manager is attached.
     *
     * @param[in]       mapsToUnregister_in
     *                  Maps leaving the atlas; borrowed, null entries are
     *                  skipped.
     *
     * @return          ATLAS_STATUS_SUCCESS always.
     */
    [[nodiscard]] AtlasStatus unregisterSceneGraphs(
        const std::vector<map::Map *> &mapsToUnregister_in);

    /*!
     * @brief           Active maps, owned by the Atlas and keyed by map id, so
     *                  every iteration runs in ascending map-id order. Guarded
     *                  by atlasMutex.
     */
    std::map<unsigned long, std::unique_ptr<map::Map>> maps;

    /*!
     * @brief           Maps marked bad by setMapBad that removeBadMaps has not
     *                  yet moved to retiredMaps; still owned by the Atlas, in
     *                  the order they went bad.
     */
    std::vector<std::unique_ptr<map::Map>> badMaps;

    /*!
     * @brief           Maps retired from active use (by removeBadMaps,
     *                  clearAtlas or postLoad) but still owned until Atlas
     *                  destruction, in the order they were retired.
     */
    std::vector<std::unique_ptr<map::Map>> retiredMaps;

    /*!
     * @brief           Maps sorted by id, filled by preSave for serialisation
     *                  and emptied by postLoad. A vector rather than a set
     *                  because libboost 1.58 on Ubuntu 16.04 fails on a
     *                  serialised set.
     */
    std::vector<map::Map *> backupMaps;

    /*!
     * @brief           The current map, or nullptr before the first map exists
     *                  and after clearAtlas. Borrowed from maps; guarded by
     *                  atlasMutex.
     */
    map::Map *p_activeMap;

    /*!
     * @brief           Distinct camera models used by the key frames;
     *                  serialised; guarded by atlasMutex in getAllCameras and
     *                  addCamera. Borrowed, never deleted through this list:
     *                  Settings or Tracking own the cameras they created and
     *                  registered, loadedCameras owns those Boost created when
     *                  this Atlas was loaded.
     */
    std::vector<camera_models::geometric_camera::GeometricCamera *> cameras;

    /*!
     * @brief           Owner of the cameras Boost created while loading this
     *                  Atlas (the content of cameras right after loading);
     *                  freed with the Atlas. Not serialised.
     */
    std::vector<
        std::unique_ptr<camera_models::geometric_camera::GeometricCamera>>
        loadedCameras;

    /*!
     * @brief           Key frame id at which the newest map was initialised;
     *                  serialised. createNewMap and preSave advance it.
     */
    unsigned long int lastInitKeyFrameId{0};

    /*!
     * @brief           Viewer given to setViewer; borrowed and only meaningful
     *                  once hasViewer is true.
     */
    Viewer *p_viewer{nullptr};

    /*!
     * @brief           True once setViewer has stored a viewer.
     */
    bool hasViewer{false};

    /*!
     * @brief           Key frame database that postLoad hands to the loaded
     *                  maps; borrowed, nullptr until setKeyFrameDatabase.
     */
    keyframe_database::KeyFrameDatabase *p_keyFrameDatabase{nullptr};

    /*!
     * @brief           ORB vocabulary that postLoad hands to the loaded maps;
     *                  borrowed, nullptr until setORBVocabulary.
     */
    ORBVocabulary *p_orbVocabulary{nullptr};

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
     * @brief           Semantics manager that serves the S-Graph of the maps in
     *                  maps; borrowed, owned by the system. The atlas owns no
     *                  graph content: each map owns its S-Graph. A map's scene
     *                  graph is registered with the manager when the map is
     *                  created or loaded and unregistered when the map leaves
     *                  maps (setMapBad, clearAtlas, postLoad) or the atlas is
     *                  destroyed. Null from construction until
     *                  setSemanticsManager(), which registers the maps created
     *                  or loaded before. The manager guards its graph with its
     *                  own graph mutex, taken after atlasMutex.
     */
    SemanticsManager *p_semanticsManager{nullptr};

    /*!
     * @brief           Snapshots of departed maps' room geometry, keyed by
     *                  map::Map::GetId().
     */
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
        roomContextHistory;

    /*!
     * @brief           Room context of the current map as of the end of the
     *                  last semantic cycle; plain data. Guarded by
     *                  roomContextMutex, which is a leaf lock.
     */
    RoomContextExport cachedRoomContext;

    /*!
     * @brief           Protects roomContextHistory and cachedRoomContext
     *                  against concurrent access from tracking and the semantic
     *                  worker thread. A leaf lock: nothing else is taken while
     *                  it is held.
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

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_ATLAS_OBJECTS_ATLAS_CLASS_H */
