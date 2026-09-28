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

#ifndef ATLAS_H
#define ATLAS_H

#include "AtlasCurrentMapStatus.h"
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
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
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
 * @brief Per-old-map bookkeeping for continuous consecutive-map matching.
 *
 * Lives in the Atlas (not in either map) because it describes a map PAIR
 * relationship across semantic cycles: when the pair was last attempted
 * and what the content looked like then, so unchanged pairs are not
 * re-attempted every cycle.
 */
struct MergeAttemptState
{
    std::chrono::steady_clock::time_point lastAttemptTime{};
    std::size_t                           contentHashAtLastAttempt{0U};
    bool                                  hasEverAttempted{false};
};

/*!
 * @brief Serialisable snapshot of a room's geometric context at the moment a
 *        map is abandoned.
 *
 * Captured before the old map is stranded, these snapshots let the Atlas
 * re-identify the corresponding physical room when a fresh map starts filling
 * in from skeleton clustering. Wall normals, centroids, and plane distances
 * are stored so that a best-match comparison can verify the room identity
 * rather than relying on a fragile centroid-only heuristic.
 */
class Atlas
{
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar.template register_type<camera_models::pinhole::Pinhole>();
        ar.template register_type<
            camera_models::kannalabrandt8::KannalaBrandt8>();

        // Save/load a set structure, the set structure is broken in
        // libboost 1.58 for ubuntu 16.04, a vector is serializated ar &
        // mspMaps;
        ar & backupMaps;
        ar & cameras;
        // Need to save/load the static Id from Frame, KeyFrame, MapPoint and
        // Map
        ar &Map::nextId;
        ar &Frame::nextId;
        ar &KeyFrame::nextId;
        ar &MapPoint::nextId;
        ar &camera_models::geometriccamera::GeometricCamera::nextId;
        ar & lastInitKeyFrameId;
    }

  public:
    enum class SnapshotCopyStatus
    {
        COMPLETE,
        UNAVAILABLE_LIVE_GENERATION
    };

    struct SnapshotCopyResult
    {
        SnapshotCopyStatus status{
            SnapshotCopyStatus::UNAVAILABLE_LIVE_GENERATION};
        std::vector<semantic::RoomContextSnapshot> snapshots;
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Atlas();
    Atlas(int initialKeyFrameId_in); // When its initialization the first map is
                                     // created
    ~Atlas();

    void createNewMap();
    void changeMap(Map *p_map_in);

    unsigned long int getLastInitKeyFrameId();

    void setViewer(Viewer *p_viewer_in);

    // Methods for adding new components in the current map
    void addMapFloor(semantic::Floor *p_floor_in);
    void addMapPlane(geometric::Plane *p_plane_in);
    void addKeyFrame(KeyFrame *p_keyFrame_in);
    void addMapPoint(MapPoint *p_mapPoint_in);
    void addMapMarker(semantic::Marker *p_marker_in);
    void addDetectedMapRoom(semantic::Room *p_room_in);
    void addCandidateMapRoom(semantic::Room *p_room_in);
    void addMapPassage(vs_graphs::core::semantic::Passage *p_passage_in);
    void addRoomWallPlane(vs_graphs::core::geometric::Plane *p_plane_in);

    /*! Reserves the next mission-stable room identity, beginning at zero. */
    int reserveRoomIdentity(void);

    /*! Reserves the next mission-stable passage identity, beginning at zero. */
    int reservePassageIdentity(void);

    /*! Reserves the next mission-stable floor identity, beginning at zero. */
    int reserveFloorIdentity(void);

    /*! Advances the room allocator past an explicitly restored identity. */
    void observeRoomIdentity(int roomId_in);

    /*! Advances the passage allocator past an explicitly restored identity. */
    void observePassageIdentity(int passageId_in);

    /*! Advances the floor allocator past an explicitly restored identity. */
    void observeFloorIdentity(int floorId_in);

    /*! Stores the mission-stable identity occupied by the camera. */
    void setCurrentSemanticRoomIdentity(int roomId_in);

    /*! Returns the last mission-stable room identity, or -1 before bootstrap.
     */
    int getCurrentSemanticRoomIdentity(void) const;

    std::vector<camera_models::geometriccamera::GeometricCamera *>
        getAllCameras();
    camera_models::geometriccamera::GeometricCamera *
        addCamera(camera_models::geometriccamera::GeometricCamera *p_camera_in);

    /* All methods without Map pointer work on current map */
    void informNewBigChange();
    int  getLastBigChangeIndex();
    void setReferenceMapPoints(const std::vector<MapPoint *> &mapPoints_in);

    long unsigned     getMarkerCount();
    long unsigned     getKeyFrameCount();
    long unsigned int getMapPointCount();

    // List of marker-ids placed on planes detected so far
    std::vector<int> visitedPlanesMarkerIds;

    // Method for get data in current map
    std::vector<semantic::Room *>                    getAllRooms();
    std::vector<semantic::Floor *>                   getAllFloors();
    std::vector<semantic::Marker *>                  getAllMarkers();
    std::vector<KeyFrame *>                          getAllKeyFrames();
    std::vector<MapPoint *>                          getAllMapPoints();
    std::vector<semantic::Room *>                    getAllDetectedMapRooms();
    std::vector<vs_graphs::core::geometric::Plane *> getAllPlanes();
    std::vector<semantic::Room *> getAllMarkerBasedMapRooms();
    std::vector<semantic::Room *> getAllCandidateMapRooms();
    std::vector<MapPoint *>       getReferenceMapPoints();
    std::vector<vs_graphs::core::semantic::Passage *> getAllPassages();

    /*!
     * @brief Get the cluster points of the map set by `voxblox_skeleton`
     */
    std::vector<std::vector<Eigen::Vector3d>> getSkeletonClusterPoints();

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

    /*!
     * @brief       Set the cluster points of the map set by `voxblox_skeleton`
     *
     * @param[in]   newClusterPoints_in
     *              The new cluster points to set
     */
    void setSkeletonClusterPoints(
        const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints_in);

    geometric::Plane *getBiggestGroundPlane();

    vector<Map *> getAllMaps();

    /*!
     * @brief Checks whether a map is still an active, non-retired Atlas map.
     *
     * @param[in] p_map_in Map pointer to validate.
     * @return True only while the map is active and not marked bad.
     */
    bool isActiveMap(Map *p_map_in);

    int countMaps();

    void clearMap();

    void clearAtlas();

    Map *getCurrentMap();

    /*!
     * @brief       Returns the current map's id, its truthful status
     *              relative to the active map set, and every active Atlas
     *              map, as one coherent, non-mutating value view.
     *
     *              Unlike GetCurrentMap(), this method never creates a map:
     *              an Atlas with no current map (e.g. immediately after
     *              clearAtlas(), or before the first CreateNewMap()) reports
     *              an empty \p currentMapId_out, \p currentMapStatus_out ==
     *              AtlasCurrentMapStatus::NO_CURRENT_MAP, and an empty
     *              returned vector, rather than fabricating a new map as a
     *              read side effect. All three facts are read under one
     *              atlasMutex critical section, so no concurrent
     *              ChangeMap()/SetMapBad()/map add or remove can produce a
     *              torn read across them -- the two-call race possible with
     *              separate GetCurrentMap() and GetAllMaps() calls cannot
     *              occur here. This is a narrower guarantee than "the
     *              current map always names an active map": Atlas's own
     *              state can already be transiently incoherent independent
     *              of any race, because SetMapBad(p_map_in) erases
     *              \p p_map_in from the active set and marks it bad without
     *              clearing p_activeMap -- a later ChangeMap() call is what
     *              eventually installs a replacement current map. When
     *              \p p_map_in == the current map, every call to this method
     *              between those two events truthfully reports
     *              \p currentMapId_out naming a map absent from the returned
     *              vector, via \p currentMapStatus_out ==
     *              AtlasCurrentMapStatus::CURRENT_MAP_NOT_ACTIVE, rather than
     *              silently claiming a consistency invariant that does not
     *              hold at that instant. This method never busy-waits,
     *              unlike GetCurrentMap().
     *
     * @param[in,out] currentMapId_inout
     *              Set to the current map's id, or left empty when
     *              p_activeMap is null at the moment of the read.
     * @param[out]  currentMapStatus_out
     *              Set to NO_CURRENT_MAP, CURRENT_MAP_ACTIVE, or
     *              CURRENT_MAP_NOT_ACTIVE; see AtlasCurrentMapStatus.h.
     * @return      Every active Atlas map, sorted by id -- identical
     *              content and order to GetAllMaps().
     */
    std::vector<Map *>
        getCoherentMapView(std::optional<long unsigned int> &currentMapId_inout,
                           AtlasCurrentMapStatus &currentMapStatus_out);

    /*!
     * @brief       Acquires exclusive access to semantic-map mutations.
     *
     *              Loop closing holds this lock while semantic entities are
     *              transformed, transferred, and fused. Semantic worker
     *              threads use the same lock before changing the graph, which
     *              prevents guarded writers from interleaving mutations.
     *
     * @return      Movable lock which releases the semantic transaction when
     *              it leaves scope.
     */
    std::unique_lock<std::mutex> acquireSemanticUpdateLock();

    /*!
     * @brief Removes a map from the active Atlas and marks it invalid.
     *
     * @param[in,out] p_map_inout Map whose merge lifecycle has completed.
     */
    void setMapBad(Map *p_map_inout);

    /*!
     * @brief Merges the semantic graph of \p p_otherMap_in into
     *        \p p_currentMap_in.
     *
     *        The current map survives and keeps its authoritative frame. The
     *        other map's keyframes, map points, planes, markers, passages,
     *        detected rooms, marker-based rooms, and floors are transformed
     *        into the current map's frame using Horn's closed-form solution on
     *        corresponding wall normals and centroids, transferred into the
     *        current map, fused, and re-associated. The other map is then
     *        marked bad.
     *
     *        Deterministic: Horn's closed-form method, no iteration.
     *
     *        The caller must already hold the semantic-update lock (see
     *        \ref acquireSemanticUpdateLock); this method does not acquire it.
     *
     * @param[in,out] p_currentMap_inout Map that survives the merge and remains
     *                            active.
     * @param[in,out] p_otherMap_inout   Map to be transformed, absorbed, then
     * marked bad.
     */
    void mergeMapPair(Map *p_currentMap_inout, Map *p_otherMap_inout);

    /*!
     * @brief Attempts validated consecutive-map merges, old into current.
     *
     * Runs the continuous shape-matching scheduler over every non-bad,
     * non-current map: room-prior seed gate, minimum-shape gate,
     * cooldown/change gate, shared-Horn transform estimation, then the
     * consecutive merge gate. On ACCEPT the old map is fused via
     * MergeMapPair (which retires it); on anything else nothing mutates.
     * At most one merge commits per call.
     *
     * The caller must already hold the semantic-update lock (see
     * \ref acquireSemanticUpdateLock); this method does not acquire it.
     */
    void attemptConsecutiveMergeIfGated(void);

    /*!
     * @brief Captures the room geometry of the current map before it is
     *        stranded by a restart.
     *
     * Called internally from \ref createNewMapWhileAtlasLocked,
     * \ref clearMap, and \ref clearAtlas so that room identity survives map
     * transitions.
     */
    void exportRoomContextFromCurrentMap();

    /*!
     * @brief Transfers room identity tags from accumulated context snapshots
     *        to untagged rooms in \p pNewMap.
     *
     * Uses a best-match strategy: the room whose centroid is closest to a
     * snapshot centroid — provided the distance is below
     * \ref kRoomContextMatchThreshold_m and wall normals agree — inherits
     * that snapshot's room identity tag.
     *
     * @param[in] p_newMap_in Map whose rooms should be examined/tagged.
     */
    void matchRoomsToContext(Map *p_newMap_in);

    /*! Copies room history without exposing references beyond the lock scope.
     */
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
        copyRoomContextHistory() const;

    /*! Copies invariant context from a live map under the semantic transaction.
     */
    std::vector<semantic::RoomContextSnapshot>
        copyRoomContextForMap(Map *p_map_in);

    /*! Copies the newest departed-map snapshot for one stable room identity. */
    std::optional<semantic::RoomContextSnapshot>
        copyLatestRoomContext(int roomId_in) const;

    /*! Copies live entities only when the caller owns the semantic transaction.
     */
    SnapshotCopyResult
        copyRoomContextForMapChecked(Map *p_map_inout,
                                     bool callerOwnsSemanticLock_in);

    /*! Returns and clears the map-created lifecycle event. */
    bool consumeNewMapCreatedEvent();

    /*! Reads the lifecycle event without acknowledging it. */
    bool peekNewMapCreatedEvent() const;

    /*! Acknowledges the pending lifecycle event after verified handling. */
    void acknowledgeNewMapCreatedEvent();

    /*!
     * @brief Returns the vector of context snapshots stored for \p mapId.
     */
    const std::vector<semantic::RoomContextSnapshot> &
        getRoomContextForMap(long unsigned int mapId_in) const;

    /*!
     * @brief Moves invalid maps out of the transient retirement queue.
     *
     * Retired maps remain owned by the Atlas until shutdown. Delayed
     * reclamation avoids invalidating raw map pointers which can still be held
     * by tracking or visualization readers after a merge.
     */
    void removeBadMaps();

    bool isInertial();
    void setInertialSensor();
    void setImuInitialized();
    bool isImuInitialized();

    // Function for garantee the correction of serialization of this object
    void preSave();
    void postLoad();

    map<long unsigned int, KeyFrame *> getAtlasKeyFrames();

    // Functions for getting the entities
    geometric::Plane *getPlaneById(int planeId_in);
    semantic::Floor  *getFloorById(int floorId_in);
    semantic::Marker *getMarkerById(int markerId_in);
    KeyFrame         *getKeyFrameById(long unsigned int idCount_in);
    vs_graphs::core::semantic::Passage *getPassageById(int passageId_in);
    vs_graphs::core::geometric::Plane  *getRoomWallPlaneById(int planeId_in);

    KeyFrameDatabase *getKeyFrameDatabase();
    void setKeyFrameDatabase(KeyFrameDatabase *p_keyFrameDatabase_in);

    ORBVocabulary *getORBVocabulary();
    void           setORBVocabulary(ORBVocabulary *p_orbVocabulary_in);

    long unsigned int getLivedKeyFrameCount();
    long unsigned int getLivedMapPointCount();

  protected:
    /*!
     * @brief Creates the next map while the caller owns atlasMutex.
     */
    void createNewMapWhileAtlasLocked();

    std::set<Map *> maps;
    std::set<Map *> badMaps;

    /*! Maps retired from active use but still owned until Atlas destruction. */
    std::set<Map *> retiredMaps;

    // Its necessary change the container from set to vector because
    // libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
    std::vector<Map *> backupMaps;

    Map *p_activeMap;

    std::vector<camera_models::geometriccamera::GeometricCamera *> cameras;

    unsigned long int lastInitKeyFrameId;

    Viewer *p_viewer;
    bool    hasViewer;

    // Class references for the map reconstruction from the save file
    KeyFrameDatabase *p_keyFrameDatabase;
    ORBVocabulary    *p_orbVocabulary;

    // Mutex
    std::mutex atlasMutex;

    /*!
     * @brief Serialises semantic graph updates with map-merge transactions.
     */
    std::mutex semanticUpdateMutex;

    /*!
     * @brief Best-match centroid distance (m) below which a room in a new map
     *        is considered the same physical room as a prior-map snapshot.
     */
    static constexpr double kRoomContextMatchThreshold_m = 2.0;

    /*!
     * @brief Minimum |cos θ| between wall normals for two wall hypotheses to
     *        be considered the same physical surface during context matching.
     */
    static constexpr double kWallNormalAlignmentCosTheta = 0.85;

    /*!
     * @brief Snapshots of departed maps' room geometry, keyed by Map::GetId().
     */
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>>
        roomContextHistory;

    /*!
     * @brief Continuous-match bookkeeping per old map ID. Guarded by
     *        atlasMutex like the other Atlas-owned indexes.
     */
    std::map<long unsigned int, MergeAttemptState> consecutiveMergeState;

    /*!
     * @brief Protects \ref mRoomContextHistory against concurrent access from
     *        tracking and the semantic worker thread.
     */
    mutable std::mutex roomContextMutex;

    /*! Mission-wide semantic allocators survive active-map replacement. */
    std::atomic<int> nextRoomIdentity{0};
    std::atomic<int> nextPassageIdentity{0};
    std::atomic<int> nextFloorIdentity{0};

    /*! Last occupied semantic identity; map-local temporal resets do not clear
     * it. */
    std::atomic<int> currentSemanticRoomIdentity{-1};

    /*! Pending lifecycle event, protected by roomContextMutex. */
    bool isNewMapCreatedPending{false};
};

} // namespace core
} // namespace vs_graphs

#endif
