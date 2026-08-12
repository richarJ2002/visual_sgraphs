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

#ifndef ATLAS_H
#define ATLAS_H

#include "Geometric/Plane.h"
#include "GeometricCamera.h"
#include "KannalaBrandt8.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Pinhole.h"
#include "Semantic/Floor.h"
#include "Semantic/Marker.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"

#include <Eigen/Core>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <map>
#include <mutex>
#include <set>
#include <vector>

namespace ORB_SLAM3
{
class Map;
class Room;
class Frame;
class Plane;
class Floor;
class Viewer;
class Marker;
class Pinhole;
class Passage;
class MapPoint;
class KeyFrame;
class KannalaBrandt8;
class KeyFrameDatabase;

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
struct RoomContextSnapshot
{
    int             roomId;   //!< Room::getId()
    Eigen::Vector3d centroid; //!< Room centroid (world frame)
    std::vector<Eigen::Vector3d>
        wallNormals; //!< Oriented wall normals toward room
    std::vector<Eigen::Vector3d> wallCentroids; //!< Per-wall centroids
    std::vector<double> wallDistances; //!< Per-wall `g2o::Plane3D::distance()`
    std::vector<Eigen::Vector3d>
           passageCentroids; //!< Detected passage openings
    double timestamp;        //!< When the snapshot was taken (s)
    std::string roomTag;     //!< Persistent identity tag "room_<id>" for cross-restart matching
};

class Atlas
{
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar.template register_type<Pinhole>();
        ar.template register_type<KannalaBrandt8>();

        // Save/load a set structure, the set structure is broken in
        // libboost 1.58 for ubuntu 16.04, a vector is serializated ar &
        // mspMaps;
        ar & mvpBackupMaps;
        ar & mvpCameras;
        // Need to save/load the static Id from Frame, KeyFrame, MapPoint and
        // Map
        ar &Map::nNextId;
        ar &Frame::nNextId;
        ar &KeyFrame::nNextId;
        ar &MapPoint::nNextId;
        ar &GeometricCamera::nNextId;
        ar & mnLastInitKFidMap;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Atlas();
    Atlas(int initKFid); // When its initialization the first map is created
    ~Atlas();

    void CreateNewMap();
    void ChangeMap(Map *pMap);

    unsigned long int GetLastInitKFid();

    void SetViewer(Viewer *pViewer);

    // Methods for adding new components in the current map
    void AddMapFloor(Floor *floor);
    void AddMapPlane(Plane *plane);
    void AddKeyFrame(KeyFrame *pKF);
    void AddMapPoint(MapPoint *pMP);
    void AddMapMarker(Marker *marker);
    void AddDetectedMapRoom(Room *room);
    void AddCandidateMapRoom(Room *room);
    void AddMapPassage(ORB_SLAM3::Passage *passage);
    void AddRoomWallPlane(ORB_SLAM3::Plane *pPlane);

    std::vector<GeometricCamera *> GetAllCameras();
    GeometricCamera               *AddCamera(GeometricCamera *pCam);

    /* All methods without Map pointer work on current map */
    void InformNewBigChange();
    int  GetLastBigChangeIdx();
    void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

    long unsigned     MarkersInMap();
    long unsigned     KeyFramesInMap();
    long unsigned int MapPointsInMap();

    // List of marker-ids placed on planes detected so far
    std::vector<int> visitedPlanesMarkerIds;

    // Method for get data in current map
    std::vector<Room *>               GetAllRooms();
    std::vector<Floor *>              GetAllFloors();
    std::vector<Marker *>             GetAllMarkers();
    std::vector<KeyFrame *>           GetAllKeyFrames();
    std::vector<MapPoint *>           GetAllMapPoints();
    std::vector<Room *>               GetAllDetectedMapRooms();
    std::vector<ORB_SLAM3::Plane *>   GetAllPlanes();
    std::vector<Room *>               GetAllMarkerBasedMapRooms();
    std::vector<Room *>               GetAllCandidateMapRooms();
    std::vector<MapPoint *>           GetReferenceMapPoints();
    std::vector<ORB_SLAM3::Passage *> GetAllPassages();

    /*!
     * @brief Get the cluster points of the map set by `voxblox_skeleton`
     */
    std::vector<std::vector<Eigen::Vector3d>> GetSkeletoClusterPoints();

    /*!
     * @brief       Gets the latest connected Voxblox skeleton edges.
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        GetSkeletonEdges(void);

    /*!
     * @brief       Stores the latest connected Voxblox skeleton edges.
     */
    void SetSkeletonEdges(
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
            &newSkeletonEdges);

    /*!
     * @brief       Set the cluster points of the map set by `voxblox_skeleton`
     *
     * @param[in]   newClusterPoints
     *              The new cluster points to set
     */
    void SetSkeletonClusterPoints(
        const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints);

    Plane *GetBiggestGroundPlane();

    vector<Map *> GetAllMaps();

    /**
     * @brief Checks whether a map is still an active, non-retired Atlas map.
     *
     * @param[in] p_map_in Map pointer to validate.
     * @return True only while the map is active and not marked bad.
     */
    bool isActiveMap(Map *p_map_in);

    int CountMaps();

    void clearMap();

    void clearAtlas();

    Map *GetCurrentMap();

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

    /**
     * @brief Removes a map from the active Atlas and marks it invalid.
     *
     * @param[in] p_map_in Map whose merge lifecycle has completed.
     */
    void SetMapBad(Map *p_map_in);

    /**
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
     * @param[in] p_currentMap_in Map that survives the merge and remains
     *                            active.
     * @param[in] p_otherMap_in   Map to be transformed, absorbed, then marked
     *                            bad.
     */
    void MergeMapPair(Map *p_currentMap_in, Map *p_otherMap_in);

    /*!
     * @brief Captures the room geometry of the current map before it is
     *        stranded by a restart.
     *
     * Called internally from \ref createNewMapWhileAtlasLocked and
     * \ref clearAtlas so that room identity survives map transitions.
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
     * @param[in] pNewMap Map whose rooms should be examined/tagged.
     */
    void matchRoomsToContext(Map *pNewMap);

    /*!
     * @brief Returns the vector of context snapshots stored for \p mapId.
     */
    const std::vector<RoomContextSnapshot> &
        getRoomContextForMap(long unsigned int mapId) const;

    /**
     * @brief Moves invalid maps out of the transient retirement queue.
     *
     * Retired maps remain owned by the Atlas until shutdown. Delayed
     * reclamation avoids invalidating raw map pointers which can still be held
     * by tracking or visualization readers after a merge.
     */
    void RemoveBadMaps();

    bool isInertial();
    void SetInertialSensor();
    void SetImuInitialized();
    bool isImuInitialized();

    // Function for garantee the correction of serialization of this object
    void PreSave();
    void PostLoad();

    map<long unsigned int, KeyFrame *> GetAtlasKeyframes();

    // Functions for getting the entities
    Plane              *GetPlaneById(int planeId);
    Floor              *GetFloorById(int floorId);
    Marker             *GetMarkerById(int markerId);
    KeyFrame           *GetKeyFrameById(long unsigned int mnId);
    ORB_SLAM3::Passage *GetPassageById(int passageId);
    ORB_SLAM3::Plane   *GetRoomWallPlaneById(int planeId);

    KeyFrameDatabase *GetKeyFrameDatabase();
    void              SetKeyFrameDababase(KeyFrameDatabase *pKFDB);

    ORBVocabulary *GetORBVocabulary();
    void           SetORBVocabulary(ORBVocabulary *pORBVoc);

    long unsigned int GetNumLivedKF();
    long unsigned int GetNumLivedMP();

  protected:
    /**
     * @brief Creates the next map while the caller owns mMutexAtlas.
     */
    void createNewMapWhileAtlasLocked();

    std::set<Map *> mspMaps;
    std::set<Map *> mspBadMaps;

    /** Maps retired from active use but still owned until Atlas destruction. */
    std::set<Map *> mspRetiredMaps;

    // Its necessary change the container from set to vector because
    // libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
    std::vector<Map *> mvpBackupMaps;

    Map *mpCurrentMap;

    std::vector<GeometricCamera *> mvpCameras;

    unsigned long int mnLastInitKFidMap;

    Viewer *mpViewer;
    bool    mHasViewer;

    // Class references for the map reconstruction from the save file
    KeyFrameDatabase *mpKeyFrameDB;
    ORBVocabulary    *mpORBVocabulary;

    // Mutex
    std::mutex mMutexAtlas;

    /*!
     * @brief Serialises semantic graph updates with map-merge transactions.
     */
    std::mutex mMutexSemanticUpdate;

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
    std::map<long unsigned int, std::vector<RoomContextSnapshot>>
        mRoomContextHistory;

    /*!
     * @brief Protects \ref mRoomContextHistory against concurrent access from
     *        tracking and the semantic worker thread.
     */
    std::mutex mRoomContextMutex;
};

} // namespace ORB_SLAM3

#endif
