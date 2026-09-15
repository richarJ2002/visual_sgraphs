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
namespace semantic { class Room; }
class Atlas;
namespace geometric { class Plane; }
class Door;
namespace semantic { class Floor; }
namespace semantic { class Marker; }
namespace semantic { class Passage; }
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
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        // Save/load a set structure, the set structure is broken in
        // libboost 1.58 for ubuntu 16.04, a vector is serializated ar &
        // mspKeyFrames; ar & mspMapPoints;
        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(KeyFrame *pKF);
    void AddMapPoint(MapPoint *pMP);
    void AddMapPlane(geometric::Plane *pPlane);
    void AddMapMarker(semantic::Marker *pMarker);
    void AddDetectedMapRoom(semantic::Room *room);
    void AddCandidateMapRoom(semantic::Room *room);
    /** Atomically moves a room from candidate to detected storage. */
    void PromoteCandidateMapRoom(semantic::Room *room);
    void AddMapFloor(vs_graphs::core::semantic::Floor *pFloor);
    void AddMapDoor(vs_graphs::core::Door *pDoor);
    void AddRoomWallPlane(vs_graphs::core::geometric::Plane *pPlane);
    void AddMapPassage(vs_graphs::core::semantic::Passage *pPassage);

    /**
     * @brief Reserves a plane identifier that will not be reused by this map.
     *
     * @return Unique plane identifier for a subsequently added plane.
     */
    int reservePlaneId(void);

    /**
     * @brief Reserves a floor identifier that will not be reused by this map.
     *
     * @return Unique floor identifier for a subsequently added floor.
     */
    int reserveFloorId(void);

    void EraseMapPoint(MapPoint *pMP);
    void EraseKeyFrame(KeyFrame *pKF);
    void EraseMapPlane(geometric::Plane *pPlane);
    void EraseMapMarker(semantic::Marker *pMarker);
    void EraseDetectedMapRoom(semantic::Room *pRoom);
    void EraseMarkerBasedMapRoom(semantic::Room *pRoom);
    void EraseRoomWallPlane(vs_graphs::core::geometric::Plane *pPlane);

    /**
     * @brief Records the room this map started with (bootstrap entry room).
     *        Set once; non-owning, owned by this map.
     */
    void setStartingRoom(vs_graphs::core::semantic::Room *p_room_in);

    /** Returns the room this map started with, if any. */
    vs_graphs::core::semantic::Room *getStartingRoom();

    /**
     * @brief Records the last current room at departure (reset/export).
     *        Set on map transitions; non-owning, owned by this map.
     */
    void setFinalRoom(vs_graphs::core::semantic::Room *p_room_in);

    /** Returns the last current room at departure, if any. */
    vs_graphs::core::semantic::Room *getFinalRoom();

    /**
     * @brief Links the next map in the mission chain. Left null on
     *        same-map clears. Non-owning; valid only while the Atlas
     *        retains both maps.
     */
    void setFollowingMap(Map *p_map_in);

    /** Returns the next map in the mission chain, if any. */
    Map *getFollowingMap();

    void EraseMapPassage(vs_graphs::core::semantic::Passage *pPassage);
    void EraseMapFloor(vs_graphs::core::semantic::Floor *p_floor_in);

    /** Clears lookup-only state after every indexed entity was transferred. */
    void ClearTransferredEntityIndexes();

    void InformNewBigChange();
    int  GetLastBigChangeIdx();
    void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

    std::vector<semantic::Room *>               GetAllRooms();
    std::vector<geometric::Plane *>              GetAllPlanes();
    std::vector<semantic::Marker *>             GetAllMarkers();
    std::vector<KeyFrame *>           GetAllKeyFrames();
    std::vector<MapPoint *>           GetAllMapPoints();
    std::vector<semantic::Room *>               GetAllDetectedMapRooms();
    std::vector<vs_graphs::core::Door *>    GetAllDoors();
    std::vector<vs_graphs::core::semantic::Floor *>   GetAllFloors();
    std::vector<semantic::Room *>               GetAllMarkerBasedMapRooms();
    std::vector<semantic::Room *>               GetAllCandidateMapRooms();
    std::vector<MapPoint *>           GetReferenceMapPoints();
    std::vector<vs_graphs::core::semantic::Passage *> GetAllPassages();

    /**
     * @brief Get the cluster points of the map set by `voxblox_skeleton`
     */
    std::vector<std::vector<Eigen::Vector3d>> GetSkeletonClusterPoints(void);

    /**
     * @brief       Set the cluster points of the map set by `voxblox_skeleton`
     *
     * @param[in]   newClusterPoints
     *              The new cluster points to set
     */
    void SetSkeletonClusterPoints(
        const std::vector<std::vector<Eigen::Vector3d>> &newClusterPoints);

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

    long unsigned     KeyFramesInMap();
    long unsigned int MarkersInMap();
    long unsigned int MapPointsInMap();

    long unsigned int GetId();
    long unsigned int GetMaxKFid();
    long unsigned int GetInitKFid();
    void              SetInitKFid(long unsigned int initKFif);

    KeyFrame           *GetOriginKF();
    semantic::Floor              *GetFloorById(int floorId);
    Door               *GetDoorById(int doorId);
    geometric::Plane              *GetPlaneById(int planeId);
    semantic::Marker             *GetMarkerById(int markerId);
    KeyFrame           *GetKeyFrameById(long unsigned int mnId);
    vs_graphs::core::semantic::Passage *GetPassageById(int passageId);
    vs_graphs::core::geometric::Plane   *GetRoomWallPlaneById(int planeId);

    geometric::Plane *GetBiggestGroundPlane();

    void SetStoredMap();
    void SetCurrentMap();

    bool IsInUse();

    bool IsBad();
    void SetBad();

    void clear();

    int  GetLastMapChange();
    int  GetMapChangeIndex();
    /** Returns the epoch of the coordinate frame containing this map. */
    std::uint64_t GetWorldFrameEpoch();
    void IncreaseChangeIndex();
    void SetLastMapChange(int currentChangeId);

    bool isImuInitialized();
    void SetImuInitialized();

    void ApplyScaledRotation(const Sophus::SE3f &T,
                             const float         s,
                             const bool          bScaledVel = false);

    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();
    void SetInertialSensor();

    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<camera_models::GeometricCamera *> &spCams);
    void PostLoad(KeyFrameDatabase                     *pKFDB,
                  ORBVocabulary                        *pORBVoc,
                  map<unsigned int, camera_models::GeometricCamera *> &mpCams);

    KeyFrame                 *mpFirstRegionKF;
    std::mutex                mMutexMapUpdate;
    vector<KeyFrame *>        mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;

    // This avoid that two points are created simultaneously in separate threads
    // (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH  = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

  protected:
    long unsigned int mnId;

    std::set<semantic::Floor *>              mspFloors;
    std::set<Door *>               mspDoors;
    std::set<geometric::Plane *>              mspPlanes;
    std::set<semantic::Marker *>             mspMarkers;
    std::set<MapPoint *>           mspMapPoints;
    std::set<KeyFrame *>           mspKeyFrames;
    std::set<semantic::Room *>               mspDetectedRooms;
    std::set<semantic::Room *>               mspMarkerBasedRooms;
    std::set<vs_graphs::core::semantic::Passage *> mspPassages;

    // Skeleton cluster points of the map set by `voxblox_skeleton`
    std::vector<std::vector<Eigen::Vector3d>> skeletonClusterPoints;

    /*!
     * @brief       Latest connected Voxblox skeleton graph edges.
     */
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> mSkeletonEdges;

    // Hashmaps and indices for fetching elements
    std::unordered_map<int, semantic::Floor *>                  mFloorIndex;
    std::unordered_map<int, Door *>                   mDoorIndex;
    std::unordered_map<int, geometric::Plane *>                  mPlaneIndex;
    std::unordered_map<int, semantic::Marker *>                 mMarkerIndex;
    std::unordered_map<long unsigned int, KeyFrame *> mKFIndex;
    std::unordered_map<int, vs_graphs::core::semantic::Passage *>     mPassageIndex;
    std::unordered_map<int, vs_graphs::core::geometric::Plane *>       mRoomWallPlaneIndex;

    /* Monotonic semantic IDs remain unique after fusion creates index gaps. */
    int nextAvailablePlaneId{0};
    int nextAvailableFloorId{0};

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04,
    // a vector is serializated
    std::vector<MapPoint *> mvpBackupMapPoints;
    std::vector<KeyFrame *> mvpBackupKeyFrames;

    KeyFrame *mpKFinitial;
    KeyFrame *mpKFlowerID;

    /* Mission-chain trace links. Rooms are owned by this map (shared
     * lifetime); the following map is Atlas-owned (see setters). */
    vs_graphs::core::semantic::Room *p_startingRoom{nullptr};
    vs_graphs::core::semantic::Room *p_finalRoom{nullptr};
    Map             *p_followingMap{nullptr};

    unsigned long int mnBackupKFlowerID;
    unsigned long int mnBackupKFinitialID;

    std::vector<MapPoint *> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    /**
     * Monotonic coordinate-frame epoch. Unlike mnMapChange, ordinary local BA
     * and content updates do not increment it; whole-map rebases do.
     */
    std::uint64_t mnWorldFrameEpoch;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte *mThumbnail;

    bool             mIsInUse;
    bool             mHasTumbnail;
    std::atomic_bool mbBad{false};

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    // Mutex
    std::mutex mMutexMap;
};

} // namespace core
} // namespace vs_graphs

#endif
