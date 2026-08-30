/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#ifndef SEMANTICSMANAGER_H
#define SEMANTICSMANAGER_H

#include "Atlas.h"
#include "GeoSemHelpers.h"
#include "Semantic/RoomTracker.h"
#include "Utils.h"

#include <cstdint>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>
#include <unordered_set>

namespace ORB_SLAM3
{
class Atlas;

class SemanticsManager
{
  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       This member contains the address of semantic map.
     */
    Atlas *mpAtlas;

    /*!
     * @brief       Serializes updates to the set of newly detected rooms.
     */
    std::mutex mMutexNewRooms;

    /*!
     * @brief       Serializes updates to the current/last-known room ids.
     */
    mutable std::mutex mMutexCurrentRoom;

    // Shutdown control (LocalMapping-style handshake)
    std::mutex mMutexFinish;
    bool       mbFinishRequested = false;
    bool       mbFinished        = false;
    bool       CheckFinish();
    void       SetFinish();

    /*!
     * @brief       Room-state machine implementing the WP13 Section 18.2
     *              transition table.
     *
     *              The tracker is a parallel, read-only observer of the
     *              currentRoomId_/lastKnownRoomId_ bookkeeping; Phase 1 does
     *              not move those ids, it just records state transitions and
     *              guards.
     */
    RoomTracker roomTracker_;

    /*!
     * @brief       Set by updateTraversalEvidence() when a passable passage
     *              crossing was observed during this semantic cycle.
     */
    bool crossingEventPending_ = false;

    /*!
     * @brief       Set by onTrackingLost() (once per loss episode) and
     *              consumed by the room tracker during the next Run cycle.
     */
    bool trackingLostPending_ = false;

    /*!
     * @brief       Id of the room the camera most recently occupied.
     *
     *              -1 until the first confirmed room has been resolved.
     */
    int currentRoomId_{-1};

    /*!
     * @brief       Id of the room known just before the current tracking loss.
     *
     *              -1 when tracking has not yet been lost.
     */
    int lastKnownRoomId_{-1};

    /*!
     * @brief       The transformation matrix from ground plane to horizontal.
     */
    Eigen::Matrix4f mPlanePoseMat;

    /*!
     * @brief       The main Run() function runs every runInterval_s seconds.
     */
    const double runInterval_s = 3.0;

    /*!
     * @brief       Member which contains the system parameters.
     */
    SystemParams *sysParams;

    /*!
     * @brief       Temporally tracked evidence for one open-passage hypothesis.
     */
    struct OpenPassageEvidence
    {
        Plane          *p_supportingWall  = nullptr;
        Eigen::Vector3d centroid_World_m  = Eigen::Vector3d::Zero();
        std::size_t     confirmationCount = 0U;
        std::size_t     missedUpdateCount = 0U;
        std::uint64_t   lastConfirmedSkeletonFingerprint = 0U;
    };

    /*!
     * @brief       Passage hypotheses awaiting repeated Voxblox confirmation.
     */
    std::vector<OpenPassageEvidence> openPassageEvidence_;

    /*!
     * @brief       Fingerprint of the last processed sparse-graph snapshot.
     */
    std::uint64_t lastSkeletonFingerprint_ = 0U;

    /*!
     * @brief       Whether lastSkeletonFingerprint_ contains a valid snapshot.
     */
    bool hasSkeletonFingerprint_ = false;

    /*!
     * @brief       Latest UAV camera centre in the active map frame.
     *
     *              Maintained while consuming ordered keyframes so traversal
     *              tests cover every consecutive camera segment.
     */
    Eigen::Vector3d currentCameraCenter_World_m = Eigen::Vector3d::Zero();

    /*!
     * @brief       Camera centre of the previous processed keyframe.
     */
    Eigen::Vector3d previousCameraCenter_World_m = Eigen::Vector3d::Zero();

    /*!
     * @brief       Whether currentCameraCenter_World_m holds a valid position.
     */
    bool hasCameraCenter_ = false;

    /* Active map whose frame contains the tracked camera centres. */
    Map *pCameraCenterMap_ = nullptr;

    /*! @brief Last keyframe consumed by traversal sampling. */
    long unsigned int lastTraversalFrameId_    = 0U;
    long unsigned int lastTraversalKeyFrameId_ = 0U;

    /*! @brief Whether the traversal keyframe cursor is initialized. */
    bool hasTraversalKeyFrameCursor_ = false;

    /*! @brief Active map which owns every map-local temporal cache below. */
    Map *pTemporalStateMap_ = nullptr;

    /*! @brief Coordinate-frame epoch used to detect whole-map rebases only. */
    std::uint64_t temporalStateWorldFrameEpoch_ = 0U;

    /*!
     * @brief Confirmed rooms reported as disconnected on the previous cycle.
     *
     *        Retaining the IDs prevents the online consistency warning from
     *        being repeated when no new passage evidence has arrived.
     */
    std::unordered_set<int> disconnectedRoomIds_;

    /*!
     * @brief       Tracks passage-created prospective room handles.
     *              A zero value denotes a live handle; entries are removed on
     *              promotion, replacement, or orphan cleanup.
     */
    std::unordered_map<int, int> prospectiveRoomCycles_;

    struct UndefendedWallState
    {
        Plane       *p_wall           = nullptr;
        unsigned int unresolvedCycles = 0U;
        std::size_t  cloudPointCount  = 0U;
        std::size_t  observationCount = 0U;
    };

    /*! @brief Weak, unused wall hypotheses awaiting bounded retirement. */
    std::unordered_map<int, UndefendedWallState> undefendedWalls_;

    /*! @brief Suppresses repeated diagnostics for the same entity ID. */
    std::unordered_set<int> loggedOrphanWallIds_;
    std::unordered_set<int> loggedRetiredWallIds_;
    std::unordered_set<int> loggedRoomCleanupIds_;

    /*!
     * @brief       Maximum number of prospective rooms allowed simultaneously.
     *              Matches office_clean's 12 rooms.
     */
    static constexpr int kMaxProspectiveRooms = 12;

    /*!
     * @brief       Spatial deduplication distance for prospective rooms
     * (meters).
     */
    static constexpr double kProspectiveDedupDistance_m = 2.0;

    /**
     * @brief Scopes transient semantic evidence to one active map and frame.
     *
     * A map switch clears every map-local cache. An in-place map correction
     * invalidates geometry fingerprints and reseeds camera traversal state.
     */
    void resetTemporalStateForMap(Map *p_activeMap_in);

    /*!
     * @brief       Seeds currentRoomId_ from the first confirmed room of the
     *              active map, only while no current room is resolved yet.
     *
     * @param[in]   p_activeMap_in
     *              Active map whose room set provides the seed room.
     */
    void seedCurrentRoomFromActiveMap(Map *p_activeMap_in);

    /*!
     * @brief       Advances the room-state machine once per semantic cycle.
     *
     *              Consumes the per-cycle crossing and tracking-loss signals
     *              and feeds them to roomTracker_ together with the abstract
     *              (Phase 4 stub) verification verdict. Read-only with respect
     *              to currentRoomId_/lastKnownRoomId_.
     */
    void updateRoomTrackerState(void);

    /*!
     * @brief       Enforces exclusive ownership of every mapped wall surface.
     *
     *              Observation-side evidence selects the most plausible room
     *              when legacy data or a map merge assigned one Plane object
     *              to multiple rooms. This structural invariant is enforced
     *              even when passage detection is disabled.
     */
    void enforceUniqueWallOwnership(void);

    /*!
     * @brief Validates finite wall associations and updates room maturity.
     *
     *        The validator repairs decisive wall clashes, rejects ambiguous
     *        self-intersections, and marks a room complete only when ordered
     *        finite wall segments form a closed non-self-intersecting loop.
     *        Passage detection remains independent and may precede boundary
     *        completion.
     */
    void validateRoomBoundaries(void);

    /*! @brief Ages and safely retires unused walls which never become valid. */
    void suppressUndefendedWalls(void);

    /*!
     * @brief Recomputes room centroids as the arithmetic mean of associated
     *        wall centroids.
     *
     *        Replaces the free-space cluster centroid with the geometric center
     *        of the room's boundary walls. Rooms with no valid walls retain
     *        their existing centroid.
     */
    void recomputeRoomCentroidsFromWalls(void);

    /*!
     * @brief Splits connected skeleton components at confirmed passages.
     *
     *        Voxblox correctly represents free-space connectivity through an
     *        opening, whereas a semantic room must stop at that opening. The
     *        raw sparse-graph edges are therefore cut only inside confirmed
     *        passage apertures before room-wall association.
     *
     * @param[in] freeSpaceClusters_World_m_in
     *            Connected Voxblox vertex components in the active map frame.
     * @return Passage-partitioned components in the same frame.
     */
    std::vector<std::vector<Eigen::Vector3d>> partitionFreeSpaceAtPassages(
        const std::vector<std::vector<Eigen::Vector3d>>
            &freeSpaceClusters_World_m_in) const;

    /*!
     * @brief Removes room-wall associations proven to pass through an opening.
     *
     *        This repairs persistent associations created before a passage was
     *        confirmed. Supporting walls on the passage plane are retained.
     */
    void detachWallsBeyondConfirmedPassages(void);

    /*!
     * @brief Adds a wall only when it preserves a valid room boundary.
     *
     *        A decisively stronger candidate replaces clashing, weaker wall
     *        hypotheses. A weaker or ambiguous candidate remains unassigned
     *        so the orphan-wall hierarchy can retain it for another room and
     *        later observations can retry the association.
     *
     * @param[in,out] p_room_inout Room whose wall set may be updated.
     * @param[in] p_candidateWall_in Candidate finite wall hypothesis.
     * @return True when the candidate is already present or was admitted.
     */
    bool admitWallToRoom(Room *p_room_inout, Plane *p_candidateWall_in);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Constructor which stores the pointer to the map in the
     *              member mpAtlas and gets the systems parameter.
     *
     * @param[in]   pAtlas
     *              Pointer to map.
     */
    explicit SemanticsManager(Atlas *pAtlas);

    /*!
     * @brief       Gets the latest skeleton cluster acquired from voxblox.
     */
    std::vector<std::vector<Eigen::Vector3d>> getLatestSkeletonCluster(void);

    /*!
     * @brief       Detects open passages where a connected Voxblox skeleton
     *              edge crosses a finite mapped wall.
     *
     * @param[in]   wallPlanes
     *              Confirmed wall planes available in the current map.
     */
    void detectOpenPassagesFromSkeletonEdges(
        const std::vector<ORB_SLAM3::Plane *> &wallPlanes);

    /*!
     * @brief       Detects doors and doorways based on the detected planes and
     *              the mapped environment.
     *
     * @param       pAtlas
     *              The Atlas containing the mapped environment
     */
    void detectDoorsAndDoorways(ORB_SLAM3::Atlas *pAtlas);

    /*!
     * @brief       Gets the latest detected room candidates from GNN-based room
     *              detection.
     */
    std::vector<ORB_SLAM3::Room *> getLatestGNNRoomCandidates(void);

    /*!
     * @brief       Filters the wall planes to remove heavily tilted walls. Does
     *              this by comparing the transpose to the ground plane. If the
     *              plane normal is horizontal to the ground then it is left
     *              alone. If the wall plane has a tile greater than the
     *              sem_seg.max_tilt_wall parameter set in System Params, its
     *              semantics are reset.
     */
    void filterWallPlanes(void);

    /*!
     * @brief       Filters the planes which are assoicated with ground semantic
     *              and resets the planes semantics if it is not horizontal
     *              enough or if its height is outside tolerances with the main
     *              ground plane. Removes points that are too far from the
     *              plane.
     *
     * @param       groundPlane
     *              The main ground plane that is the reference
     */
    void filterGroundPlanes(Plane *groundPlane);

    /*!
     * @brief       Updates the passages in the map based on the detected doors
     *              and doorways.
     *
     * @param       pAtlas
     *              The Atlas containing the mapped environment
     */
    void updatePassages(ORB_SLAM3::Atlas *pAtlas);

    /*!
     * @brief       Records traversal evidence on every passable passage whose
     *              aperture the UAV camera trajectory crossed since the last
     *              semantic cycle.
     *
     *              Consumes all new keyframes in deterministic frame/id order
     *              and checks each consecutive camera-centre segment against
     *              passage aperture geometry. The evidence is a separate
     *              "settled" mark and never modifies observation confirmation.
     *
     * @param[in]   pAtlas
     *              The Atlas containing the mapped environment.
     */
    void updateTraversalEvidence(ORB_SLAM3::Atlas *pAtlas);

    /*!
     * @brief       Returns the id of the room currently occupied by the camera.
     *
     * @return      Current room id, or -1 before any room is confirmed.
     */
    int getCurrentRoomId() const;

    /*!
     * @brief       Returns the id of the room last occupied before tracking was
     *              lost.
     *
     * @return      Last-known room id, or -1 before any tracking loss.
     */
    int getLastKnownRoomId() const;

    /*!
     * @brief       Persists the current room as the last-known room when
     *              tracking is lost.
     *
     *              Records the transition only once per loss episode: repeated
     *              calls while the current room is unchanged are no-ops.
     */
    void onTrackingLost(void);

    /*!
     * @brief       Transforms the plane equation to the ground reference
     *              defined by mPlanePoseMat.
     *
     * @param       planeEq
     *              The plane equation
     *
     * @return      The transformed plane equation
     */
    Eigen::Vector3f
        transformPlaneEqToGroundReference(const Eigen::Vector4d &planeEq);

    /*!
     * @brief       Gets the median height of a ground plane after
     *              transformation to referece by mPlanePoseMat.
     *
     * @param       groundPlane
     *              The ground plane
     *
     * @return      The median height of the ground plane
     */
    float computeGroundPlaneHeight(Plane *groundPlane);

    /*!
     * @brief       Computes the transformation matrix from the ground plane to
     *              the horizontal (y-inverted).
     *
     * @param       plane
     *              The plane
     *
     * @return      the transformation matrix
     */
    Eigen::Matrix4f computePlaneToHorizontal(const Plane *plane);

    /*!
     * @brief       Checks for the existing of a room with particular walls
     *              close to a cluster. It returns the address of the existing
     *              room if found, otherwise returns nullptr.
     *
     * @param       clusterCentroid_World_in
     *              The centroid of the cluster in the world frame of the
     *              current map.
     *
     * @param       wallList_World_in
     *              The list of walls to be checked in the current world map
     *
     * @param[in]   freeSpaceCluster_World_m_in
     *              Connected Voxblox skeleton vertices supporting the room.
     *
     * @param[in]   excludedRoomIds_in
     *              Room IDs already matched to other clusters in this cycle.
     */
    ORB_SLAM3::Room *associateRooms(
        const Eigen::Vector3d                  clusterCentroid_World_in,
        const std::vector<ORB_SLAM3::Plane *> &wallList_World_in,
        const std::vector<Eigen::Vector3d>    &freeSpaceCluster_World_m_in,
        const std::unordered_set<int>         &excludedRoomIds_in);

    /*!
     * @brief       Fuses duplicate room hypotheses supported by one connected
     *              free-space region when no finite wall separates them.
     *
     * @param[in,out] p_retainedRoom_inout
     *                Room which retains the consolidated semantic topology.
     * @param[in]   freeSpaceCluster_World_m_in
     *              Connected Voxblox skeleton vertices for the current room.
     * @param[in]   wallList_World_in
     *              Current mapped wall surfaces used as merge vetoes.
     */
    void consolidateRoomsInFreeSpaceCluster(
        ORB_SLAM3::Room                       *p_retainedRoom_inout,
        const std::vector<Eigen::Vector3d>    &freeSpaceCluster_World_m_in,
        const std::vector<ORB_SLAM3::Plane *> &wallList_World_in);

    /*!
     * @brief       Ensures every valid WALL plane belongs to at least one  room
     *              or provisional structural element.
     */
    void associateAllWallsToRooms(void);

    /*!
     * @brief       Associates each passage with the closest room on either
     *              side of its supporting wall.
     */
    void associatePassagesToRooms(void);

    /*!
     * @brief       Re-associates rooms based on fixed time intervals to
     *              avoid duplicates.
     */
    void reAssociateRooms(void);

    /*!
     * @brief       Processes the latest skeleton cluster to detect rooms based
     *              on free space clustering.
     */
    void detectRoom_FreeSpaceCluster(void);

    /*!
     * @brief       Gets the rooms detected by the GNN module.
     */
    void detectRoom_GNN(void);

    /*!
     * @brief       Gets the updated floors containing rooms and corridors.
     */
    void getUpdatedFloors(void);

    /*!
     * @brief       Method which runs the thread of the segmantic manager.
     */
    void Run(void);

    /*!
     * @brief       Requests a graceful stop of the semantic manager thread.
     */
    void RequestFinish();

    /*!
     * @brief       True once the semantic manager thread has exited Run().
     */
    bool isFinished();
};
} // namespace ORB_SLAM3

#endif // SEMANTICSEG_H
