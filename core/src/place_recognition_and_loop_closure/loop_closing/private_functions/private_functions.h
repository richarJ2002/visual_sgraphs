/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  LoopClosing translation units.
 *
 * @note            These helpers were file-scope functions inside the
 *                  anonymous namespace of LoopClosing.cc; external linkage
 *                  here is module-internal only. Names are kept verbatim
 *                  (identifier renaming is a separate step).
 */

#ifndef VS_GRAPHS_CORE_LOOPCLOSING_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_LOOPCLOSING_PRIVATE_FUNCTIONS_H

#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingClass.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/LoopClosingStatusEnum.h"
#include "place_recognition_and_loop_closure/loop_closing/objects/MergeMapContentsStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticMergeGateResultStruct.h"
#include "scene_understanding/semantic/semantic_verify/objects/SemanticVerifyConfigStruct.h"
#include "scene_understanding/semantics_manager/objects/GraphLockModeEnum.h"
#include "scene_understanding/semantics_manager/objects/GraphLockScopeStruct.h"

#include <mutex>
#include <vector>

namespace vs_graphs
{
namespace core
{

namespace geometric
{
class Plane;
} /* namespace geometric */

namespace map
{
class Map;
} /* namespace map */

namespace semantic
{
class Floor;
class Passage;
class Room;
} /* namespace semantic */

namespace types
{
class SystemParams;
} /* namespace types */

/*!
 * @brief           Number of keyframes a merge window tries to reach, and the
 *                  size of the temporal chain walked on each side of an
 *                  inertial keyframe.
 */
inline constexpr int COUNT_TEMPORAL_KEY_FRAMES = 25;

/*!
 * @brief           Maximum number of covisibility rounds spent growing a merge
 *                  window towards COUNT_TEMPORAL_KEY_FRAMES keyframes.
 */
inline constexpr int MAXIMUM_EXPANSION_ITERATIONS = 5;

/*!
 * @brief           Resolves the surviving and the absorbed map of a merge
 *                  candidate and checks that the candidate is still valid: both
 *                  keyframes exist and are not bad, the maps differ, are not
 *                  bad and are both active.
 *
 * @param[in]       p_currentKeyFrame_in
 *                  Keyframe of the surviving map that detected the merge.
 *
 * @param[in]       p_mergeMatchedKeyFrame_in
 *                  Keyframe of the other map that matched it.
 *
 * @param[in]       p_atlas_in
 *                  Atlas that owns both maps.
 *
 * @param[out]      p_currentMap_out
 *                  Map of the current keyframe; null when it could not be
 *                  resolved.
 *
 * @param[out]      p_mergeMap_out
 *                  Map of the matched keyframe; null when it could not be
 *                  resolved.
 *
 * @return          True when the merge may proceed; false when the candidate is
 *                  stale and must be rejected.
 */
[[nodiscard]] bool
    selectMergeMaps(keyframe::KeyFrame *p_currentKeyFrame_in,
                    keyframe::KeyFrame *p_mergeMatchedKeyFrame_in,
                    atlas::Atlas       *p_atlas_in,
                    map::Map          *&p_currentMap_out,
                    map::Map          *&p_mergeMap_out);

/*!
 * @brief           Asks local mapping to stop and blocks until it has stopped,
 *                  so no other thread changes keyframes or map points during a
 *                  merge.
 *
 * @param[in,out]   p_localMapper_inout
 *                  Local mapper to stop.
 */
void stopLocalMapping(local_mapping::LocalMapping *p_localMapper_inout);

/*!
 * @brief           Hands the semantic graph back to the semantic workers and
 *                  lets local mapping run again.
 *
 * @param[in,out]   semanticUpdateLock_inout
 *                  Held semantic update lock; released here.
 *
 * @param[in,out]   p_localMapper_inout
 *                  Local mapper to release.
 */
void resumeMergeWorkers(std::unique_lock<std::mutex> &semanticUpdateLock_inout,
                        local_mapping::LocalMapping  *p_localMapper_inout);

/*!
 * @brief           Takes the semantic update lock, then re-checks the merge
 *                  candidate. Retained retired maps keep stale pointers alive,
 *                  so non-null pointers alone are not enough after the workers
 *                  have been quiesced. On rejection the lock and local mapping
 *                  are released.
 *
 * @param[in,out]   p_atlas_inout
 *                  Atlas that grants the semantic update lock.
 *
 * @param[in,out]   p_localMapper_inout
 *                  Local mapper released on rejection.
 *
 * @param[in]       p_currentKeyFrame_in
 *                  Keyframe of the surviving map.
 *
 * @param[in]       p_mergeMatchedKeyFrame_in
 *                  Keyframe of the absorbed map.
 *
 * @param[in]       p_currentMap_in
 *                  Map the current keyframe must still belong to.
 *
 * @param[in]       p_mergeMap_in
 *                  Map the matched keyframe must still belong to.
 *
 * @param[out]      semanticUpdateLock_out
 *                  Semantic update lock; still held on return true, released on
 *                  return false.
 *
 * @return          True when the merge may proceed with the lock held; false
 *                  when it was rejected.
 */
[[nodiscard]] bool lockAndRevalidateMergeCandidate(
    atlas::Atlas                 *p_atlas_inout,
    local_mapping::LocalMapping  *p_localMapper_inout,
    keyframe::KeyFrame           *p_currentKeyFrame_in,
    keyframe::KeyFrame           *p_mergeMatchedKeyFrame_in,
    map::Map                     *p_currentMap_in,
    map::Map                     *p_mergeMap_in,
    std::unique_lock<std::mutex> &semanticUpdateLock_out);

/*!
 * @brief           Locks the scene graphs of both merge maps through the
 *                  semantics manager, in ascending map-id order. The caller
 *                  holds the semantic update lock and, for an exclusive lock,
 *                  both maps' update mutexes first (lock order: update
 *                  mutexes, then graph locks).
 *
 * @param[in]       p_atlas_in
 *                  Atlas that gives the semantics manager.
 *
 * @param[in]       p_currentMap_in
 *                  Surviving map.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map.
 *
 * @param[in]       mode_in
 *                  How to lock the graphs.
 *
 * @param[in,out]   scope_out
 *                  Receives the locks; unlocks on destruction.
 */
void lockMergeGraphs(atlas::Atlas   *p_atlas_in,
                     map::Map       *p_currentMap_in,
                     map::Map       *p_mergeMap_in,
                     GraphLockMode   mode_in,
                     GraphLockScope &scope_out);

/*!
 * @brief           Computes the similarity that expresses merge-map world
 *                  coordinates in the current-map world frame, from the current
 *                  keyframe pose and the similarity found by place recognition.
 *
 * @param[in]       p_currentKeyFrame_in
 *                  Keyframe of the surviving map.
 *
 * @param[in]       mergeSimilarity_mergeWorldToCurrentCamera_in
 *                  Similarity from the merge-map world frame to the current
 *                  keyframe camera frame.
 *
 * @return          Similarity from the merge-map world frame to the current-map
 *                  world frame.
 */
[[nodiscard]] g2o::Sim3 calculateMergeMapTransform(
    keyframe::KeyFrame *p_currentKeyFrame_in,
    const g2o::Sim3    &mergeSimilarity_mergeWorldToCurrentCamera_in);

/*!
 * @brief           Formats a merge similarity as the log fields appended to
 *                  every [SemanticMergeGate] line: translation in metres,
 *                  rotation quaternion (x, y, z, w) and scale, each starting
 *                  with a space.
 *
 * @param[in]       mergeTransform_absorbedToSurviving_in
 *                  Similarity from the absorbed map's world frame to the
 *                  surviving map's world frame; maps a point as
 *                  p_surviving = scale * R * p_absorbed + t.
 *
 * @return          The formatted fields.
 */
[[nodiscard]] std::string formatMergeTransform(
    const g2o::Sim3 &mergeTransform_absorbedToSurviving_in);

/*!
 * @brief           Runs the semantic merge gate on a merge candidate and prints
 *                  its decision record (committed=0) and, on a second line, the
 *                  element pairs and problems behind the decision.
 *
 * @param[in]       p_atlas_in
 *                  Atlas whose semantics manager lists the rooms of both maps.
 *
 * @param[in]       p_currentMap_in
 *                  Surviving map.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map.
 *
 * @param[in]       mapTransform_mergeWorldToCurrentWorld_in
 *                  Proposed similarity from the merge-map world frame to the
 *                  current-map world frame.
 *
 * @param[out]      gateResult_out
 *                  Decision, reason and evidence counts of the gate.
 */
void evaluateSemanticMergeGate(
    atlas::Atlas                      *p_atlas_in,
    map::Map                          *p_currentMap_in,
    map::Map                          *p_mergeMap_in,
    const g2o::Sim3                   &mapTransform_mergeWorldToCurrentWorld_in,
    semantic::SemanticMergeGateResult &gateResult_out);

/*!
 * @brief           Grows a keyframe window around a centre keyframe: adds its
 *                  strongest covisible keyframes, then, while the window is
 *                  smaller than COUNT_TEMPORAL_KEY_FRAMES, repeatedly adds the
 *                  covisible keyframes of the keyframes already in it, up to
 *                  MAXIMUM_EXPANSION_ITERATIONS rounds.
 *
 * @param[in]       p_centerKeyFrame_in
 *                  Keyframe the window is built around; always inserted.
 *
 * @param[in,out]   windowKeyFrames_inout
 *                  Window to grow.
 */
void expandWindowByCovisibility(
    keyframe::KeyFrame             *p_centerKeyFrame_in,
    std::set<keyframe::KeyFrame *> &windowKeyFrames_inout);

/*!
 * @brief           Adds every map point observed by the valid keyframes of a
 *                  window to a map point set.
 *
 * @param[in]       windowKeyFrames_in
 *                  Keyframes whose map points are collected.
 *
 * @param[in,out]   windowMapPoints_inout
 *                  Set the map points are inserted into.
 */
void collectWindowMapPoints(
    const std::set<keyframe::KeyFrame *> &windowKeyFrames_in,
    std::set<map_point::MapPoint *>      &windowMapPoints_inout);

/*!
 * @brief           Builds the current-map side of the weld: the temporal chain
 *                  around the current keyframe when both maps are inertial,
 *                  otherwise the current keyframe alone, grown by covisibility,
 *                  with the map points the window observes.
 *
 * @param[in]       p_currentKeyFrame_in
 *                  Keyframe of the surviving map that detected the merge.
 *
 * @param[in]       p_currentMap_in
 *                  Surviving map.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map.
 *
 * @param[in,out]   windowKeyFrames_inout
 *                  Receives the window keyframes.
 *
 * @param[in,out]   windowMapPoints_inout
 *                  Receives the map points the window observes.
 */
void buildCurrentLocalWindow(
    keyframe::KeyFrame              *p_currentKeyFrame_in,
    map::Map                        *p_currentMap_in,
    map::Map                        *p_mergeMap_in,
    std::set<keyframe::KeyFrame *>  &windowKeyFrames_inout,
    std::set<map_point::MapPoint *> &windowMapPoints_inout);

/*!
 * @brief           Builds the absorbed-map side of the weld: the temporal chain
 *                  around the matched keyframe when both maps are inertial,
 *                  otherwise the matched keyframe alone, grown by covisibility,
 *                  with the map points the window observes.
 *
 * @param[in]       p_mergeMatchedKeyFrame_in
 *                  Keyframe of the absorbed map that matched the current
 *                  keyframe.
 *
 * @param[in]       p_currentMap_in
 *                  Surviving map.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map.
 *
 * @param[in,out]   windowKeyFrames_inout
 *                  Receives the window keyframes.
 *
 * @param[in,out]   windowMapPoints_inout
 *                  Receives the map points the window observes.
 */
void buildMergeLocalWindow(
    keyframe::KeyFrame              *p_mergeMatchedKeyFrame_in,
    map::Map                        *p_currentMap_in,
    map::Map                        *p_mergeMap_in,
    std::set<keyframe::KeyFrame *>  &windowKeyFrames_inout,
    std::set<map_point::MapPoint *> &windowMapPoints_inout);

/*!
 * @brief           Expresses the pose of every absorbed-map welding keyframe in
 *                  the current-map world frame. Ownership does not change; the
 *                  corrected pose, scale and velocity are cached on the
 *                  keyframe for the later transfer.
 *
 * @param[in]       mergeConnectedKeyFrames_in
 *                  Welding keyframes of the absorbed map.
 *
 * @param[in]       p_currentMap_in
 *                  Surviving map; tells whether the IMU is initialised.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map; keyframes of other maps are skipped.
 *
 * @param[in]       mapTransform_currentWorldToMergeWorld_in
 *                  Similarity from the current-map world frame to the merge-map
 *                  world frame.
 *
 * @param[in,out]   nonCorrectedKeyFramePoses_inout
 *                  Receives each keyframe pose before correction (world to
 *                  camera, unit scale).
 *
 * @param[in,out]   correctedKeyFramePoses_inout
 *                  Receives each keyframe pose in the current-map world frame
 *                  (world to camera, with scale).
 */
void correctMergeKeyFramePoses(
    const std::set<keyframe::KeyFrame *> &mergeConnectedKeyFrames_in,
    map::Map                             *p_currentMap_in,
    map::Map                             *p_mergeMap_in,
    const g2o::Sim3 &mapTransform_currentWorldToMergeWorld_in,
    loop_closing::LoopClosing::KeyFrameAndPose &nonCorrectedKeyFramePoses_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout);

/*!
 * @brief           Computes the current-map world position and normal of every
 *                  absorbed-map welding point and caches them on the point for
 *                  the later transfer. Points that are null, bad or no longer
 *                  in the absorbed map are removed from the set.
 *
 * @param[in,out]   mapPointMerges_inout
 *                  Welding points of the absorbed map; invalid ones are erased.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map.
 *
 * @param[in]       mapTransform_mergeWorldToCurrentWorld_in
 *                  Similarity from the merge-map world frame to the current-map
 *                  world frame.
 */
void correctMergeMapPoints(
    std::set<map_point::MapPoint *> &mapPointMerges_inout,
    map::Map                        *p_mergeMap_in,
    const g2o::Sim3                 &mapTransform_mergeWorldToCurrentWorld_in);

/*!
 * @brief           Moves the corrected welding keyframes and map points from
 *                  the absorbed map into the surviving map while holding both
 *                  map update mutexes, then makes the surviving map the active
 *                  map.
 *
 * @param[in,out]   p_atlas_inout
 *                  Atlas whose active map becomes the surviving map.
 *
 * @param[in]       p_currentKeyFrame_in
 *                  Keyframe that triggered the merge; its id is recorded on
 *                  every moved keyframe.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map receiving the objects.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map losing the objects.
 *
 * @param[in]       mergeConnectedKeyFrames_in
 *                  Welding keyframes of the absorbed map, with corrected poses
 *                  cached.
 *
 * @param[in]       mapPointMerges_in
 *                  Welding points of the absorbed map, with corrected positions
 *                  cached.
 */
void transferMergeWindow(
    atlas::Atlas                          *p_atlas_inout,
    keyframe::KeyFrame                    *p_currentKeyFrame_in,
    map::Map                              *p_currentMap_inout,
    map::Map                              *p_mergeMap_inout,
    const std::set<keyframe::KeyFrame *>  &mergeConnectedKeyFrames_in,
    const std::set<map_point::MapPoint *> &mapPointMerges_in);

/*!
 * @brief           Attaches the absorbed map spanning tree beneath the current
 *                  keyframe: the matched keyframe becomes a child of the
 *                  current keyframe and every parent edge along its old chain
 *                  to the absorbed-map root is reversed, so the tree stays
 *                  acyclic.
 *
 * @param[in]       p_currentKeyFrame_in
 *                  Keyframe of the surviving map; new parent of the matched
 *                  keyframe.
 *
 * @param[in,out]   p_mergeMatchedKeyFrame_inout
 *                  Matched keyframe of the absorbed map; its parent chain is
 *                  reversed.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map whose origin keyframe stops being a root.
 */
void reverseMergeSpanningTree(keyframe::KeyFrame *p_currentKeyFrame_in,
                              keyframe::KeyFrame *p_mergeMatchedKeyFrame_inout,
                              map::Map           *p_mergeMap_in);

/*!
 * @brief           Recomputes the covisibility connections of every valid
 *                  keyframe in a list from their shared map point observations.
 *
 * @param[in]       keyFrames_in
 *                  Keyframes to update; null and bad keyframes are skipped.
 */
void updateKeyFrameConnections(
    const std::vector<keyframe::KeyFrame *> &keyFrames_in);

/*!
 * @brief           Refines the welded region right after the keyframes moved:
 *                  visual and inertial bundle adjustment when the sensor has an
 *                  IMU, visual-only local bundle adjustment otherwise.
 *
 * @param[in]       p_tracker_in
 *                  Tracker that tells the sensor configuration.
 *
 * @param[in,out]   p_currentKeyFrame_inout
 *                  Current keyframe of the surviving map.
 *
 * @param[in,out]   p_mergeMatchedKeyFrame_inout
 *                  Matched keyframe of the absorbed map.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 *
 * @param[in,out]   correctedKeyFramePoses_inout
 *                  Corrected keyframe poses; updated by the inertial
 *                  optimisation.
 *
 * @param[in]       mergeConnectedKeyFrames_in
 *                  Welding keyframes of the absorbed map, now owned by the
 *                  surviving map.
 *
 * @param[in]       currentWindowKeyFrames_in
 *                  Welding keyframes of the surviving map; held fixed by the
 *                  visual optimisation.
 */
void optimizeMergeWindow(
    tracking::Tracking                         *p_tracker_in,
    keyframe::KeyFrame                         *p_currentKeyFrame_inout,
    keyframe::KeyFrame                         *p_mergeMatchedKeyFrame_inout,
    map::Map                                   *p_currentMap_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout,
    const std::vector<keyframe::KeyFrame *>    &mergeConnectedKeyFrames_in,
    const std::vector<keyframe::KeyFrame *>    &currentWindowKeyFrames_in);

/*!
 * @brief           Copies the plane, keyframe and map point lists of the
 *                  absorbed map.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map.
 *
 * @param[out]      mergeMapContents_out
 *                  Receives the object lists.
 */
void collectMergeMapContents(
    map::Map                       *p_mergeMap_in,
    loop_closing::MergeMapContents &mergeMapContents_out);

/*!
 * @brief           Tells whether any keyframe in a list is still valid and
 *                  owned by the absorbed map.
 *
 * @param[in]       keyFrames_in
 *                  Keyframes to test.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map.
 *
 * @return          True when at least one keyframe is non-null, not bad and
 *                  belongs to the absorbed map.
 */
[[nodiscard]] bool
    hasMergeMapKeyFrame(const std::vector<keyframe::KeyFrame *> &keyFrames_in,
                        map::Map                                *p_mergeMap_in);

/*!
 * @brief           Moves the position and normal of every valid absorbed-map
 *                  point into the current-map world frame and refreshes its
 *                  normal and depth range.
 *
 * @param[in]       mapPoints_in
 *                  Map points to correct; null, bad and foreign points are
 *                  skipped.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map the points must still belong to.
 *
 * @param[in]       mapTransform_mergeWorldToCurrentWorld_in
 *                  Similarity from the merge-map world frame to the current-map
 *                  world frame.
 */
void correctMapPointsToCurrentWorld(
    const std::vector<map_point::MapPoint *> &mapPoints_in,
    map::Map                                 *p_mergeMap_in,
    const g2o::Sim3 &mapTransform_mergeWorldToCurrentWorld_in);

/*!
 * @brief           Applies the map similarity, including its scale, to every
 *                  remaining keyframe and map point of the absorbed map while
 *                  holding its update mutex. Used for monocular sensors, whose
 *                  maps differ in scale.
 *
 * @param[in]       p_currentMap_in
 *                  Surviving map; tells whether the IMU is initialised.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map whose update mutex is held.
 *
 * @param[in]       mergeMapContents_in
 *                  Remaining keyframes and map points of the absorbed map.
 *
 * @param[in]       mapTransform_mergeWorldToCurrentWorld_in
 *                  Similarity from the merge-map world frame to the current-map
 *                  world frame.
 *
 * @param[in]       mapTransform_currentWorldToMergeWorld_in
 *                  Similarity from the current-map world frame to the merge-map
 *                  world frame.
 *
 * @param[in,out]   nonCorrectedKeyFramePoses_inout
 *                  Receives each keyframe pose before correction.
 *
 * @param[in,out]   correctedKeyFramePoses_inout
 *                  Receives each keyframe pose in the current-map world frame.
 */
void correctMergeMapMonocular(
    map::Map                             *p_currentMap_in,
    map::Map                             *p_mergeMap_inout,
    const loop_closing::MergeMapContents &mergeMapContents_in,
    const g2o::Sim3 &mapTransform_mergeWorldToCurrentWorld_in,
    const g2o::Sim3 &mapTransform_currentWorldToMergeWorld_in,
    loop_closing::LoopClosing::KeyFrameAndPose &nonCorrectedKeyFramePoses_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout);

/*!
 * @brief           Carries the pose change of the corrected absorbed-map
 *                  keyframes over to the semantic geometry (planes, rooms,
 *                  passages, floors) of the absorbed map. A monocular merge can
 *                  change scale, so the scale of each corrected keyframe is
 *                  kept when its pose is rebuilt.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map whose semantic geometry is corrected.
 *
 * @param[in]       p_semanticsManager_in
 *                  Semantics manager that serves the absorbed map's S-Graph;
 *                  borrowed.
 *
 * @param[in]       nonCorrectedKeyFramePoses_in
 *                  Keyframe poses before correction.
 *
 * @param[in]       correctedKeyFramePoses_in
 *                  Corrected keyframe poses with scale; supplies the scale
 *                  after the final pose.
 *
 * @param[in]       mapTransform_mergeWorldToCurrentWorld_in
 *                  Fallback similarity from the merge-map world frame to the
 *                  current-map world frame.
 */
void propagateMergedPoseCorrections(
    map::Map         *p_mergeMap_inout,
    SemanticsManager *p_semanticsManager_in,
    const loop_closing::LoopClosing::KeyFrameAndPose
        &nonCorrectedKeyFramePoses_in,
    const loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_in,
    const g2o::Sim3 &mapTransform_mergeWorldToCurrentWorld_in);

/*!
 * @brief           Moves keyframes and map points that are still owned by the
 *                  absorbed map into the surviving map. Poses are left as they
 *                  are.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map receiving the objects.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map losing the objects.
 *
 * @param[in]       keyFrames_in
 *                  Keyframes to move; null, bad and foreign ones are skipped.
 *
 * @param[in]       mapPoints_in
 *                  Map points to move; null, bad and foreign ones are skipped.
 */
void transferKeyFramesAndMapPoints(
    map::Map                                 *p_currentMap_inout,
    map::Map                                 *p_mergeMap_inout,
    const std::vector<keyframe::KeyFrame *>  &keyFrames_in,
    const std::vector<map_point::MapPoint *> &mapPoints_in);

/*!
 * @brief           Moves the planes of the absorbed map into the surviving map.
 *                  Each plane takes the next free plane id of the surviving
 *                  map.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map.
 *
 * @param[in]       planes_in
 *                  Planes of the absorbed map; null and bad planes are skipped.
 */
void transferPlanes(map::Map                              *p_currentMap_inout,
                    map::Map                              *p_mergeMap_inout,
                    const std::vector<geometric::Plane *> &planes_in);

/*!
 * @brief           Clears the derived free-space skeleton of the surviving map,
 *                  which the external Voxblox node rebuilds in the corrected
 *                  frame.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 */
void clearDerivedSkeleton(map::Map *p_currentMap_inout);

/*!
 * @brief           Commits a finished merge: links the two keyframes with merge
 *                  edges, bumps the change index, announces the map revision to
 *                  derived mapping consumers, makes the surviving map active
 *                  and removes the absorbed map from the atlas.
 *
 * @param[in,out]   p_atlas_inout
 *                  Atlas that drops the absorbed map.
 *
 * @param[in,out]   p_currentKeyFrame_inout
 *                  Keyframe of the surviving map.
 *
 * @param[in,out]   p_mergeMatchedKeyFrame_inout
 *                  Matched keyframe, now owned by the surviving map.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 *
 * @param[in,out]   p_mergeMap_inout
 *                  Absorbed map, marked bad.
 */
void commitMapMerge(atlas::Atlas       *p_atlas_inout,
                    keyframe::KeyFrame *p_currentKeyFrame_inout,
                    keyframe::KeyFrame *p_mergeMatchedKeyFrame_inout,
                    map::Map           *p_currentMap_inout,
                    map::Map           *p_mergeMap_inout);

/*!
 * @brief           Prints the decision record of a committed merge
 *                  (committed=1) and stores a MapMergeRecord on the surviving
 *                  map.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map; receives the record.
 *
 * @param[in]       p_mergeMap_in
 *                  Absorbed map.
 *
 * @param[in]       floorDecision_in
 *                  Floor check outcome text reported by the gate.
 *
 * @param[in]       mergeTransform_absorbedToSurviving_in
 *                  Similarity from the absorbed map's world frame to the
 *                  surviving map's world frame.
 *
 * @param[in]       mergeTime_s_in
 *                  Merge time on the ROS node clock, seconds.
 */
void recordAcceptedMerge(map::Map          *p_currentMap_inout,
                         map::Map          *p_mergeMap_in,
                         const std::string &floorDecision_in,
                         const g2o::Sim3 &mergeTransform_absorbedToSurviving_in,
                         double           mergeTime_s_in);

/*!
 * @brief           Decides whether the global bundle adjustment that was
 *                  stopped for the merge is started again: only if one was
 *                  running, and then only when the surviving map has no
 *                  initialised IMU or is a small single map.
 *
 * @param[in]       wasGlobalBundleAdjustmentStopped_in
 *                  True when a global bundle adjustment was running before the
 *                  merge.
 *
 * @param[in]       p_currentMap_in
 *                  Surviving map.
 *
 * @param[in,out]   p_atlas_inout
 *                  Atlas that counts its maps.
 *
 * @return          True when the global bundle adjustment must be relaunched.
 */
[[nodiscard]] bool shouldRelaunchGlobalBundleAdjustment(
    bool          wasGlobalBundleAdjustmentStopped_in,
    map::Map     *p_currentMap_in,
    atlas::Atlas *p_atlas_inout);

/*!
 * @brief           Aligns the surviving inertial map with the absorbed map:
 *                  empties the local mapping queue, applies the scaled rotation
 *                  of the merge similarity to the surviving map and updates the
 *                  tracker IMU state, all under the surviving map update mutex.
 *
 * @param[in,out]   p_localMapper_inout
 *                  Local mapper whose queue is emptied.
 *
 * @param[in,out]   p_tracker_inout
 *                  Tracker whose IMU frame state is updated.
 *
 * @param[in]       p_currentKeyFrame_in
 *                  Keyframe of the surviving map; supplies the IMU bias.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 *
 * @param[in]       p_semanticsManager_in
 *                  Semantics manager that serves the surviving map's S-Graph;
 *                  borrowed.
 *
 * @param[in]       mapTransform_currentWorldToMergeWorld_in
 *                  Similarity from the current-map world frame to the merge-map
 *                  world frame; rotation, translation and scale are applied.
 */
void applyInertialMapAlignment(
    local_mapping::LocalMapping *p_localMapper_inout,
    tracking::Tracking          *p_tracker_inout,
    keyframe::KeyFrame          *p_currentKeyFrame_in,
    map::Map                    *p_currentMap_inout,
    SemanticsManager            *p_semanticsManager_in,
    const g2o::Sim3             &mapTransform_currentWorldToMergeWorld_in);

/*!
 * @brief           Completes the IMU initialisation of the surviving map when
 *                  the sensor has an IMU and the second inertial bundle
 *                  adjustment has not run yet: estimates the biases with zero
 *                  priors, updates the tracker IMU state and marks the map
 *                  initialised.
 *
 * @param[in,out]   p_tracker_inout
 *                  Tracker that tells the sensor and receives the bias.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 */
void initializeInertialMap(tracking::Tracking *p_tracker_inout,
                           map::Map           *p_currentMap_inout);

/*!
 * @brief           Records the current pose of every keyframe of a map as a
 *                  unit-scale similarity (world to camera).
 *
 * @param[in]       p_currentMap_in
 *                  Map whose keyframes are recorded.
 *
 * @param[in,out]   keyFramePoses_inout
 *                  Receives one similarity per keyframe.
 */
void collectKeyFramePoses(
    map::Map                                   *p_currentMap_in,
    loop_closing::LoopClosing::KeyFrameAndPose &keyFramePoses_inout);

/*!
 * @brief           Picks the keyframes and map points that the inertial merge
 *                  fuses: at most six keyframes around the matched keyframe and
 *                  around the current keyframe, and the map points of the
 *                  absorbed-side keyframes, capped at about one thousand.
 *
 * @param[in]       p_mergeMatchedKeyFrame_in
 *                  Matched keyframe of the absorbed map.
 *
 * @param[in,out]   p_currentKeyFrame_inout
 *                  Current keyframe; its connections are refreshed first.
 *
 * @param[out]      mergeConnectedKeyFrames_out
 *                  Matched keyframe and its covisible keyframes.
 *
 * @param[out]      currentConnectedKeyFrames_out
 *                  Current keyframe and its covisible keyframes.
 *
 * @param[out]      mapPointsToFuse_out
 *                  Map points observed by the absorbed-side keyframes.
 */
void gatherInertialFusionCandidates(
    keyframe::KeyFrame                 *p_mergeMatchedKeyFrame_in,
    keyframe::KeyFrame                 *p_currentKeyFrame_inout,
    std::vector<keyframe::KeyFrame *>  &mergeConnectedKeyFrames_out,
    std::vector<keyframe::KeyFrame *>  &currentConnectedKeyFrames_out,
    std::vector<map_point::MapPoint *> &mapPointsToFuse_out);

/*!
 * @brief           Runs the inertial welding bundle adjustment when the
 *                  surviving map had at least ten keyframes before the merge
 *                  and the tracker has a last keyframe.
 *
 * @param[in,out]   p_tracker_inout
 *                  Tracker supplying its last keyframe.
 *
 * @param[in,out]   p_mergeMatchedKeyFrame_inout
 *                  Matched keyframe of the absorbed map.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map.
 *
 * @param[in,out]   correctedKeyFramePoses_inout
 *                  Corrected keyframe poses written by the optimisation.
 *
 * @param[in]       keyFrameCount_in
 *                  Keyframe count of the surviving map before the merge.
 *
 * @return          True when the bundle adjustment ran.
 */
[[nodiscard]] bool runInertialMergeBundleAdjustment(
    tracking::Tracking                         *p_tracker_inout,
    keyframe::KeyFrame                         *p_mergeMatchedKeyFrame_inout,
    map::Map                                   *p_currentMap_inout,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout,
    int                                         keyFrameCount_in);

/*!
 * @brief           Carries the pose change made by the inertial bundle
 *                  adjustment over to the semantic geometry of the surviving
 *                  map. Poses after the optimisation are read back from the
 *                  keyframes, at unit scale, and the semantic fallback
 *                  transform is the identity.
 *
 * @param[in,out]   p_currentMap_inout
 *                  Surviving map whose semantic geometry is corrected.
 *
 * @param[in]       p_semanticsManager_in
 *                  Semantics manager that serves the surviving map's S-Graph;
 *                  borrowed.
 *
 * @param[in]       nonCorrectedKeyFramePoses_in
 *                  Keyframe poses before the optimisation.
 *
 * @param[in,out]   correctedKeyFramePoses_inout
 *                  Receives the keyframe poses after the optimisation.
 */
void propagateInertialPoseCorrections(
    map::Map         *p_currentMap_inout,
    SemanticsManager *p_semanticsManager_in,
    const loop_closing::LoopClosing::KeyFrameAndPose
                                               &nonCorrectedKeyFramePoses_in,
    loop_closing::LoopClosing::KeyFrameAndPose &correctedKeyFramePoses_inout);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_LOOPCLOSING_PRIVATE_FUNCTIONS_H */
