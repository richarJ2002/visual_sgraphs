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
 * @file            KeyFrame.h
 *
 * @brief           Declares KeyFrame, a frame kept in the map: its pose,
 *                  features, observed map points and links to other key frames.
 */

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "ImuTypes.h"
#include "KeyFrameStatus.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "CameraModels/GeometricCamera/objects/GeometricCamera.h"

#include <boost/serialization/access.hpp>
#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vs_graphs
{
namespace core
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;
namespace camera_models
{
class GeometricCamera;
}

namespace semantic
{
class Marker;
}
namespace geometric
{
class Plane;
}
namespace semantic
{
class Passage;
}

/*!
 * @brief           A frame the SLAM system keeps in the map: its camera pose,
 *                  features, observed map points, markers, planes and passages,
 *                  and its links to other key frames (covisibility graph,
 *                  spanning tree, loop and merge edges). Tracking creates it
 *                  from a Frame; Tracking, LocalMapping and LoopClosing share
 *                  it across threads, so most state is guarded by one of the
 *                  mutexes below.
 */
class KeyFrame
{
    friend class boost::serialization::access;

    /*!
     * @brief           Loads or saves the data of this key frame through a
     *                  Boost archive. Links to other key frames, map points and
     *                  cameras are stored as the ids filled in by preSave, not
     *                  as pointers.
     *
     * @param[in,out]   ar
     *                  Boost archive being read from or written to.
     *
     * @param[in]       version
     *                  Archive version, passed on to the matrix and pose
     *                  helpers.
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();
    /*!
     * @brief           Creates a key frame from a tracked frame: copies its
     *                  camera parameters, key points, features, map point
     *                  matches, pose and velocity, takes the next id from
     *                  nextId and stores the map and database it belongs to.
     *
     * @param[in,out]   F_inout
     *                  Frame this key frame is created from.
     *
     * @param[in]       p_map_in
     *                  Map that will contain this key frame; borrowed, must not
     *                  be null.
     *
     * @param[in]       p_keyFrameDatabase_in
     *                  Database that indexes this key frame for place
     *                  recognition; borrowed.
     */
    KeyFrame(Frame            &F_inout,
             Map              *p_map_in,
             KeyFrameDatabase *p_keyFrameDatabase_in);

    // Pose functions
    /*!
     * @brief           Stores the camera pose and refreshes everything derived
     *                  from it: the inverse pose, both rotation matrices and,
     *                  when the IMU calibration is set, the IMU position.
     *
     * @param[in]       cameraPose_worldToCamera_in
     *                  Camera pose; maps world-frame points into the camera
     *                  frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        setPose(const Sophus::SE3f &cameraPose_worldToCamera_in);
    /*!
     * @brief           Stores the IMU velocity of this key frame and marks it
     *                  as available.
     *
     * @param[in]       Vw_in
     *                  IMU velocity in the world frame, metres per second.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus setVelocity(const Eigen::Vector3f &Vw_in);

    /*!
     * @brief           Returns the camera pose of this key frame.
     *
     * @param[out]      pose_out
     *                  Camera pose; maps world-frame points into the camera
     *                  frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getPose(Sophus::SE3f &pose_out);

    /*!
     * @brief           Returns the inverse of the camera pose.
     *
     * @param[out]      poseInverse_out
     *                  Camera pose inverted; maps camera-frame points into the
     *                  world frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getPoseInverse(Sophus::SE3f &poseInverse_out);
    /*!
     * @brief           Returns the camera centre of this key frame.
     *
     * @param[out]      cameraCenter_out
     *                  Camera centre in the world frame, metres.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getCameraCenter(Eigen::Vector3f &cameraCenter_out);

    /*!
     * @brief           Returns the IMU position. It is only refreshed by
     *                  setPose when the IMU calibration is set.
     *
     * @param[out]      imuPosition_out
     *                  IMU position in the world frame, metres.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getImuPosition(Eigen::Vector3f &imuPosition_out);
    /*!
     * @brief           Returns the IMU orientation: the camera-to-world pose
     *                  composed with the IMU calibration.
     *
     * @param[out]      imuRotation_out
     *                  IMU orientation in the world frame as a rotation matrix.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getImuRotation(Eigen::Matrix3f &imuRotation_out);
    /*!
     * @brief           Returns the IMU pose: the camera-to-world pose composed
     *                  with the IMU calibration.
     *
     * @param[out]      imuPose_out
     *                  IMU pose in the world frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getImuPose(Sophus::SE3f &imuPose_out);
    /*!
     * @brief           Returns the rotation part of the camera pose.
     *
     * @param[out]      rotation_out
     *                  Rotation of the world-to-camera pose.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getRotation(Eigen::Matrix3f &rotation_out);
    /*!
     * @brief           Returns the translation part of the camera pose.
     *
     * @param[out]      translation_out
     *                  Translation of the world-to-camera pose, metres: the
     *                  world origin seen from the camera.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getTranslation(Eigen::Vector3f &translation_out);
    /*!
     * @brief           Returns the stored IMU velocity; only meaningful once
     *                  isVelocitySet reports true.
     *
     * @param[out]      velocity_out
     *                  IMU velocity in the world frame, metres per second.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getVelocity(Eigen::Vector3f &velocity_out);
    /*!
     * @brief           Tells whether an IMU velocity has been stored for this
     *                  key frame.
     *
     * @param[out]      isVelocitySet_out
     *                  True when the velocity is available.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus isVelocitySet(bool &isVelocitySet_out);

    // Bag of Words Representation
    /*!
     * @brief           Computes the bag-of-words and feature vectors from the
     *                  descriptors when either is still empty. Needs the ORB
     *                  vocabulary to be set and not null.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus computeBagOfWords();

    // Covisibility graph functions
    /*!
     * @brief           Records that this key frame shares weight observations
     *                  with another key frame, and re-sorts the covisibility
     *                  order when the weight is new or changed.
     *
     * @param[in,out]   p_keyFrame_inout
     *                  Covisible key frame; borrowed.
     *
     * @param[in]       weight_in
     *                  Covisibility weight: number of shared observations.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus addConnection(KeyFrame  *p_keyFrame_inout,
                                               const int &weight_in);
    /*!
     * @brief           Removes the covisibility link to a key frame and
     *                  re-sorts the covisibility order if the link existed.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame whose link is removed; borrowed.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus eraseConnection(KeyFrame *p_keyFrame_in);

    /*!
     * @brief           Recomputes the covisibility links from the map points
     *                  (and, when plane-based covisibility is enabled, the
     *                  planes) this key frame observes. Key frames with at
     *                  least 15 shared observations are linked, or the best one
     *                  if none reaches that; the first call also picks the
     *                  spanning-tree parent.
     *
     * @param[in]       upParent_in
     *                  Only switches debug printing of the shared-observation
     *                  counts to standard output on (false); it does not
     *                  influence the parent choice.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus updateConnections(bool upParent_in = true);
    /*!
     * @brief           Rebuilds the covisibility lists ordered by decreasing
     *                  weight, leaving out key frames flagged bad.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus updateBestCovisibles();
    /*!
     * @brief           Returns every key frame linked to this one by
     *                  covisibility.
     *
     * @param[out]      connectedKeyFrames_out
     *                  Linked key frames; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getConnectedKeyFrames(std::set<KeyFrame *> &connectedKeyFrames_out);
    /*!
     * @brief           Returns the covisible key frames ordered by decreasing
     *                  weight.
     *
     * @param[out]      vectorCovisibleKeyFrames_out
     *                  Covisible key frames, strongest link first; borrowed
     *                  pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getVectorCovisibleKeyFrames(
        std::vector<KeyFrame *> &vectorCovisibleKeyFrames_out);
    /*!
     * @brief           Returns the covisible key frames with the strongest
     *                  links.
     *
     * @param[in]       N_in
     *                  Maximum number of key frames to return.
     *
     * @param[out]      bestCovisibilityKeyFrames_out
     *                  Up to N_in key frames, strongest link first; fewer if
     *                  this key frame has fewer links.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getBestCovisibilityKeyFrames(
        const int               &N_in,
        std::vector<KeyFrame *> &bestCovisibilityKeyFrames_out);
    /*!
     * @brief           Returns the covisible key frames whose link weight is at
     *                  least a threshold.
     *
     * @param[in]       w_in
     *                  Minimum covisibility weight.
     *
     * @param[out]      covisiblesByWeight_out
     *                  Matching key frames, strongest link first; empty if
     *                  none.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
                                 getCovisiblesByWeight(const int               &w_in,
                                                       std::vector<KeyFrame *> &covisiblesByWeight_out);
    /*!
     * @brief           Returns the covisibility weight of the link to a key
     *                  frame.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to look up.
     *
     * @param[out]      weight_out
     *                  Link weight, or 0 when the key frames are not linked.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getWeight(KeyFrame *p_keyFrame_in,
                                           int      &weight_out);

    // Spanning tree functions
    /*!
     * @brief           Adds a key frame as a child of this one in the spanning
     *                  tree.
     *
     * @param[in]       p_keyFrame_in
     *                  Child key frame; borrowed.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus addChild(KeyFrame *p_keyFrame_in);
    /*!
     * @brief           Removes a key frame from the children of this one in the
     *                  spanning tree.
     *
     * @param[in]       p_keyFrame_in
     *                  Child key frame to remove.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus eraseChild(KeyFrame *p_keyFrame_in);
    /*!
     * @brief           Makes a key frame the spanning-tree parent of this one
     *                  and registers this key frame as its child. This key
     *                  frame is not removed from the children of the previous
     *                  parent.
     *
     * @param[in,out]   p_keyFrame_inout
     *                  New parent; must not be this key frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always; throws
     *                  std::invalid_argument when the new parent is this key
     *                  frame.
     */
    [[nodiscard]] KeyFrameStatus changeParent(KeyFrame *p_keyFrame_inout);
    /*!
     * @brief           Returns the spanning-tree children of this key frame.
     *
     * @param[out]      childs_out
     *                  Child key frames; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getChilds(std::set<KeyFrame *> &childs_out);
    /*!
     * @brief           Returns the spanning-tree parent of this key frame.
     *
     * @param[out]      p_parent_out
     *                  Parent key frame; null for the root.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getParent(KeyFrame *&p_parent_out);
    /*!
     * @brief           Tells whether a key frame is a spanning-tree child of
     *                  this one.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to look for.
     *
     * @param[out]      hasChild_out
     *                  True when it is a child.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus hasChild(KeyFrame *p_keyFrame_in,
                                          bool     &hasChild_out);
    /*!
     * @brief           Sets whether the next updateConnections call may choose
     *                  the spanning-tree parent.
     *
     * @param[in]       isFirst_in
     *                  True to allow choosing the parent.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus setFirstConnection(bool isFirst_in);

    // Loop Edges
    /*!
     * @brief           Records a loop-closure edge to a key frame and protects
     *                  this key frame from being erased.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame at the other end of the loop edge.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus addLoopEdge(KeyFrame *p_keyFrame_in);
    /*!
     * @brief           Returns the key frames joined to this one by
     *                  loop-closure edges.
     *
     * @param[out]      loopEdges_out
     *                  Loop-edge key frames; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getLoopEdges(std::set<KeyFrame *> &loopEdges_out);

    // Merge Edges
    /*!
     * @brief           Records a map-merge edge to a key frame and protects
     *                  this key frame from being erased.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame at the other end of the merge edge.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus addMergeEdge(KeyFrame *p_keyFrame_in);
    /*!
     * @brief           Returns the key frames joined to this one by map-merge
     *                  edges.
     *
     * @param[out]      mergeEdges_out
     *                  Merge-edge key frames; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getMergeEdges(std::set<KeyFrame *> &mergeEdges_out);

    // MapPoint observation functions
    /*!
     * @brief           Counts the key points that currently have a map point,
     *                  including map points flagged bad.
     *
     * @param[out]      mapPointCount_out
     *                  Number of non-null map point matches.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getMapPointCount(int &mapPointCount_out);
    /*!
     * @brief           Stores the map point matched to a key point, replacing
     *                  any previous match.
     *
     * @param[in]       p_mapPoint_in
     *                  Map point; borrowed.
     *
     * @param[in]       index_in
     *                  Key point index; must be smaller than the number of key
     *                  points (not checked).
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus addMapPoint(MapPoint     *p_mapPoint_in,
                                             const size_t &index_in);
    /*!
     * @brief           Clears the map point matched to a key point.
     *
     * @param[in]       index_in
     *                  Key point index; must be smaller than the number of key
     *                  points (not checked).
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus eraseMapPointMatch(const int &index_in);
    /*!
     * @brief           Clears the matches of a map point in this key frame, in
     *                  the left and, if present, the right key point index.
     *
     * @param[in]       p_mapPoint_in
     *                  Map point to unmatch; must not be null and must observe
     *                  this key frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus eraseMapPointMatch(MapPoint *p_mapPoint_in);
    /*!
     * @brief           Replaces the map point matched to a key point.
     *
     * @param[in]       index_in
     *                  Key point index; must be smaller than the number of key
     *                  points (not checked).
     *
     * @param[in]       p_mapPoint_in
     *                  New map point; borrowed, may be null.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus replaceMapPointMatch(const int &index_in,
                                                      MapPoint  *p_mapPoint_in);
    /*!
     * @brief           Returns the distinct map points this key frame observes,
     *                  leaving out missing and bad ones.
     *
     * @param[out]      mapPoints_out
     *                  Valid map points; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getMapPoints(std::set<MapPoint *> &mapPoints_out);
    /*!
     * @brief           Returns the map point matched to every key point.
     *
     * @param[out]      mapPointMatches_out
     *                  One entry per key point, in key point order; null where
     *                  there is no match.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getMapPointMatches(std::vector<MapPoint *> &mapPointMatches_out);
    /*!
     * @brief           Counts the key points matched to a good map point,
     *                  optionally only those seen by enough key frames.
     *
     * @param[in]       minimumObservation_in
     *                  Minimum number of observations a map point needs; 0 or
     *                  less counts every good map point.
     *
     * @param[out]      trackedMapPointCount_out
     *                  Number of matching key points.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getTrackedMapPointCount(const int &minimumObservation_in,
                                int       &trackedMapPointCount_out);
    /*!
     * @brief           Returns the map point matched to a key point.
     *
     * @param[in]       index_in
     *                  Key point index; must be smaller than the number of key
     *                  points (not checked).
     *
     * @param[out]      p_mapPoint_out
     *                  Matched map point; null when there is none.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getMapPoint(const size_t &index_in,
                                             MapPoint    *&p_mapPoint_out);

    // MapMarker observation functions
    /*!
     * @brief           Adds a marker observed by this key frame; duplicates are
     *                  not filtered.
     *
     * @param[in]       p_marker_in
     *                  Marker; borrowed.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus addMapMarker(semantic::Marker *p_marker_in);
    /*!
     * @brief           Returns the markers observed by this key frame.
     *
     * @param[out]      mapMarkers_out
     *                  Observed markers; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getMapMarkers(std::vector<semantic::Marker *> &mapMarkers_out);

    // MapPlane observation functions
    /*!
     * @brief           Adds a plane observed by this key frame. A null plane or
     *                  one already present is ignored.
     *
     * @param[in]       p_plane_in
     *                  Plane; borrowed.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus addMapPlane(geometric::Plane *p_plane_in);
    /*!
     * @brief           Returns the planes observed by this key frame. Callers
     *                  must check the entries for null.
     *
     * @param[out]      mapPlanes_out
     *                  Observed planes; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getMapPlanes(std::vector<geometric::Plane *> &mapPlanes_out);
    /*!
     * @brief           Removes a plane from the planes observed by this key
     *                  frame. A null plane is reported on standard error and
     *                  ignored.
     *
     * @param[in]       p_plane_in
     *                  Plane to remove.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus removeMapPlane(geometric::Plane *p_plane_in);

    /*!
     * @brief           Replaces a retired mapped plane in this keyframe.
     *
     *                  Duplicate retained-plane entries are removed while the
     *                  feature mutex is held.
     *
     * @param[in]       p_retiredPlane_in
     *                  Plane hypothesis which is being retired.
     *
     * @param[in]       p_retainedPlane_in
     *                  Plane hypothesis which owns the fused observations.
     *
     * @param[out]      wasReplaced_out
     *                  True when the retired plane was present.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS, or
     *                  KEY_FRAME_STATUS_INVALID_ARGUMENT when an input is
     *                  rejected.
     */
    [[nodiscard]] KeyFrameStatus
        replaceMapPlane(geometric::Plane *p_retiredPlane_in,
                        geometric::Plane *p_retainedPlane_in,
                        bool             &wasReplaced_out);

    /*!
     * @brief           Adds a mapped passage association.
     *
     * @param[in]       p_passage_in
     *                  Passage observed by this keyframe.
     */
    [[nodiscard]] KeyFrameStatus
        addMapPassage(vs_graphs::core::semantic::Passage *p_passage_in);

    /*!
     * @brief           Replaces a retired passage association after fusion.
     *
     * @param[in]       p_retiredPassage_in
     *                  Duplicate passage being retired.
     *
     * @param[in]       p_retainedPassage_in
     *                  Passage retaining the associations.
     *
     * @param[out]      wasReplaced_out
     *                  True when the retired passage was present.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS, or
     *                  KEY_FRAME_STATUS_INVALID_ARGUMENT when an input is
     *                  rejected.
     */
    [[nodiscard]] KeyFrameStatus replaceMapPassage(
        vs_graphs::core::semantic::Passage *p_retiredPassage_in,
        vs_graphs::core::semantic::Passage *p_retainedPassage_in,
        bool                               &wasReplaced_out);

    /*!
     * @brief           Returns the passages (doorways) observed by this key
     *                  frame.
     *
     * @param[out]      mapPassages_out
     *                  Observed passages; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getMapPassages(
        std::vector<vs_graphs::core::semantic::Passage *> &mapPassages_out);

    // KeyPoint functions
    /*!
     * @brief           Finds the key points that lie inside a square around an
     *                  image position, using the key point grid. Undistorted
     *                  key points are compared unless the camera is a stereo
     *                  fisheye.
     *
     * @param[in]       x_in
     *                  Pixel column of the square's centre.
     *
     * @param[in]       y_in
     *                  Pixel row of the square's centre.
     *
     * @param[in]       r_in
     *                  Half side length of the square, pixels.
     *
     * @param[out]      featuresInArea_out
     *                  Indices of the key points found; empty if none.
     *
     * @param[in]       isRightCamera_in
     *                  True to search the right image of a stereo fisheye pair.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
                                 getFeaturesInArea(const float         &x_in,
                                                   const float         &y_in,
                                                   const float         &r_in,
                                                   std::vector<size_t> &featuresInArea_out,
                                                   const bool           isRightCamera_in = false) const;
    /*!
     * @brief           Turns a key point that has a valid depth into a 3D point
     *                  in the world frame.
     *
     * @param[in]       index_in
     *                  Key point index.
     *
     * @param[out]      x3D_out
     *                  3D point in the world frame, metres; set only when
     *                  isUnprojected_out is true.
     *
     * @param[out]      isUnprojected_out
     *                  True when the key point has a positive depth.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus unprojectStereo(int              index_in,
                                                 Eigen::Vector3f &x3D_out,
                                                 bool &isUnprojected_out);

    // Image
    /*!
     * @brief           Tells whether a pixel lies inside the image bounds of
     *                  this key frame.
     *
     * @param[in]       x_in
     *                  Pixel column.
     *
     * @param[in]       y_in
     *                  Pixel row.
     *
     * @param[out]      isInImage_out
     *                  True when the pixel is inside.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus isInImage(const float &x_in,
                                           const float &y_in,
                                           bool        &isInImage_out) const;

    // Enable/Disable bad flag changes
    /*!
     * @brief           Protects this key frame from being erased until setErase
     *                  is called.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus setNotErase();
    /*!
     * @brief           Lifts the erase protection when no loop edge needs it
     *                  and, if an erase was postponed meanwhile, erases the key
     *                  frame now by calling setBadFlag.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus setErase();

    // Set/check bad flag
    /*!
     * @brief           Removes this key frame from the map: unlinks it from the
     *                  covisibility graph and from its observations, gives its
     *                  spanning-tree children a new parent, stores tcp and
     *                  deletes it from the map and the database. Does nothing
     *                  for the first key frame of the map, and postpones the
     *                  removal while the key frame is erase-protected.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus setBadFlag();
    /*!
     * @brief           Tells whether this key frame has been flagged bad.
     *
     * @param[out]      isBad_out
     *                  True once setBadFlag has removed the key frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus isBad(bool &isBad_out);

    /*!
     * @brief           Computes a depth statistic of the map points this key
     *                  frame observes, used in the monocular case: the sorted
     *                  depths are indexed at (count - 1) / q_in, so 2 gives the
     *                  median.
     *
     * @param[in]       q_in
     *                  Divisor selecting the element of the sorted depths; must
     *                  not be 0.
     *
     * @param[out]      sceneMedianDepth_out
     *                  Depth along the camera z axis, metres; -1 when the key
     *                  frame has no key points.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        computeSceneMedianDepth(const int q_in, float &sceneMedianDepth_out);

    /*!
     * @brief           Orders covisibility weights from the highest to the
     *                  lowest.
     *
     * @param[in]       a_in
     *                  First weight.
     *
     * @param[in]       b_in
     *                  Second weight.
     *
     * @return          True when a_in is greater than b_in.
     */
    static bool weightComp(int a_in, int b_in)
    {
        return a_in > b_in;
    }

    /*!
     * @brief           Orders key frames by increasing id.
     *
     * @param[in,out]   p_keyFrame1_inout
     *                  First key frame.
     *
     * @param[in,out]   p_keyFrame2_inout
     *                  Second key frame.
     *
     * @return          True when the first key frame has the smaller id.
     */
    static bool lId(KeyFrame *p_keyFrame1_inout, KeyFrame *p_keyFrame2_inout)
    {
        return p_keyFrame1_inout->id < p_keyFrame2_inout->id;
    }

    /*!
     * @brief           Returns the map that contains this key frame.
     *
     * @param[out]      p_map_out
     *                  Containing map; borrowed.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getMap(Map *&p_map_out);
    /*!
     * @brief           Sets the map that contains this key frame, for example
     *                  after maps are merged.
     *
     * @param[in]       p_map_in
     *                  New containing map; borrowed.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus updateMap(Map *p_map_in);

    /*!
     * @brief           Stores a new IMU bias and passes it on to the
     *                  preintegrated measurements when there are any.
     *
     * @param[in]       b_in
     *                  New accelerometer and gyroscope bias.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus setNewBias(const IMU::Bias &b_in);
    /*!
     * @brief           Returns the gyroscope bias of this key frame.
     *
     * @param[out]      gyroBias_out
     *                  Gyroscope bias (x, y, z), radians per second.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getGyroBias(Eigen::Vector3f &gyroBias_out);

    /*!
     * @brief           Returns the accelerometer bias of this key frame.
     *
     * @param[out]      accBias_out
     *                  Accelerometer bias (x, y, z), metres per second squared.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getAccBias(Eigen::Vector3f &accBias_out);

    /*!
     * @brief           Returns the full IMU bias of this key frame.
     *
     * @param[out]      imuBias_out
     *                  Accelerometer and gyroscope bias.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getImuBias(IMU::Bias &imuBias_out);

    /*!
     * @brief           Projects a map point into the image of this key frame
     *                  and applies the lens distortion model. Reads the pose
     *                  without taking the pose mutex.
     *
     * @param[in]       p_mapPoint_in
     *                  Map point to project; must not be null.
     *
     * @param[out]      keyPoint_out
     *                  Distorted pixel position; set only when isProjected_out
     *                  is true.
     *
     * @param[out]      u_out
     *                  Pixel column of the projection; meaningful only when
     *                  isProjected_out is true.
     *
     * @param[out]      v_out
     *                  Pixel row of the projection; meaningful only when
     *                  isProjected_out is true.
     *
     * @param[out]      isProjected_out
     *                  True when the point is in front of the camera and inside
     *                  the image bounds.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus projectPointDistort(MapPoint    *p_mapPoint_in,
                                                     cv::Point2f &keyPoint_out,
                                                     float       &u_out,
                                                     float       &v_out,
                                                     bool &isProjected_out);
    /*!
     * @brief           Projects a map point into the image of this key frame
     *                  with the pinhole model, without lens distortion. Reads
     *                  the pose without taking the pose mutex.
     *
     * @param[in]       p_mapPoint_in
     *                  Map point to project; must not be null.
     *
     * @param[out]      keyPoint_out
     *                  Pixel position; set only when isProjected_out is true.
     *
     * @param[out]      u_out
     *                  Pixel column of the projection; meaningful only when
     *                  isProjected_out is true.
     *
     * @param[out]      v_out
     *                  Pixel row of the projection; meaningful only when
     *                  isProjected_out is true.
     *
     * @param[out]      isProjected_out
     *                  True when the point is in front of the camera and inside
     *                  the image bounds.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        projectPointUnDistort(MapPoint    *p_mapPoint_in,
                              cv::Point2f &keyPoint_out,
                              float       &u_out,
                              float       &v_out,
                              bool        &isProjected_out);

    /*!
     * @brief           Prepares this key frame for saving: replaces its
     *                  pointers to key frames, map points and cameras by ids in
     *                  the backup members. A pointer whose target is not in the
     *                  given sets is stored as no id.
     *
     * @param[in]       keyFrames_in
     *                  Key frames being saved.
     *
     * @param[in]       mapPoints_in
     *                  Map points being saved.
     *
     * @param[in]       cameras_in
     *                  Cameras being saved.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        preSave(std::set<KeyFrame *> &keyFrames_in,
                std::set<MapPoint *> &mapPoints_in,
                std::set<camera_models::geometriccamera::GeometricCamera *>
                    &cameras_in);
    /*!
     * @brief           Rebuilds the pointers to key frames, map points and
     *                  cameras of a loaded key frame from the ids stored by
     *                  preSave, then clears the backup members.
     *
     * @param[in]       keyFrameId_in
     *                  Loaded key frames by id.
     *
     * @param[in]       mapPointId_in
     *                  Loaded map points by id.
     *
     * @param[in]       cameraId_in
     *                  Loaded cameras by id.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        postLoad(std::map<long unsigned int, KeyFrame *> &keyFrameId_in,
                 std::map<long unsigned int, MapPoint *> &mapPointId_in,
                 std::map<unsigned int,
                          camera_models::geometriccamera::GeometricCamera *>
                     &cameraId_in);

    /*!
     * @brief           Sets the vocabulary used by computeBagOfWords.
     *
     * @param[in]       p_orbVocabulary_in
     *                  Vocabulary; borrowed.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        setORBVocabulary(ORBVocabulary *p_orbVocabulary_in);
    /*!
     * @brief           Sets the database that indexes this key frame.
     *
     * @param[in]       p_keyFrameDatabase_in
     *                  Database; borrowed.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        setKeyFrameDatabase(KeyFrameDatabase *p_keyFrameDatabase_in);

    /*!
     * @brief           True when this key frame carries inertial data: taken
     *                  from the map's IMU-initialised flag at construction and
     *                  set by Tracking for inertial key frames.
     */
    bool isImu;

    // The following variables are accesed from only 1 thread or never change
    // (no mutex needed).
  public:
    /*!
     * @brief           Id given to the next key frame created from a Frame;
     *                  shared by all key frames and incremented in the
     *                  constructor without a lock.
     */
    static long unsigned int nextId;
    /*!
     * @brief           Unique id of this key frame, taken from nextId at
     *                  construction; also how saved maps refer to it.
     */
    long unsigned int        id;
    /*!
     * @brief           Id of the Frame this key frame was created from.
     */
    const long unsigned int  frameId;

    /*!
     * @brief           Capture time of the source frame, seconds.
     */
    const double timeStamp;

    // Grid (to speed up feature matching)
    /*!
     * @brief           Number of columns of the key point grid laid over the
     *                  image (FRAME_GRID_COLS).
     */
    const int   gridCols;
    /*!
     * @brief           Number of rows of the key point grid laid over the image
     *                  (FRAME_GRID_ROWS).
     */
    const int   gridRows;
    /*!
     * @brief           Grid cells per pixel along x: turns an image column into
     *                  a grid column.
     */
    const float gridElementWidthInverse;
    /*!
     * @brief           Grid cells per pixel along y: turns an image row into a
     *                  grid row.
     */
    const float gridElementHeightInverse;

    // Variables used by the tracking
    /*!
     * @brief           Id of the last frame whose local map Tracking collected
     *                  this key frame into; stops it being added twice for one
     *                  frame.
     */
    long unsigned int trackReferenceFrameId;
    /*!
     * @brief           Id of the key frame whose neighbour fusion in
     *                  LocalMapping last visited this key frame; stops it being
     *                  fused twice.
     */
    long unsigned int fuseTargetKeyFrameId;

    // Variables used by the local mapping
    /*!
     * @brief           Id of the key frame whose local bundle adjustment last
     *                  included this key frame as optimised; 0 when none.
     */
    long unsigned int baLocalKeyFrameId;
    /*!
     * @brief           Id of the key frame whose local bundle adjustment last
     *                  included this key frame as fixed (not optimised); 0 when
     *                  none.
     */
    long unsigned int baFixedKeyFrameId;

    /*!
     * @brief           Meant to count bundle adjustment iterations; set to 0 at
     *                  construction and never read or updated afterwards.
     */
    long unsigned int optimizationCount;

    // Variables used by the keyframe database
    /*!
     * @brief           Id of the query key frame for which the loop candidate
     *                  search last counted this key frame.
     */
    long unsigned int loopQuery;
    /*!
     * @brief           Number of vocabulary words this key frame shares with
     *                  the current loop query.
     */
    int               loopWords;
    /*!
     * @brief           Bag-of-words similarity between this key frame and the
     *                  current loop query.
     */
    float             loopScore;
    /*!
     * @brief           Id of the frame for which the relocalisation candidate
     *                  search last counted this key frame.
     */
    long unsigned int relocQuery;
    /*!
     * @brief           Number of vocabulary words this key frame shares with
     *                  the current relocalisation query.
     */
    int               relocWords;
    /*!
     * @brief           Bag-of-words similarity between this key frame and the
     *                  current relocalisation query.
     */
    float             relocScore;
    /*!
     * @brief           Id of the query key frame for which the merge candidate
     *                  search last counted this key frame.
     */
    long unsigned int mergeQuery;
    /*!
     * @brief           Number of vocabulary words this key frame shares with
     *                  the current merge query.
     */
    int               mergeWords;
    /*!
     * @brief           Bag-of-words similarity between this key frame and the
     *                  current merge query.
     */
    float             mergeScore;
    /*!
     * @brief           Id of the query key frame for which the place
     *                  recognition search last counted this key frame.
     */
    long unsigned int placeRecognitionQuery;
    /*!
     * @brief           Number of vocabulary words this key frame shares with
     *                  the current place recognition query.
     */
    int               placeRecognitionWords;
    /*!
     * @brief           Bag-of-words similarity between this key frame and the
     *                  current place recognition query.
     */
    float             placeRecognitionScore;

    /*!
     * @brief           True while LoopClosing is running place recognition with
     *                  this key frame as the current key frame.
     */
    bool isInCurrentPlaceRecognition;

    // Variables used by loop closing
    /*!
     * @brief           Camera pose computed by global bundle adjustment, world
     *                  to camera; LoopClosing applies it with setPose.
     */
    Sophus::SE3f      tcwGBA;
    /*!
     * @brief           Camera pose before the global bundle adjustment
     *                  correction, world to camera; used to move map points
     *                  with their reference key frame.
     */
    Sophus::SE3f      tcwBefGBA;
    /*!
     * @brief           IMU velocity computed by global bundle adjustment, world
     *                  frame; LoopClosing applies it with setVelocity.
     */
    Eigen::Vector3f   vwbGBA;
    /*!
     * @brief           IMU velocity before the global bundle adjustment
     *                  correction, world frame.
     */
    Eigen::Vector3f   vwbBefGBA;
    /*!
     * @brief           IMU bias computed by global bundle adjustment;
     *                  LoopClosing applies it with setNewBias.
     */
    IMU::Bias         biasGBA;
    /*!
     * @brief           Identifies the global bundle adjustment run that last
     *                  corrected this key frame; 0 when none has.
     */
    long unsigned int baGlobalKeyFrameId;

    // Variables used by merging
    /*!
     * @brief           Camera pose after the map merge correction, world to
     *                  camera of the surviving map.
     */
    Sophus::SE3f      tcwMerge;
    /*!
     * @brief           Camera pose before the map merge correction, world to
     *                  camera.
     */
    Sophus::SE3f      tcwBefMerge;
    /*!
     * @brief           Inverse of tcwBefMerge: camera to world, before the map
     *                  merge correction.
     */
    Sophus::SE3f      twcBefMerge;
    /*!
     * @brief           IMU velocity after the map merge correction, world
     *                  frame.
     */
    Eigen::Vector3f   vwbMerge;
    /*!
     * @brief           IMU velocity before the map merge correction, world
     *                  frame; never written or read.
     */
    Eigen::Vector3f   vwbBefMerge;
    /*!
     * @brief           IMU bias after the map merge correction; never written
     *                  or read.
     */
    IMU::Bias         biasMerge;
    /*!
     * @brief           Id of the current key frame whose map merge corrected
     *                  this key frame; 0 when none.
     */
    long unsigned int mergeCorrectedKeyFrameId;
    /*!
     * @brief           Id of a merge key frame; set to nothing and never read
     *                  or written after construction.
     */
    long unsigned int mergeKeyFrameId;
    /*!
     * @brief           Scale of the map merge; left uninitialised and never
     *                  read or written.
     */
    float             scaleMerge;
    /*!
     * @brief           Id of the key frame whose merge local bundle adjustment
     *                  last included this key frame; 0 when none.
     */
    long unsigned int baLocalMergeId;

    /*!
     * @brief           Scale factor of the merge correction applied to this key
     *                  frame.
     */
    float correctedScale;

    // Calibration parameters
    /*!
     * @brief           Focal length along x, pixels.
     */
    const float fx;
    /*!
     * @brief           Focal length along y, pixels.
     */
    const float fy;
    /*!
     * @brief           Principal point column, pixels.
     */
    const float cx;
    /*!
     * @brief           Principal point row, pixels.
     */
    const float cy;
    /*!
     * @brief           Inverse of fx.
     */
    const float invfx;
    /*!
     * @brief           Inverse of fy.
     */
    const float invfy;
    /*!
     * @brief           Stereo baseline multiplied by fx, metres times pixels.
     */
    const float mbf;
    /*!
     * @brief           Stereo baseline, metres.
     */
    const float mb;
    /*!
     * @brief           Depth that separates close from far points, metres:
     *                  close points are created from one view, far points from
     *                  two as in the monocular case.
     */
    const float depthThreshold;
    /*!
     * @brief           Lens distortion coefficients of the camera in OpenCV
     *                  order (k1, k2, p1, p2 and optionally k3).
     */
    cv::Mat     distortionCoefficients;

    /*!
     * @brief           Number of key points of this key frame; the length of
     *                  the per-key-point arrays below and of mapPoints.
     */
    const int keyPointCount;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    /*!
     * @brief           Key points as detected, in image pixels; same index as
     *                  uRight, depths, descriptors and mapPoints.
     */
    const std::vector<cv::KeyPoint> keyPoints;
    /*!
     * @brief           Key points after removing the lens distortion, in image
     *                  pixels; same indexing as keyPoints.
     */
    const std::vector<cv::KeyPoint> keyPointsUndistorted;
    /*!
     * @brief           Pixel column of each key point in the right image for
     *                  stereo and RGB-D points; negative for monocular points.
     */
    const std::vector<float>        uRight;
    /*!
     * @brief           Depth of each key point along the camera z axis, metres;
     *                  negative for monocular points.
     */
    const std::vector<float>        depths;
    /*!
     * @brief           ORB descriptors, one row per key point.
     */
    const cv::Mat                   descriptors;

    // BoW
    /*!
     * @brief           Bag-of-words vector of the descriptors; filled by
     *                  computeBagOfWords unless copied from the frame.
     */
    DBoW2::BowVector     bowVector;
    /*!
     * @brief           Vocabulary nodes of the descriptors with the indices of
     *                  the key points under each node.
     */
    DBoW2::FeatureVector featureVector;

    /*!
     * @brief           Pose of this key frame's camera relative to its parent,
     *                  set by setBadFlag: maps parent-camera points into this
     *                  camera frame.
     */
    Sophus::SE3f tcp;

    // Scale
    /*!
     * @brief           Number of levels of the ORB image pyramid.
     */
    const int                scaleLevelCount;
    /*!
     * @brief           Scale ratio between consecutive ORB pyramid levels.
     */
    const float              scaleFactor;
    /*!
     * @brief           Natural logarithm of scaleFactor.
     */
    const float              logScaleFactor;
    /*!
     * @brief           Scale of each ORB pyramid level relative to level 0.
     */
    const std::vector<float> scaleFactors;
    /*!
     * @brief           Squared scale factor of each pyramid level: the key
     *                  point noise variance at that level, pixels squared.
     */
    const std::vector<float> levelSigmaSquared;
    /*!
     * @brief           Inverse of levelSigmaSquared for each pyramid level.
     */
    const std::vector<float> invLevelSigmaSquared;

    // Image bounds and calibration
    /*!
     * @brief           Smallest pixel column of the undistorted image area.
     */
    const int gridMinX;
    /*!
     * @brief           Smallest pixel row of the undistorted image area.
     */
    const int gridMinY;
    /*!
     * @brief           Largest pixel column of the undistorted image area.
     */
    const int gridMaxX;
    /*!
     * @brief           Largest pixel row of the undistorted image area.
     */
    const int gridMaxY;

    // Preintegrated IMU measurements from previous keyframe
    /*!
     * @brief           Previous key frame in time order for the inertial chain;
     *                  null for the first one; borrowed.
     */
    KeyFrame *p_prevKF;
    /*!
     * @brief           Next key frame in time order for the inertial chain;
     *                  null for the newest one; borrowed.
     */
    KeyFrame *p_nextKF;

    /*!
     * @brief           IMU measurements integrated since the previous key
     *                  frame; borrowed from the frame, null without IMU, and
     *                  pointing at backupImuPreintegrated after postLoad.
     */
    IMU::Preintegrated *p_imuPreintegrated;
    /*!
     * @brief           IMU-to-camera calibration copied from the frame.
     */
    IMU::Calib          imuCalibration;

    /*!
     * @brief           Id of the map this key frame was first created in; kept
     *                  after merges and used to colour key frames when drawing.
     */
    unsigned int originMapId;

    /*!
     * @brief           Name of the image file of the source frame.
     */
    std::string fileName;

    /*!
     * @brief           Index of the dataset sequence the source frame came
     *                  from.
     */
    int datasetId;

    /*!
     * @brief           Loop candidates of this key frame; only ever cleared by
     *                  LoopClosing, never filled.
     */
    std::vector<KeyFrame *> loopCandKFs;
    /*!
     * @brief           Merge candidates of this key frame; only ever cleared by
     *                  LoopClosing, never filled.
     */
    std::vector<KeyFrame *> mergeCandKFs;

    // bool mbHasHessian;
    // cv::Mat mHessianPose;

    // For Semantic Segmentation
    /*!
     * @brief           Colour image of the source frame, sent to semantic
     *                  segmentation and released by clearPointCloud.
     */
    cv::Mat colorImg;
    /*!
     * @brief           True once the colour image has been published for
     *                  semantic segmentation.
     */
    bool    isPublished;

    // The following variables need to be accessed trough a mutex to be thread
    // safe.
  protected:
    /*!
     * @brief           Camera pose, world to camera. Guarded by poseMutex.
     */
    Sophus::SE3<float> poseTcw;
    /*!
     * @brief           Rotation part of poseTcw. Guarded by poseMutex.
     */
    Eigen::Matrix3f    rotationRcw;
    /*!
     * @brief           Inverse of poseTcw: camera to world. Guarded by
     *                  poseMutex.
     */
    Sophus::SE3<float> twc;
    /*!
     * @brief           Rotation part of twc. Guarded by poseMutex.
     */
    Eigen::Matrix3f    rotationRwc;

    /*!
     * @brief           IMU position in the world frame, metres; refreshed by
     *                  setPose when the IMU calibration is set. Guarded by
     *                  poseMutex.
     */
    Eigen::Vector3f owb;
    /*!
     * @brief           IMU velocity in the world frame, metres per second;
     *                  valid when isVelocityAvailable is true. Guarded by
     *                  poseMutex.
     */
    Eigen::Vector3f velocityVw;
    /*!
     * @brief           True once a velocity has been stored. Guarded by
     *                  poseMutex.
     */
    bool            isVelocityAvailable;

    /*!
     * @brief           Transform between the two cameras of a stereo fisheye
     *                  pair, right camera to left camera; inverse of poseTrl.
     *                  Guarded by poseMutex.
     */
    Sophus::SE3<float> poseTlr;
    /*!
     * @brief           Transform between the two cameras of a stereo fisheye
     *                  pair, left camera to right camera; inverse of poseTlr.
     *                  Guarded by poseMutex.
     */
    Sophus::SE3<float> poseTrl;

    /*!
     * @brief           Accelerometer and gyroscope bias of this key frame.
     *                  Guarded by poseMutex.
     */
    IMU::Bias imuBias;

    /*!
     * @brief           Map point matched to each key point, in key point order;
     *                  null where there is none. Guarded by featuresMutex.
     */
    std::vector<MapPoint *> mapPoints;

    /*!
     * @brief           Markers observed in this key frame. Guarded by
     *                  featuresMutex.
     */
    std::vector<semantic::Marker *> mapMarkers;

    /*!
     * @brief           Planes observed in this key frame; entries can be null.
     *                  Guarded by featuresMutex.
     */
    std::vector<geometric::Plane *> mapPlanes;

    /*!
     * @brief           Passages (doorways) observed in this key frame. Guarded
     *                  by featuresMutex.
     */
    std::vector<vs_graphs::core::semantic::Passage *> mapPassages;

    /*!
     * @brief           Ids of the matched map points, -1 for none, in key point
     *                  order; filled by preSave and consumed by postLoad.
     */
    std::vector<long long int> backupMapPointsId;

    /*!
     * @brief           Database that indexes this key frame for place
     *                  recognition; borrowed.
     */
    KeyFrameDatabase *p_keyFrameDatabase;
    /*!
     * @brief           Vocabulary that turns the descriptors into bag-of-words
     *                  vectors; borrowed.
     */
    ORBVocabulary    *p_orbVocabulary;

    /*!
     * @brief           Key point indices per image grid cell, indexed
     *                  [column][row]; speeds up the search for key points near
     *                  a position.
     */
    std::vector<std::vector<std::vector<size_t>>> grid;

    /*!
     * @brief           Covisible key frames with their link weight. Guarded by
     *                  connectionsMutex.
     */
    std::map<KeyFrame *, int>        connectedKeyFrameWeights;
    /*!
     * @brief           Covisible key frames by decreasing link weight. Guarded
     *                  by connectionsMutex.
     */
    std::vector<KeyFrame *>          orderedConnectedKeyFrames;
    /*!
     * @brief           Link weights in the order of orderedConnectedKeyFrames.
     *                  Guarded by connectionsMutex.
     */
    std::vector<int>                 orderedWeights;
    /*!
     * @brief           Covisibility links by key frame id; filled by preSave
     *                  and consumed by postLoad.
     */
    std::map<long unsigned int, int> backupConnectedKeyFrameIdWeights;

    // Spanning Tree and Loop Edges
    /*!
     * @brief           True until updateConnections has chosen the
     *                  spanning-tree parent. Guarded by connectionsMutex.
     */
    bool                           isFirstConnection;
    /*!
     * @brief           Parent in the spanning tree; null for the root. Guarded
     *                  by connectionsMutex.
     */
    KeyFrame                      *p_parent;
    /*!
     * @brief           Children in the spanning tree. Guarded by
     *                  connectionsMutex.
     */
    std::set<KeyFrame *>           childrens;
    /*!
     * @brief           Key frames joined to this one by loop-closure edges.
     *                  Guarded by connectionsMutex.
     */
    std::set<KeyFrame *>           loopEdges;
    /*!
     * @brief           Key frames joined to this one by map-merge edges.
     *                  Guarded by connectionsMutex.
     */
    std::set<KeyFrame *>           mergeEdges;
    /*!
     * @brief           Id of the spanning-tree parent, -1 for none; filled by
     *                  preSave and consumed by postLoad.
     */
    long long int                  backupParentId;
    /*!
     * @brief           Ids of the spanning-tree children; filled by preSave and
     *                  consumed by postLoad.
     */
    std::vector<long unsigned int> backupChildrensId;
    /*!
     * @brief           Ids of the loop-edge key frames; filled by preSave and
     *                  consumed by postLoad.
     */
    std::vector<long unsigned int> backupLoopEdgesId;
    /*!
     * @brief           Ids of the merge-edge key frames; filled by preSave and
     *                  consumed by postLoad.
     */
    std::vector<long unsigned int> backupMergeEdgesId;

    // Bad flags
    /*!
     * @brief           True while this key frame must not be erased (it is
     *                  being processed or has a loop or merge edge); setBadFlag
     *                  then only sets isPendingErase. Guarded by
     *                  connectionsMutex.
     */
    bool isEraseProtected;
    /*!
     * @brief           True when setBadFlag was called during erase protection;
     *                  setErase carries it out.
     */
    bool isPendingErase;
    /*!
     * @brief           True once setBadFlag has removed this key frame from the
     *                  map. Guarded by connectionsMutex.
     */
    bool isFlaggedBad;

    /*!
     * @brief           Half the stereo baseline, metres; only used for drawing.
     */
    float halfBaseline;

    // Variables to be passed to GeometricSegmentation
    /*!
     * @brief           Markers detected in the source frame, copied at
     *                  construction for the semantic analysis.
     */
    std::vector<semantic::Marker *> currentFrameMarkers;
    /*!
     * @brief           Map points of the source frame, copied at construction
     *                  for the semantic analysis.
     */
    std::vector<MapPoint *>         currentFrameMapPoints;

    // point clouds
    /*!
     * @brief           Colour point cloud of the source frame for semantic
     *                  segmentation; set to null by clearPointCloud.
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentFramePointClouds;
    /*!
     * @brief           Per-class point clouds produced by semantic
     *                  segmentation; emptied by clearClsClouds.
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> currentClsCloudPtrs;

    /*!
     * @brief           Map that contains this key frame; borrowed. Guarded by
     *                  mapMutex.
     */
    Map *p_map;

    // Backup variables for inertial
    /*!
     * @brief           Id of p_prevKF, -1 for none; filled by preSave and
     *                  consumed by postLoad.
     */
    long long int      backupPrevKFId;
    /*!
     * @brief           Id of p_nextKF, -1 for none; filled by preSave and
     *                  consumed by postLoad.
     */
    long long int      backupNextKFId;
    /*!
     * @brief           Copy of the preintegrated IMU measurements made by
     *                  preSave; p_imuPreintegrated points at it after postLoad.
     */
    IMU::Preintegrated backupImuPreintegrated;

    // Backup for Cameras
    /*!
     * @brief           Id of p_camera for saving, NO_SAVED_ID when it is
     *                  absent; filled by preSave and consumed by postLoad.
     */
    unsigned int backupCameraId;
    /*!
     * @brief           Id of p_camera2 for saving, NO_SAVED_ID when it is
     *                  absent; filled by preSave and consumed by postLoad.
     */
    unsigned int backupCamera2Id;

    /*!
     * @brief           Pinhole camera matrix built from fx, fy, cx and cy.
     */
    Eigen::Matrix3f calibrationMatrixEigen;

    // Mutex
    /*!
     * @brief           Guards the pose, velocity, stereo fisheye transform and
     *                  IMU bias members.
     */
    std::mutex poseMutex;
    /*!
     * @brief           Guards the covisibility graph, spanning tree, loop and
     *                  merge edges and the erase flags.
     */
    std::mutex connectionsMutex;
    /*!
     * @brief           Guards the map point, marker, plane and passage lists.
     */
    std::mutex featuresMutex;
    /*!
     * @brief           Guards p_map.
     */
    std::mutex mapMutex;

  public:
    /*!
     * @brief           Camera model of the (left) image; borrowed.
     */
    camera_models::geometriccamera::GeometricCamera *p_camera;
    /*!
     * @brief           Camera model of the second camera of a stereo fisheye
     *                  pair; borrowed, null otherwise.
     */
    camera_models::geometriccamera::GeometricCamera *p_camera2;

    /*!
     * @brief           For each left key point, the index of its matching
     *                  right-image key point in a stereo fisheye pair, -1 when
     *                  it has none.
     */
    std::vector<int> leftToRightMatches;
    /*!
     * @brief           For each right key point, the index of its matching
     *                  left-image key point in a stereo fisheye pair.
     */
    std::vector<int> rightToLeftMatches;

    /*!
     * @brief           Returns the transform between the two cameras of a
     *                  stereo fisheye pair, left to right.
     *
     * @param[out]      relativePoseTrl_out
     *                  Transform that maps left-camera points into the right
     *                  camera frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getRelativePoseTrl(Sophus::SE3f &relativePoseTrl_out);
    /*!
     * @brief           Returns the transform between the two cameras of a
     *                  stereo fisheye pair, right to left.
     *
     * @param[out]      relativePoseTlr_out
     *                  Transform that maps right-camera points into the left
     *                  camera frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getRelativePoseTlr(Sophus::SE3f &relativePoseTlr_out);

    /*!
     * @brief           Key points of the right image of a stereo fisheye pair,
     *                  in image pixels.
     */
    const std::vector<cv::KeyPoint> keyPointsRight;

    /*!
     * @brief           Number of key points of the left image of a stereo
     *                  fisheye pair; -1 for other cameras.
     */
    const int leftKeyPointCount;
    /*!
     * @brief           Number of key points of the right image of a stereo
     *                  fisheye pair.
     */
    const int rightKeyPointCount;

    /*!
     * @brief           Key point indices per grid cell for the right image of a
     *                  stereo fisheye pair, indexed [column][row].
     */
    std::vector<std::vector<std::vector<size_t>>> gridRight;

    /*!
     * @brief           Returns the pose of the right camera of a stereo fisheye
     *                  pair.
     *
     * @param[out]      rightPose_out
     *                  Pose that maps world-frame points into the right camera
     *                  frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getRightPose(Sophus::SE3<float> &rightPose_out);
    /*!
     * @brief           Returns the inverse pose of the right camera of a stereo
     *                  fisheye pair.
     *
     * @param[out]      rightPoseInverse_out
     *                  Pose that maps right-camera points into the world frame.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getRightPoseInverse(Sophus::SE3<float> &rightPoseInverse_out);

    /*!
     * @brief           Returns the centre of the right camera of a stereo
     *                  fisheye pair.
     *
     * @param[out]      rightCameraCenter_out
     *                  Right camera centre in the world frame, metres.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getRightCameraCenter(Eigen::Vector3f &rightCameraCenter_out);
    /*!
     * @brief           Returns the rotation part of the right camera pose.
     *
     * @param[out]      rightRotation_out
     *                  Rotation of the world-to-right-camera pose.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getRightRotation(Eigen::Matrix<float, 3, 3> &rightRotation_out);
    /*!
     * @brief           Returns the translation part of the right camera pose.
     *
     * @param[out]      rightTranslation_out
     *                  Translation of the world-to-right-camera pose, metres.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getRightTranslation(Eigen::Vector3f &rightTranslation_out);

    /*!
     * @brief           Returns the markers detected in the frame this key frame
     *                  was created from.
     *
     * @param[out]      getCurrentFrameMarkers_out
     *                  Detected markers; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getCurrentFrameMarkers(
        std::vector<semantic::Marker *> &getCurrentFrameMarkers_out) const;
    /*!
     * @brief           Returns the map points of the frame this key frame was
     *                  created from.
     *
     * @param[out]      getCurrentFrameMapPoints_out
     *                  Map points; borrowed pointers.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getCurrentFrameMapPoints(
        std::vector<MapPoint *> &getCurrentFrameMapPoints_out) const;

    // getters and setter for point clouds
    /*!
     * @brief           Returns the colour point cloud of the frame this key
     *                  frame was created from.
     *
     * @param[out]      getCurrentFramePointCloud_out
     *                  Shared pointer to the point cloud; null once
     *                  clearPointCloud has run.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus getCurrentFramePointCloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getCurrentFramePointCloud_out)
        const;
    /*!
     * @brief           Releases the colour point cloud and the colour image
     *                  once semantic segmentation no longer needs them. The
     *                  point cloud must not already be null.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus clearPointCloud();
    /*!
     * @brief           Returns the per-class point clouds produced by semantic
     *                  segmentation.
     *
     * @param[out]      getClsCloudPtrs_out
     *                  Shared pointers to the point clouds.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus
        getClsCloudPtrs(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
                            &getClsCloudPtrs_out) const;
    /*!
     * @brief           Stores the per-class point clouds produced by semantic
     *                  segmentation.
     *
     * @param[in]       p_clsCloudPtrs_in
     *                  Shared pointers to the point clouds; copied.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus setCurrentClsCloudPtrs(
        std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>
            &p_clsCloudPtrs_in);
    /*!
     * @brief           Empties and releases the per-class point clouds. None of
     *                  the stored pointers may be null.
     *
     * @return          KEY_FRAME_STATUS_SUCCESS always.
     */
    [[nodiscard]] KeyFrameStatus clearClsClouds();
};

} // namespace core
} // namespace vs_graphs

#endif // KEYFRAME_H
