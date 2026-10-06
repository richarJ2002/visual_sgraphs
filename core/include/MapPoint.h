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
 * @file            MapPoint.h
 *
 * @brief           Declares MapPoint, a 3-D landmark in the world frame with
 *                  its descriptor and the key frames that observe it.
 */

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "MapPointStatus.h"
#include "Utils/Converter/objects/Converter.h"

#include <boost/serialization/access.hpp>
#include <map>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <set>
#include <tuple>

namespace vs_graphs
{
namespace core
{

class KeyFrame;
class Map;
class Frame;

/*!
 * @brief           A 3-D landmark of the map: a point in the world frame that
 *                  key frames observe, with the descriptor used to match it
 *                  again and the viewing data tracking needs. Points are never
 *                  freed; a point that is no longer useful is flagged bad.
 */
class MapPoint
{

    friend class boost::serialization::access;
    /*!
     * @brief           Boost serialisation hook for saving and loading the
     *                  point. Only the identity, counters, world position,
     *                  normal, descriptor and the id-based backups of the
     *                  pointer fields are archived; call preSave() before
     *                  saving and postLoad() after loading.
     *
     * @param[in,out]   ar
     *                  Archive being written or read.
     *
     * @param[in]       version
     *                  Archive version number, forwarded to the matrix helper.
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint();

    /*!
     * @brief           Creates a point from a position triangulated for a key
     *                  frame. The key frame becomes its reference key frame and
     *                  the point gets the next free id.
     *
     * @param[in]       Pos_in
     *                  Position in the world frame.
     *
     * @param[in]       p_referenceKeyFrame_in
     *                  Reference key frame, must not be null; borrowed, not
     *                  owned.
     *
     * @param[in]       p_map_in
     *                  Map the point belongs to, must not be null; borrowed,
     *                  not owned.
     */
    MapPoint(const Eigen::Vector3f &Pos_in,
             KeyFrame              *p_referenceKeyFrame_in,
             Map                   *p_map_in);
    /*!
     * @brief           Creates a point from a position triangulated for a frame
     *                  that is not yet a key frame. The normal and the
     *                  scale-invariance distances are computed from the frame,
     *                  and the point has no reference key frame yet.
     *
     * @param[in]       Pos_in
     *                  Position in the world frame.
     *
     * @param[in]       p_map_in
     *                  Map the point belongs to, must not be null; borrowed,
     *                  not owned.
     *
     * @param[in]       p_frame_inout
     *                  Frame the point comes from, must not be null; borrowed
     *                  and only read here.
     *
     * @param[in]       indexF_in
     *                  Index of the key point in the frame. With a second
     *                  camera, an index from the left key point count upwards
     *                  is a right camera key point.
     */
    MapPoint(const Eigen::Vector3f &Pos_in,
             Map                   *p_map_in,
             Frame                 *p_frame_inout,
             const int             &indexF_in);

    /*!
     * @brief           Sets the position of the point.
     *
     * @param[in]       Pos_in
     *                  New position in the world frame.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus setWorldPos(const Eigen::Vector3f &Pos_in);
    /*!
     * @brief           Returns the position of the point.
     *
     * @param[out]      worldPos_out
     *                  Position in the world frame.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus getWorldPos(Eigen::Vector3f &worldPos_out);

    /*!
     * @brief           Returns the mean viewing direction of the point.
     *
     * @param[out]      normal_out
     *                  Unit vector in the world frame, or zero before it has
     *                  been computed.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus getNormal(Eigen::Vector3f &normal_out);
    /*!
     * @brief           Overwrites the mean viewing direction of the point.
     *
     * @param[in]       normal_in
     *                  New direction in the world frame.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus
        setNormalVector(const Eigen::Vector3f &normal_in);

    /*!
     * @brief           Returns the key frame that created the point, or the
     *                  oldest remaining observer after that key frame was
     *                  removed.
     *
     * @param[out]      p_referenceKeyFrame_out
     *                  Reference key frame; may be null; borrowed.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus
        getReferenceKeyFrame(KeyFrame *&p_referenceKeyFrame_out);

    /*!
     * @brief           Returns a copy of the observations of the point.
     *
     * @param[out]      observations_out
     *                  Maps each observing key frame to its (left, right) key
     *                  point indices; -1 means the camera does not see the
     *                  point.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus getObservations(
        std::map<KeyFrame *, std::tuple<int, int>> &observations_out);
    /*!
     * @brief           Returns how many key points observe the point.
     *
     * @param[out]      observationCount_out
     *                  Observation count; a stereo observation counts as two.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus getObservationCount(int &observationCount_out);

    /*!
     * @brief           Records that a key frame observes this point through one
     *                  of its key points. Replaces an earlier index on the same
     *                  side; stereo observations count twice.
     *
     * @param[in]       p_keyFrame_inout
     *                  Observing key frame; borrowed and only read here.
     *
     * @param[in]       index_in
     *                  Key point index in the key frame. With a second camera,
     *                  an index from the left key point count upwards is a
     *                  right camera key point.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus addObservation(KeyFrame *p_keyFrame_inout,
                                                int       index_in);
    /*!
     * @brief           Removes the observation of a key frame. If it was the
     *                  reference key frame, the observer with the lowest id
     *                  that is not bad becomes the reference. The point is
     *                  flagged bad when two or fewer observations remain. Does
     *                  nothing when the key frame is not an observer.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to remove.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus eraseObservation(KeyFrame *p_keyFrame_in);

    /*!
     * @brief           Looks up where a key frame observes the point.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to look up.
     *
     * @param[out]      indexInKeyFrame_out
     *                  (left, right) key point indices; (-1, -1) when the key
     *                  frame does not observe the point.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus
                                 getIndexInKeyFrame(KeyFrame             *p_keyFrame_in,
                                                    std::tuple<int, int> &indexInKeyFrame_out);
    /*!
     * @brief           Tells whether a key frame observes the point.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame to look up.
     *
     * @param[out]      isInKeyFrame_out
     *                  True when the key frame is an observer.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus isInKeyFrame(KeyFrame *p_keyFrame_in,
                                              bool     &isInKeyFrame_out);

    /*!
     * @brief           Flags the point bad: forgets its observations, makes
     *                  every observing key frame drop its match and removes the
     *                  point from its map. The object is not freed.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus setBadFlag();
    /*!
     * @brief           Tells whether the point has been flagged bad.
     *
     * @param[out]      isBad_out
     *                  True when flagged bad.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus isBad(bool &isBad_out);

    /*!
     * @brief           Merges this point into another one: the observers switch
     *                  to the other point (or drop their match when they
     *                  already see it), the other point inherits the found and
     *                  visible counts and its descriptor is recomputed, and
     *                  this point is flagged bad and removed from its map. Does
     *                  nothing when both have the same id.
     *
     * @param[in,out]   p_mapPoint_inout
     *                  Point that takes over the observations.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus replace(MapPoint *p_mapPoint_inout);
    /*!
     * @brief           Returns the point this one was merged into.
     *
     * @param[out]      p_replaced_out
     *                  Replacement point; null when none; borrowed.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus getReplaced(MapPoint *&p_replaced_out);

    /*!
     * @brief           Adds to the count of frames in which the point was
     *                  expected to be visible.
     *
     * @param[in]       n_in
     *                  Amount to add.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus increaseVisible(int n_in = 1);
    /*!
     * @brief           Adds to the count of frames in which the point was
     *                  actually matched.
     *
     * @param[in]       n_in
     *                  Amount to add.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus increaseFound(int n_in = 1);
    /*!
     * @brief           Returns how often the point was matched compared with
     *                  how often it was expected to be visible.
     *
     * @param[out]      foundRatio_out
     *                  Found count divided by visible count.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus getFoundRatio(float &foundRatio_out);

    /*!
     * @brief           Picks, among the descriptors of the observing key frames
     *                  that are not bad, the one with the smallest median
     *                  distance to the others and stores it as the descriptor
     *                  of the point. Does nothing when the point is bad or has
     *                  no usable observation.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus computeDistinctiveDescriptors();

    /*!
     * @brief           Returns a copy of the descriptor used to match the
     *                  point.
     *
     * @param[out]      descriptor_out
     *                  Copy of the descriptor.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus getDescriptor(cv::Mat &descriptor_out);

    /*!
     * @brief           Recomputes the mean viewing direction and the
     *                  scale-invariance distances from the current observations
     *                  and the reference key frame. Does nothing when the point
     *                  is bad or has no observation.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus updateNormalAndDepth();

    /*!
     * @brief           Returns the nearest distance at which the point is
     *                  expected to match, which is 0.8 times the stored minimum
     *                  distance.
     *
     * @param[out]      minDistanceInvariance_out
     *                  Distance, same units as the world frame.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus
        getMinDistanceInvariance(float &minDistanceInvariance_out);
    /*!
     * @brief           Returns the farthest distance at which the point is
     *                  expected to match, which is 1.2 times the stored maximum
     *                  distance.
     *
     * @param[out]      maxDistanceInvariance_out
     *                  Distance, same units as the world frame.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus
        getMaxDistanceInvariance(float &maxDistanceInvariance_out);
    /*!
     * @brief           Predicts the image pyramid level at which the point is
     *                  seen from a given distance, using the key frame pyramid.
     *
     * @param[in]       currentDistance_in
     *                  Distance between camera and point, same units as the
     *                  world frame.
     *
     * @param[in]       p_keyFrame_in
     *                  Key frame whose scale factors are used.
     *
     * @param[out]      scaleLevel_out
     *                  Predicted level, limited to the key frame levels; 0 when
     *                  the distance is zero.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus predictScale(const float &currentDistance_in,
                                              KeyFrame    *p_keyFrame_in,
                                              int         &scaleLevel_out);
    /*!
     * @brief           Predicts the image pyramid level at which the point is
     *                  seen from a given distance, using the frame pyramid.
     *
     * @param[in]       currentDistance_in
     *                  Distance between camera and point, same units as the
     *                  world frame.
     *
     * @param[in]       p_pF_in
     *                  Frame whose scale factors are used.
     *
     * @param[out]      scaleLevel_out
     *                  Predicted level, limited to the frame levels; 0 when the
     *                  distance is zero.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus predictScale(const float &currentDistance_in,
                                              Frame       *p_pF_in,
                                              int         &scaleLevel_out);

    /*!
     * @brief           Returns the map the point currently belongs to.
     *
     * @param[out]      p_map_out
     *                  Map; borrowed.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus getMap(Map *&p_map_out);
    /*!
     * @brief           Moves the point to another map.
     *
     * @param[in]       p_map_in
     *                  New map; borrowed, not owned.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus updateMap(Map *p_map_in);

    /*!
     * @brief           Prints the id of the point and of every observing key
     *                  frame with its map to standard output, for debugging.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus printObservations();

    /*!
     * @brief           Prepares the point for saving: stores the ids of the
     *                  replacement point, of the observers and of the reference
     *                  key frame, and removes observers that are not being
     *                  saved.
     *
     * @param[in]       keyFrames_in
     *                  Key frames that will be saved.
     *
     * @param[in]       mapPoints_in
     *                  Points that will be saved.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus preSave(std::set<KeyFrame *> &keyFrames_in,
                                         std::set<MapPoint *> &mapPoints_in);
    /*!
     * @brief           Restores the pointers of the point after loading from
     *                  the ids stored by preSave(), then discards the stored
     *                  ids. Observers that cannot be found are dropped; an
     *                  observer without a stored right key point index gets -1
     *                  (not seen by the right camera).
     *
     * @param[in]       keyFrameId_in
     *                  Loaded key frames by id.
     *
     * @param[in]       mapPointId_in
     *                  Loaded points by id.
     *
     * @return          MAP_POINT_STATUS_SUCCESS always.
     */
    [[nodiscard]] MapPointStatus
        postLoad(std::map<long unsigned int, KeyFrame *> &keyFrameId_in,
                 std::map<long unsigned int, MapPoint *> &mapPointId_in);

  public:
    /*!
     * @brief           Unique id of the point, assigned from nextId when it is
     *                  created.
     */
    long unsigned int        id;
    /*!
     * @brief           Id the next created point receives; shared by all maps.
     */
    static long unsigned int nextId;
    /*!
     * @brief           Id of the key frame that created the point; -1 when it
     *                  was created from a plain frame.
     */
    long int                 firstKeyFrameId;
    /*!
     * @brief           Id of the frame the point was first created in.
     */
    long int                 firstFrameId;
    /*!
     * @brief           Number of key points that observe the point; a stereo
     *                  observation counts as two.
     */
    int                      observationCount;

    // Variables used by the tracking
    /*!
     * @brief           Pixel x of the projection into the current frame set by
     *                  the frustum test; -1 when not visible.
     */
    float             trackProjX;
    /*!
     * @brief           Pixel y of the projection into the current frame set by
     *                  the frustum test; -1 when not visible.
     */
    float             trackProjY;
    /*!
     * @brief           Distance from the camera to the point in the current
     *                  frame, set by the frustum test.
     */
    float             trackDepth;
    /*!
     * @brief           Distance from the right camera to the point, set by the
     *                  frustum test.
     */
    float             trackDepthR;
    /*!
     * @brief           Pixel x of the projection into the right camera, set by
     *                  the frustum test.
     */
    float             trackProjXR;
    /*!
     * @brief           Pixel y of the projection into the right camera, set by
     *                  the frustum test.
     */
    float             trackProjYR;
    /*!
     * @brief           True when the point lies in the view of the current
     *                  frame.
     */
    bool              isTrackedInView;
    /*!
     * @brief           True when the point lies in the view of the right camera
     *                  of the current frame.
     */
    bool              isTrackedInRightView;
    /*!
     * @brief           Image pyramid level predicted for the point in the
     *                  current frame; -1 when not visible.
     */
    int               trackScaleLevel;
    /*!
     * @brief           Image pyramid level predicted for the point in the right
     *                  camera; -1 when not visible.
     */
    int               trackScaleLevelR;
    /*!
     * @brief           Cosine of the angle between the viewing ray and the
     *                  normal of the point in the current frame.
     */
    float             trackViewCos;
    /*!
     * @brief           Cosine of the same angle for the right camera.
     */
    float             trackViewCosR;
    /*!
     * @brief           Id of the frame for which tracking last added the point
     *                  to its local map points.
     */
    long unsigned int trackReferenceFrameId;
    /*!
     * @brief           Id of the frame in which the point was last seen by
     *                  tracking.
     */
    long unsigned int lastSeenFrameId;

    // Variables used by local mapping
    /*!
     * @brief           Id of the key frame whose local bundle adjustment last
     *                  included the point.
     */
    long unsigned int baLocalKeyFrameId;
    /*!
     * @brief           Id of the key frame for which the point was last a
     *                  fusion candidate.
     */
    long unsigned int fuseCandidateKeyFrameId;

    // Variables used by loop closing
    /*!
     * @brief           Id of the key frame whose loop correction last moved the
     *                  point.
     */
    long unsigned int correctedByKeyFrameId;
    /*!
     * @brief           Id of the key frame that was the reference key frame
     *                  when the loop correction moved the point.
     */
    long unsigned int correctedReferenceKeyFrameId;
    /*!
     * @brief           Position from the last global bundle adjustment, in the
     *                  world frame.
     */
    Eigen::Vector3f   posGBA;
    /*!
     * @brief           Id of the loop key frame of the global bundle adjustment
     *                  that produced posGBA.
     */
    long unsigned int baGlobalKeyFrameId;
    /*!
     * @brief           Id of the main key frame of the merge bundle adjustment
     *                  that last included the point.
     */
    long unsigned int baLocalMergeId;

    // Variable used by merging
    /*!
     * @brief           Position of the point in the merged map world frame,
     *                  computed during a map merge.
     */
    Eigen::Vector3f posMerge;
    /*!
     * @brief           Viewing direction of the point in the merged map world
     *                  frame, computed during a map merge.
     */
    Eigen::Vector3f normalVectorMerge;

    /*!
     * @brief           Guards the world position while points are moved by a
     *                  global change; held by setWorldPos().
     */
    static std::mutex globalMutex;

    /*!
     * @brief           Id of the map the point was created in, before any
     *                  merge.
     */
    unsigned int originMapId;

  protected:
    /*!
     * @brief           Position of the point in the world frame; guarded by
     *                  positionMutex.
     */
    Eigen::Vector3f worldPos;

    /*!
     * @brief           Key frames that observe the point, each with its (left,
     *                  right) key point indices (-1 = not seen by that camera);
     *                  guarded by featuresMutex.
     */
    std::map<KeyFrame *, std::tuple<int, int>> observations;
    /*!
     * @brief           Observed key frame id to left key point index; filled by
     *                  preSave() so the pointer map can be saved, emptied by
     *                  postLoad().
     */
    std::map<long unsigned int, int>           backupObservationIds1;
    /*!
     * @brief           Observed key frame id to right key point index; filled
     *                  and emptied like backupObservationIds1.
     */
    std::map<long unsigned int, int>           backupObservationIds2;

    /*!
     * @brief           Mean viewing direction of the point in the world frame,
     *                  unit length or zero; guarded by positionMutex.
     */
    Eigen::Vector3f normalVector;

    /*!
     * @brief           Descriptor used to match the point; guarded by
     *                  featuresMutex.
     */
    cv::Mat descriptor;

    /*!
     * @brief           Key frame that created the point, or the oldest
     *                  remaining observer; may be null; borrowed; guarded by
     *                  featuresMutex.
     */
    KeyFrame         *p_referenceKeyFrame;
    /*!
     * @brief           Id of the reference key frame; stored by preSave() and
     *                  read by postLoad().
     */
    long unsigned int backupRefKeyFrameId;

    /*!
     * @brief           Number of frames in which the point was expected to be
     *                  visible; guarded by featuresMutex.
     */
    int visibleCount;
    /*!
     * @brief           Number of frames in which the point was matched; guarded
     *                  by featuresMutex.
     */
    int foundCount;

    /*!
     * @brief           True once the point is flagged bad. The object is kept
     *                  in memory.
     */
    bool          isFlaggedBad;
    /*!
     * @brief           Point this one was merged into; null when none;
     *                  borrowed.
     */
    MapPoint     *p_replaced;
    /*!
     * @brief           Id of the replacement point stored by preSave(); -1 when
     *                  none.
     */
    long long int backupReplacedId;

    /*!
     * @brief           Smallest distance at which the point is matched, in
     *                  world frame units; guarded by positionMutex.
     */
    float minDistance;
    /*!
     * @brief           Largest distance at which the point is matched, in world
     *                  frame units; guarded by positionMutex.
     */
    float maxDistance;

    /*!
     * @brief           Map the point belongs to; borrowed; guarded by mapMutex.
     */
    Map *p_map;

    // Mutex
    /*!
     * @brief           Guards worldPos, normalVector, minDistance and
     *                  maxDistance.
     */
    std::mutex positionMutex;
    /*!
     * @brief           Guards the observations, descriptor, reference key frame
     *                  and counters.
     */
    std::mutex featuresMutex;
    /*!
     * @brief           Guards p_map.
     */
    std::mutex mapMutex;
};

} // namespace core
} // namespace vs_graphs

#endif // MAPPOINT_H
