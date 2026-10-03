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

/*!
 * @file            Marker.h
 *
 * @brief           Declares Marker, a fiducial marker seen by the camera and
 *                  placed in the map.
 */

#ifndef MARKER_H
#define MARKER_H

#include <cstdint>
#include <map>
#include <mutex>

#include "Semantic/MarkerStatus.h"
#include "Thirdparty/Sophus/sophus/se3.hpp"
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace vs_graphs
{
namespace core
{
class Map;
class KeyFrame;
namespace semantic
{

/*!
 * @brief           A fiducial marker (such as an ArUco tag) seen by the camera,
 *                  with its pose in the local and global maps and the key
 *                  frames that observed it.
 */
class Marker
{
  public:
    /*!
     * @brief           Enumerator which defines the semantic types a marker
     *                  can be labeled with.
     */
    enum class MarkerVariant : std::int8_t
    {
        UNKNOWN        = -1,
        ON_DOOR        = 0,
        ON_WALL        = 1,
        ON_ROOM_CENTER = 2
    };

  private:
    /*!
     * @brief           Identifier of the marker itself; the same physical
     *                  marker has the same id in every key frame and map, which
     *                  is how frame markers are matched to map markers.
     */
    int id;

    /*!
     * @brief           Identifier of this marker in the local optimizer.
     */
    int opId;

    /*!
     * @brief           Identifier of this marker in the global optimizer.
     */
    int opIdG;

    /*!
     * @brief           Time at which the marker was observed, in seconds.
     *                  Guarded by stateMutex.
     */
    double time;

    /*!
     * @brief           True once the marker has been matched to a marker in the
     *                  global map. Guarded by stateMutex.
     */
    bool isInGlobalMap;

    /*!
     * @brief           Pose (position and orientation) of the marker in the
     *                  local map. Only copied from another marker; never
     *                  recomputed here. Guarded by stateMutex.
     */
    Sophus::SE3f localPose;

    /*!
     * @brief           Pose of the marker in the global map, mapping points
     *                  from the marker frame into the world frame. Guarded by
     *                  geometryMutex.
     */
    Sophus::SE3f globalPose;

    /*!
     * @brief           The kind of structure the marker is attached to (door,
     *                  wall or room centre); UNKNOWN until labelled. Guarded by
     *                  stateMutex.
     */
    MarkerVariant markerType;

    /*!
     * @brief           Key frames that observed this marker, each with the
     *                  marker pose in that key frame's camera frame. The key
     *                  frames are borrowed. Guarded by observationsMutex.
     */
    std::map<KeyFrame *, Sophus::SE3f> observations;

  public:
    Marker() :
        id(-1),
        opId(-1),
        opIdG(-1),
        time(0.0),
        isInGlobalMap(false),
        localPose(Sophus::SE3f()),
        globalPose(Sophus::SE3f()),
        markerType(MarkerVariant::UNKNOWN),
        p_map(nullptr)
    {}

    /*!
     * @brief           Applies a map-frame similarity transform to the marker.
     *
     * @param[in]       alignmentTransform_oldWorldToNewWorld_in
     *                  Transform from the old map frame to the surviving frame.
     */
    [[nodiscard]] MarkerStatus applyTransform(
        const g2o::Sim3 &alignmentTransform_oldWorldToNewWorld_in);

    /*!
     * @brief           Returns the identifier of this marker.
     *
     * @param[out]      id_out
     *                  Marker identifier; -1 = not assigned yet.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus getId(int &id_out) const;
    /*!
     * @brief           Sets the identifier of this marker.
     *
     * @param[in]       id_in
     *                  New marker identifier.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setId(int id_in);

    /*!
     * @brief           Returns the identifier of this marker in the local
     *                  optimizer.
     *
     * @param[out]      opId_out
     *                  Local optimizer identifier; -1 = not assigned yet.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus getOpId(int &opId_out) const;
    /*!
     * @brief           Sets the identifier of this marker in the local
     *                  optimizer.
     *
     * @param[in]       opId_in
     *                  New local optimizer identifier.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setOpId(int opId_in);

    /*!
     * @brief           Returns the identifier of this marker in the global
     *                  optimizer.
     *
     * @param[out]      opIdG_out
     *                  Global optimizer identifier; -1 = not assigned yet.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus getOpIdG(int &opIdG_out) const;
    /*!
     * @brief           Sets the identifier of this marker in the global
     *                  optimizer.
     *
     * @param[in]       opIdG_in
     *                  New global optimizer identifier.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setOpIdG(int opIdG_in);

    /*!
     * @brief           Returns the time at which this marker was observed.
     *
     * @param[out]      time_out
     *                  Observation time, seconds.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus getTime(double &time_out) const;
    /*!
     * @brief           Sets the time at which this marker was observed.
     *
     * @param[in]       timestamp_in
     *                  Observation time, seconds.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setTime(double timestamp_in);

    /*!
     * @brief           Returns the kind of structure this marker is attached
     *                  to.
     *
     * @param[out]      markerType_out
     *                  Marker type; UNKNOWN until labelled.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus
        getMarkerType(Marker::MarkerVariant &markerType_out) const;
    /*!
     * @brief           Sets the kind of structure this marker is attached to.
     *
     * @param[in]       newType_in
     *                  New marker type.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setMarkerType(MarkerVariant newType_in);

    /*!
     * @brief           Tells whether this marker has been matched to a marker
     *                  in the global map.
     *
     * @param[out]      isMarkerInGMap_out
     *                  True when the marker is in the global map.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus isMarkerInGMap(bool &isMarkerInGMap_out) const;
    /*!
     * @brief           Records whether this marker is in the global map.
     *
     * @param[in]       isInGlobalMap_in
     *                  True when the marker is in the global map.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setMarkerInGMap(bool isInGlobalMap_in);

    /*!
     * @brief           Returns the pose of this marker in the local map.
     *
     * @param[out]      localPose_out
     *                  Marker pose in the local map.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus getLocalPose(Sophus::SE3f &localPose_out) const;
    /*!
     * @brief           Sets the pose of this marker in the local map.
     *
     * @param[in]       localPose_in
     *                  New marker pose in the local map.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setLocalPose(const Sophus::SE3f &localPose_in);

    /*!
     * @brief           Returns the pose of this marker in the global map.
     *
     * @param[out]      globalPose_out
     *                  Marker pose mapping marker-frame points into the
     *                  world frame.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus
        getGlobalPose(Sophus::SE3f &globalPose_out) const;
    /*!
     * @brief           Sets the pose of this marker in the global map.
     *
     * @param[in]       globalPose_in
     *                  New marker pose mapping marker-frame points into
     *                  the world frame.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setGlobalPose(const Sophus::SE3f &globalPose_in);

    /*!
     * @brief           Adds or replaces a marker observation from one keyframe.
     *
     * @param[in]       p_keyFrame_in
     *                  Non-owning observing keyframe pointer.
     *
     * @param[in]       markerPose_markerToCamera_in
     *                  Marker pose expressed in the observing camera frame.
     */
    [[nodiscard]] MarkerStatus
        addObservation(KeyFrame           *p_keyFrame_in,
                       const Sophus::SE3f &markerPose_markerToCamera_in);

    /*!
     * @brief           Removes an observation before its keyframe is retired.
     *
     * @param[in]       p_keyFrame_in
     *                  Non-owning observing keyframe pointer.
     */
    [[nodiscard]] MarkerStatus eraseObservation(KeyFrame *p_keyFrame_in);

    /*!
     * @brief           Returns a thread-safe snapshot of marker observations.
     */
    [[nodiscard]] MarkerStatus getObservations(
        std::map<core::KeyFrame *, Sophus::SE3f> &observations_out) const;

    /*!
     * @brief           Returns the map this marker belongs to.
     *
     * @param[out]      p_map_out
     *                  Borrowed map pointer; null when no map was set.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus getMap(core::Map *&p_map_out);
    /*!
     * @brief           Sets the map this marker belongs to.
     *
     * @param[in]       p_map_in
     *                  Borrowed map pointer; the marker does not delete it.
     *
     * @return          MARKER_STATUS_SUCCESS always.
     */
    [[nodiscard]] MarkerStatus setMap(Map *p_map_in);

  protected:
    /*!
     * @brief           Map this marker belongs to; borrowed, null = none.
     *                  Guarded by mapMutex.
     */
    Map *p_map{nullptr};

    /*!
     * @brief           Guards p_map.
     */
    std::mutex mapMutex;

    /*!
     * @brief           Guards the global pose.
     */
    mutable std::mutex geometryMutex;

    /*!
     * @brief           Guards the time, the global-map flag, the local pose and
     *                  the marker type.
     */
    mutable std::mutex stateMutex;

    /*!
     * @brief           Guards the key frame observations.
     */
    mutable std::mutex observationsMutex;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif
