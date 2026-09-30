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

class Marker
{
  public:
    /*!
     * @brief        Enumerator which defines the semantic types a marker
     *               can be labeled with.
     */
    enum class MarkerVariant : std::int8_t
    {
        UNKNOWN        = -1,
        ON_DOOR        = 0,
        ON_WALL        = 1,
        ON_ROOM_CENTER = 2
    };

  private:
    int    id;            // The marker's identifier
    int    opId;          // The marker's identifier in the local optimizer
    int    opIdG;         // The marker's identifier in the global optimizer
    double time;          // The timestamp (in seconds) of observing the marker
    bool   isInGlobalMap; // Check if the marker is in the Global Map or not
    Sophus::SE3f
        localPose; // Marker's pose (position and orientation) in the Local Map
    Sophus::SE3f  globalPose; // Marker's pose (position and orientation) in the
                              // Global Map
    MarkerVariant markerType; // The semantic object the marker is labeled with
                              // (e.g., wall, etc.)
    std::map<KeyFrame *, Sophus::SE3f>
        observations; // Marker's observations in KeyFrames

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
     * @brief       Applies a map-frame similarity transform to the marker.
     *
     * @param[in]   transform_oldWorldToNewWorld_in
     *              Transform from the old map frame to the surviving frame.
     */
    [[nodiscard]] MarkerStatus
        applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in);

    [[nodiscard]] MarkerStatus getId(int &id_out) const;
    [[nodiscard]] MarkerStatus setId(int id_in);

    [[nodiscard]] MarkerStatus getOpId(int &opId_out) const;
    [[nodiscard]] MarkerStatus setOpId(int opId_in);

    [[nodiscard]] MarkerStatus getOpIdG(int &opIdG_out) const;
    [[nodiscard]] MarkerStatus setOpIdG(int opIdG_in);

    [[nodiscard]] MarkerStatus getTime(double &time_out) const;
    [[nodiscard]] MarkerStatus setTime(double timestamp_in);

    [[nodiscard]] MarkerStatus
        getMarkerType(Marker::MarkerVariant &markerType_out) const;
    [[nodiscard]] MarkerStatus setMarkerType(MarkerVariant newType_in);

    [[nodiscard]] MarkerStatus isMarkerInGMap(bool &isMarkerInGMap_out) const;
    [[nodiscard]] MarkerStatus setMarkerInGMap(bool isInGlobalMap_in);

    [[nodiscard]] MarkerStatus getLocalPose(Sophus::SE3f &localPose_out) const;
    [[nodiscard]] MarkerStatus setLocalPose(const Sophus::SE3f &localPose_in);

    [[nodiscard]] MarkerStatus
        getGlobalPose(Sophus::SE3f &globalPose_out) const;
    [[nodiscard]] MarkerStatus setGlobalPose(const Sophus::SE3f &globalPose_in);

    /*!
     * @brief       Adds or replaces a marker observation from one keyframe.
     *
     * @param[in]   p_keyFrame_in
     *              Non-owning observing keyframe pointer.
     *
     * @param[in]   markerPose_markerToCamera_in
     *              Marker pose expressed in the observing camera frame.
     */
    [[nodiscard]] MarkerStatus
        addObservation(KeyFrame           *p_keyFrame_in,
                       const Sophus::SE3f &markerPose_markerToCamera_in);

    /*!
     * @brief Removes an observation before its keyframe is retired.
     *
     * @param[in] p_keyFrame_in Non-owning observing keyframe pointer.
     */
    [[nodiscard]] MarkerStatus eraseObservation(KeyFrame *p_keyFrame_in);

    /*!
     * @brief       Returns a thread-safe snapshot of marker observations.
     */
    [[nodiscard]] MarkerStatus getObservations(
        std::map<core::KeyFrame *, Sophus::SE3f> &observations_out) const;

    [[nodiscard]] MarkerStatus getMap(core::Map *&p_map_out);
    [[nodiscard]] MarkerStatus setMap(Map *p_map_in);

  protected:
    Map               *p_map{nullptr};
    std::mutex         mapMutex;
    mutable std::mutex geometryMutex;
    mutable std::mutex stateMutex;
    mutable std::mutex observationsMutex;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif
