/**
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

#ifndef MARKER_H
#define MARKER_H

#include "KeyFrame.h"
#include "Map.h"

namespace ORB_SLAM3
{
class Map;
class KeyFrame;

class Marker
{
  public:
    enum markerVariant
    {
        UNKNOWN        = -1,
        ON_DOOR        = 0,
        ON_WALL        = 1,
        ON_ROOM_CENTER = 2
    };

  private:
    int    id;           // The marker's identifier
    int    opId;         // The marker's identifier in the local optimizer
    int    opIdG;        // The marker's identifier in the global optimizer
    double time;         // The timestamp (in seconds) of observing the marker
    bool   markerInGMap; // Check if the marker is in the Global Map or not
    Sophus::SE3f
        localPose; // Marker's pose (position and orientation) in the Local Map
    Sophus::SE3f  globalPose; // Marker's pose (position and orientation) in the
                              // Global Map
    markerVariant markerType; // The semantic object the marker is labeled with
                              // (e.g., wall, etc.)
    std::map<KeyFrame *, Sophus::SE3f>
        observations; // Marker's observations in KeyFrames

  public:
    Marker();
    ~Marker();

    /*!
     * @brief       Applies a map-frame similarity transform to the marker.
     *
     * @param[in]   transform_oldWorldToNewWorld_in
     *              Transform from the old map frame to the surviving frame.
     */
    void applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in);

    int  getId() const;
    void setId(int value);

    int  getOpId() const;
    void setOpId(int value);

    int  getOpIdG() const;
    void setOpIdG(int value);

    double getTime() const;
    void   setTime(double value);

    markerVariant getMarkerType() const;
    void          setMarkerType(markerVariant newType);

    bool isMarkerInGMap() const;
    void setMarkerInGMap(bool value);

    Sophus::SE3f getLocalPose() const;
    void         setLocalPose(const Sophus::SE3f &value);

    Sophus::SE3f getGlobalPose() const;
    void         setGlobalPose(const Sophus::SE3f &value);

    /*!
     * @brief       Adds or replaces a marker observation from one keyframe.
     *
     * @param[in]   p_keyFrame_in
     *              Non-owning observing keyframe pointer.
     *
     * @param[in]   markerPose_markerToCamera_in
     *              Marker pose expressed in the observing camera frame.
     */
    void addObservation(KeyFrame           *p_keyFrame_in,
                        const Sophus::SE3f &markerPose_markerToCamera_in);

    /**
     * @brief Removes an observation before its keyframe is retired.
     *
     * @param[in] p_keyFrame_in Non-owning observing keyframe pointer.
     */
    void eraseObservation(KeyFrame *p_keyFrame_in);

    /*!
     * @brief       Returns a thread-safe snapshot of marker observations.
     */
    std::map<KeyFrame *, Sophus::SE3f> getObservations() const;

    Map *getMap();
    void setMap(Map *pMap);

  protected:
    Map               *mpMap;
    std::mutex         mMutexMap;
    mutable std::mutex mMutexGeometry;
    mutable std::mutex mMutexState;
    mutable std::mutex mMutexObservations;
};

} // namespace ORB_SLAM3

#endif
