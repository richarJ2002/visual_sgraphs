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

#ifndef ROOM_H
#define ROOM_H

#include "Geometric/Plane.h"
#include "Passage.h"
#include "Thirdparty/g2o/g2o/types/vertex_plane.h"

namespace ORB_SLAM3
{

class Room
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /* ---------------------------------------------------------------------- *
     * PUBLIC MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       Enumerator which defines the types of rooms available as
     *              an object.
     */
    enum roomVariant
    {
        /*!
         * @brief       Room is not defined, due to lack of semantic
         *              information.
         */
        UNDEFINED = -1,

        /*!
         * @brief       Room contains two parallel walls.
         */
        CORRIDOR = 0,

        /*!
         * @brief       Room contains more than two walls
         */
        ROOM = 1
    };

  private:
    /* ---------------------------------------------------------------------- *
     * PRIVATE MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       The room's identifier.
     */
    int id{-1};

    /*!
     * @brief       The room's identifier in the local optimizer.
     */
    int opId{-1};

    /*!
     * @brief       The room's identifier in the global optimizer.
     */
    int opIdG{-1};

    /*!
     * @brief       Marks the room as bad (if true, the room will not be used).
     */
    bool mbBad{false};

    /*!
     * @brief       The identifier of the room's meta-marker (containing
     *              information about the room).
     */
    int metaMarkerId{-1};

    /*!
     * @brief       The name devoted for each room (optional).
     */
    std::string name;

    /*!
     * @brief       Checks if it is a candidate room (meta-marker detected) or
     *              not.
     */
    bool hasKnownLabel{false};

    /*!
     * @brief       The meta-marker assigned for the room.
     */
    Marker *metaMarker{nullptr};

    /*!
     * @brief       The ground plane associated with the room.
     */
    Plane *groundPlane{nullptr};

    /*!
     * @brief       The room's semantic type (e.g., corridor, room, etc.).
     */
    roomVariant variant{roomVariant::UNDEFINED};

    /*!
     * @brief       The center of the room as a 3D vector in the global
     *              reference.
     */
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};

    /*!
     * @brief       The vector of detected walls of a room.
     */
    std::vector<Plane *> walls;

    /*!
     * @brief       The vector of detected doorways of a room.
     */
    std::vector<ORB_SLAM3::Passage *> doorways;

  public:
    /* ---------------------------------------------------------------------- *
     * PUBLIC METHODS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       TODO
     */
    Room();

    /*!
     * @brief       TODO
     */
    ~Room();

    /*!
     * @brief       TODO
     */
    int getId() const;

    /*!
     * @brief       TODO
     */
    void setId(int value);

    /*!
     * @brief       TODO
     */
    int getOpId() const;

    /*!
     * @brief       TODO
     */
    void setOpId(int value);

    /*!
     * @brief       TODO
     */
    int getOpIdG() const;

    /*!
     * @brief       TODO
     */
    void setOpIdG(int value);

    /*!
     * @brief       TODO
     */
    bool isBad();

    /*!
     * @brief       TODO
     */
    void setBad();

    /*!
     * @brief       TODO
     */
    roomVariant getRoomVariant();

    /*!
     * @brief       TODO
     */
    void setRoomVariant(roomVariant value);

    /*!
     * @brief       TODO
     */
    bool getHasKnownLabel() const;

    /*!
     * @brief       TODO
     */
    void setHasKnownLabel(bool value);

    /*!
     * @brief       TODO
     */
    int getMetaMarkerId() const;

    /*!
     * @brief       TODO
     */
    void setMetaMarkerId(int value);

    /*!
     * @brief       TODO
     */
    Marker *getMetaMarker() const;

    /*!
     * @brief       TODO
     */
    void setMetaMarker(Marker *value);

    /*!
     * @brief       TODO
     */
    std::string getName() const;

    /*!
     * @brief       TODO
     */
    void setName(std::string value);

    /*!
     * @brief       TODO
     */
    void setDoorways(ORB_SLAM3::Passage *value);

    /*!
     * @brief       TODO
     */
    std::vector<ORB_SLAM3::Passage *> getPassages() const;

    /*!
     * @brief       TODO
     */
    void setWalls(ORB_SLAM3::Plane *value);

    /*!
     * @brief       TODO
     */
    std::vector<ORB_SLAM3::Plane *> getWalls() const;

    /*!
     * @brief       TODO
     */
    void clearWalls();

    /*!
     * @brief       TODO
     */
    Plane *getGroundPlane() const;

    /*!
     * @brief       TODO
     */
    void setGroundPlane(Plane *ground);

    /*!
     * @brief       TODO
     */
    Eigen::Vector3d getCentroid() const;

    /*!
     * @brief       TODO
     */
    void setCentroid(Eigen::Vector3d value);

    /*!
     * @brief       TODO
     */
    Map *getMap();

    /*!
     * @brief       TODO
     */
    void setMap(Map *pMap);

  protected:
    /* ---------------------------------------------------------------------- *
     * PROTECTED MEMBERS
     * ---------------------------------------------------------------------- */

    /*!
     * @brief       TODO
     */
    Map *mpMap{nullptr};

    /*!
     * @brief       TODO
     */
    std::mutex mMutexMap;
};
} // namespace ORB_SLAM3

#endif