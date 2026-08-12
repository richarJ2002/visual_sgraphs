/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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

#include "Semantic/Floor.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ORB_SLAM3
{

Floor::Floor() :
    id(-1),
    opId(-1),
    opIdG(-1),
    name(""),
    centroid(Eigen::Vector3d::Zero()),
    mpMap(nullptr)
{}
Floor::~Floor() {}

void Floor::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);

    centroid = transform_oldWorldToNewWorld_in.map(centroid);

    if (planeIdentity.has_value())
    {
        const std::optional<PlaneIdentity> transformedIdentity =
            transformPlaneIdentity(*planeIdentity,
                                   transform_oldWorldToNewWorld_in);

        if (transformedIdentity.has_value())
        {
            planeIdentity = *transformedIdentity;
        }
        else
        {
            planeIdentity.reset();
        }
    }
}

int Floor::getId() const
{
    return id;
}

void Floor::setId(int value)
{
    id = value;
}

int Floor::getOpId() const
{
    return opId;
}

void Floor::setOpId(int value)
{
    opId = value;
}

int Floor::getOpIdG() const
{
    return opIdG;
}

void Floor::setOpIdG(int value)
{
    opIdG = value;
}

std::string Floor::getName() const
{
    return name;
}

void Floor::setName(std::string value)
{
    name = value;
}

Eigen::Vector3d Floor::getCentroid() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return centroid;
}

void Floor::setCentroid(Eigen::Vector3d value)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    centroid = value;
}

bool Floor::hasPlaneIdentity() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return planeIdentity.has_value();
}

std::optional<Floor::PlaneIdentity> Floor::getPlaneIdentity() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return planeIdentity;
}

bool Floor::setPlaneIdentity(const Eigen::Vector4d &equation_World_in,
                             const std::size_t finiteSupportCount_in,
                             const std::size_t observationCount_in)
{
    Eigen::Vector4d normalizedEquation_World = equation_World_in;
    const double normalNorm = normalizedEquation_World.head<3>().norm();

    if (!normalizedEquation_World.allFinite() || !std::isfinite(normalNorm) ||
        normalNorm < 1e-8)
    {
        return false;
    }

    normalizedEquation_World /= normalNorm;

    std::lock_guard<std::mutex> lock(mMutexGeometry);
    planeIdentity = PlaneIdentity{normalizedEquation_World,
                                  finiteSupportCount_in,
                                  observationCount_in};
    return true;
}

void Floor::clearPlaneIdentity(void)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    planeIdentity.reset();
}

std::optional<Floor::PlaneIdentity> Floor::transformPlaneIdentity(
    const PlaneIdentity &identity_OldWorld_in,
    const g2o::Sim3      &transform_oldWorldToNewWorld_in)
{
    Eigen::Vector4d equation_OldWorld = identity_OldWorld_in.equation_World;
    const double oldNormalNorm = equation_OldWorld.head<3>().norm();

    if (!equation_OldWorld.allFinite() || !std::isfinite(oldNormalNorm) ||
        oldNormalNorm < 1e-8)
    {
        return std::nullopt;
    }

    equation_OldWorld /= oldNormalNorm;

    const Eigen::Matrix3d rotation_oldWorldToNewWorld =
        transform_oldWorldToNewWorld_in.rotation().toRotationMatrix();
    const Eigen::Vector3d translation_NewWorld =
        transform_oldWorldToNewWorld_in.translation();
    const double scale = transform_oldWorldToNewWorld_in.scale();

    if (!rotation_oldWorldToNewWorld.allFinite() ||
        !translation_NewWorld.allFinite() || !std::isfinite(scale))
    {
        return std::nullopt;
    }

    const Eigen::Vector3d normal_NewWorld =
        rotation_oldWorldToNewWorld * equation_OldWorld.head<3>();
    Eigen::Vector4d equation_NewWorld;
    equation_NewWorld << normal_NewWorld,
        scale * equation_OldWorld(3) -
            normal_NewWorld.dot(translation_NewWorld);

    const double newNormalNorm = equation_NewWorld.head<3>().norm();
    if (!equation_NewWorld.allFinite() || !std::isfinite(newNormalNorm) ||
        newNormalNorm < 1e-8)
    {
        return std::nullopt;
    }

    equation_NewWorld /= newNormalNorm;
    return PlaneIdentity{equation_NewWorld,
                         identity_OldWorld_in.finiteSupportCount,
                         identity_OldWorld_in.observationCount};
}

bool Floor::planeIdentitiesMatch(const PlaneIdentity &firstIdentity_in,
                                 const PlaneIdentity &secondIdentity_in,
                                 const double maximumNormalAngle_deg_in,
                                 const double maximumOffset_m_in,
                                 double      &normalAngle_deg_out,
                                 double      &offset_m_out)
{
    Eigen::Vector4d firstEquation  = firstIdentity_in.equation_World;
    Eigen::Vector4d secondEquation = secondIdentity_in.equation_World;
    const double firstNormalNorm   = firstEquation.head<3>().norm();
    const double secondNormalNorm  = secondEquation.head<3>().norm();

    if (!firstEquation.allFinite() || !secondEquation.allFinite() ||
        firstNormalNorm < 1e-8 || secondNormalNorm < 1e-8)
    {
        normalAngle_deg_out = std::numeric_limits<double>::infinity();
        offset_m_out        = std::numeric_limits<double>::infinity();
        return false;
    }

    firstEquation /= firstNormalNorm;
    secondEquation /= secondNormalNorm;

    double normalDot = firstEquation.head<3>().dot(secondEquation.head<3>());
    if (normalDot < 0.0)
    {
        secondEquation = -secondEquation;
        normalDot      = -normalDot;
    }

    normalDot = std::clamp(normalDot, -1.0, 1.0);
    normalAngle_deg_out =
        std::acos(normalDot) * 180.0 / std::acos(-1.0);
    offset_m_out = std::abs(firstEquation(3) - secondEquation(3));

    return normalAngle_deg_out <= maximumNormalAngle_deg_in &&
           offset_m_out <= maximumOffset_m_in;
}

Floor *Floor::selectBestObservedFloor(const std::vector<Floor *> &floors_in)
{
    Floor                        *p_bestFloor = nullptr;
    std::optional<PlaneIdentity> bestIdentity;

    for (Floor *p_candidateFloor : floors_in)
    {
        if (p_candidateFloor == nullptr)
        {
            continue;
        }

        const std::optional<PlaneIdentity> candidateIdentity =
            p_candidateFloor->getPlaneIdentity();

        const bool candidateIsBetter =
            candidateIdentity.has_value() &&
            (!bestIdentity.has_value() ||
             candidateIdentity->finiteSupportCount >
                 bestIdentity->finiteSupportCount ||
             (candidateIdentity->finiteSupportCount ==
                  bestIdentity->finiteSupportCount &&
              candidateIdentity->observationCount >
                  bestIdentity->observationCount));

        const bool evidenceIsEqual =
            candidateIdentity.has_value() == bestIdentity.has_value() &&
            (!candidateIdentity.has_value() ||
             (candidateIdentity->finiteSupportCount ==
                  bestIdentity->finiteSupportCount &&
              candidateIdentity->observationCount ==
                  bestIdentity->observationCount));

        if (p_bestFloor == nullptr || candidateIsBetter ||
            (evidenceIsEqual &&
             p_candidateFloor->getId() < p_bestFloor->getId()))
        {
            p_bestFloor = p_candidateFloor;
            bestIdentity = candidateIdentity;
        }
    }

    return p_bestFloor;
}

std::vector<ORB_SLAM3::Room *> Floor::getRooms() const
{
    std::lock_guard<std::mutex> lock(mMutexRooms);
    return rooms;
}

void Floor::addRoom(ORB_SLAM3::Room *value)
{
    if (value == nullptr)
    {
        return;
    }

    Floor *p_previousFloor = value->getFloor();
    if (p_previousFloor != nullptr && p_previousFloor != this)
    {
        p_previousFloor->detachRoom(value);
    }

    {
        std::lock_guard<std::mutex> lock(mMutexRooms);
        const bool alreadyPresent =
            std::find(rooms.begin(), rooms.end(), value) != rooms.end();

        if (!alreadyPresent)
        {
            rooms.push_back(value);
        }
    }

    value->setFloor(this);
}

void Floor::setRooms(const std::vector<ORB_SLAM3::Room *> &value)
{
    std::vector<Room *> newRooms;
    newRooms.reserve(value.size());

    for (Room *p_room : value)
    {
        if (p_room != nullptr &&
            std::find(newRooms.begin(), newRooms.end(), p_room) ==
                newRooms.end())
        {
            Floor *p_previousFloor = p_room->getFloor();
            if (p_previousFloor != nullptr && p_previousFloor != this)
            {
                p_previousFloor->detachRoom(p_room);
            }
            newRooms.push_back(p_room);
        }
    }

    std::vector<Room *> oldRooms;
    {
        std::lock_guard<std::mutex> lock(mMutexRooms);
        oldRooms = rooms;
        rooms    = newRooms;
    }

    for (Room *p_oldRoom : oldRooms)
    {
        if (p_oldRoom != nullptr &&
            std::find(newRooms.begin(), newRooms.end(), p_oldRoom) ==
                newRooms.end() &&
            p_oldRoom->getFloor() == this)
        {
            p_oldRoom->setFloor(nullptr);
        }
    }

    for (Room *p_newRoom : newRooms)
    {
        p_newRoom->setFloor(this);
    }
}

bool Floor::replaceRoom(Room *p_retiredRoom_in, Room *p_retainedRoom_in)
{
    if (p_retiredRoom_in == nullptr || p_retainedRoom_in == nullptr ||
        p_retiredRoom_in == p_retainedRoom_in)
    {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mMutexRooms);
        if (std::find(rooms.begin(), rooms.end(), p_retiredRoom_in) ==
            rooms.end())
        {
            return false;
        }
    }

    Floor *p_previousRetainedFloor = p_retainedRoom_in->getFloor();
    if (p_previousRetainedFloor != nullptr &&
        p_previousRetainedFloor != this)
    {
        p_previousRetainedFloor->detachRoom(p_retainedRoom_in);
    }

    bool                replacedRetiredRoom = false;
    std::vector<Room *> rebuiltRooms;
    {
        std::lock_guard<std::mutex> lock(mMutexRooms);
        rebuiltRooms.reserve(rooms.size());

        for (Room *p_existingRoom : rooms)
        {
            Room *p_candidateRoom = p_existingRoom;

            if (p_existingRoom == p_retiredRoom_in)
            {
                p_candidateRoom     = p_retainedRoom_in;
                replacedRetiredRoom = true;
            }

            if (p_candidateRoom == nullptr ||
                std::find(rebuiltRooms.begin(),
                          rebuiltRooms.end(),
                          p_candidateRoom) != rebuiltRooms.end())
            {
                continue;
            }

            rebuiltRooms.push_back(p_candidateRoom);
        }

        if (replacedRetiredRoom)
        {
            rooms.swap(rebuiltRooms);
        }
    }

    if (replacedRetiredRoom)
    {
        if (p_retiredRoom_in->getFloor() == this)
        {
            p_retiredRoom_in->setFloor(nullptr);
        }
        p_retainedRoom_in->setFloor(this);
    }

    return replacedRetiredRoom;
}

void Floor::detachRoom(Room *p_room_in)
{
    if (p_room_in == nullptr)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mMutexRooms);
        rooms.erase(std::remove(rooms.begin(), rooms.end(), p_room_in),
                    rooms.end());
    }

    if (p_room_in->getFloor() == this)
    {
        p_room_in->setFloor(nullptr);
    }
}

ORB_SLAM3::Map *Floor::getMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void Floor::setMap(ORB_SLAM3::Map *pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}
} // namespace ORB_SLAM3
