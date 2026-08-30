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

#include "Semantic/Room.h"
#include <algorithm>
#include <cmath>

namespace ORB_SLAM3
{

Room::Room() = default;

Room::~Room() = default;

void Room::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    unique_lock<mutex> lock(mMutexMap);

    centroid = transform_oldWorldToNewWorld_in.map(centroid);
}

int Room::getId() const
{
    return id;
}

void Room::setId(int value)
{
    id = value;
}

int Room::getOpId() const
{
    return opId;
}

void Room::setOpId(int value)
{
    opId = value;
}

int Room::getOpIdG() const
{
    return opIdG;
}

void Room::setOpIdG(int value)
{
    opIdG = value;
}

void Room::setBad()
{
    unique_lock<mutex> lock(mMutexMap);
    mbBad = true;
}

bool Room::isBad()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbBad;
}

int Room::getMetaMarkerId() const
{
    return metaMarkerId;
}

void Room::setMetaMarkerId(int value)
{
    metaMarkerId = value;
}

Marker *Room::getMetaMarker() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return metaMarker;
}

void Room::setMetaMarker(Marker *value)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    metaMarker = value;
}

std::string Room::getName() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return name;
}

void Room::setName(std::string value)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    name = value;
}

std::string Room::getRoomTag() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return mRoomTag;
}

void Room::setRoomTag(const std::string &tag)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    mRoomTag = tag;
}

bool Room::hasRoomTag() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return !mRoomTag.empty();
}

void Room::setMatchedContext(RoomContextSnapshot *ctx)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    mpMatchedContext = ctx;
}

RoomContextSnapshot *Room::getMatchedContext() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return mpMatchedContext;
}

Room::roomVariant Room::getRoomVariant()
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return variant;
}

void Room::setRoomVariant(Room::roomVariant value)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    variant = value;
}

Room::BoundaryStatus Room::getBoundaryStatus() const
{
    std::lock_guard<std::mutex> boundaryStatusLock(mMutexBoundaryStatus);
    return boundaryStatus;
}

void Room::setBoundaryStatus(const BoundaryStatus boundaryStatus_in)
{
    std::lock_guard<std::mutex> boundaryStatusLock(mMutexBoundaryStatus);
    boundaryStatus = boundaryStatus_in;
}

bool Room::isBoundaryComplete() const
{
    return getBoundaryStatus() == BoundaryStatus::COMPLETE;
}

bool Room::getHasKnownLabel() const
{
    return hasKnownLabel;
}

void Room::setHasKnownLabel(bool value)
{
    hasKnownLabel = value;
}

std::vector<Plane *> Room::getWalls() const
{
    std::lock_guard<std::mutex> lock(mMutexWalls);
    return walls;
}

std::optional<Eigen::Vector3d>
    Room::getWallNormalTowardRoom_World(const Plane *p_wall_in) const
{
    /* Reject a missing wall association. */
    if (p_wall_in == nullptr)
    {
        return std::nullopt;
    }

    /* Read, but never modify, the globally expressed wall equation. */
    Eigen::Vector4d wallEquation_World =
        p_wall_in->getGlobalEquation().coeffs();

    Eigen::Vector3d roomCentroid_World_m;

    {
        std::lock_guard<std::mutex> lock(mMutexMap);
        roomCentroid_World_m = centroid;
    }

    /* Reject non-finite geometry before evaluating its signed distance. */
    if (!wallEquation_World.allFinite() || !roomCentroid_World_m.allFinite())
    {
        return std::nullopt;
    }

    const double wallNormalNorm = wallEquation_World.head<3>().norm();

    /* A plane without a usable normal has no defined orientation. */
    constexpr double minimumWallNormalNorm = 1e-8;

    if (!std::isfinite(wallNormalNorm) ||
        wallNormalNorm < minimumWallNormalNorm)
    {
        return std::nullopt;
    }

    /* Normalize all coefficients so the signed value is measured in metres. */
    wallEquation_World /= wallNormalNorm;

    Eigen::Vector3d wallNormalTowardRoom_World = wallEquation_World.head<3>();

    const double roomSignedDistanceToWall_m =
        wallNormalTowardRoom_World.dot(roomCentroid_World_m) +
        wallEquation_World(3);

    if (!std::isfinite(roomSignedDistanceToWall_m))
    {
        return std::nullopt;
    }

    /* Flip only the returned value when the stored normal points away. */
    if (roomSignedDistanceToWall_m < 0.0)
    {
        wallNormalTowardRoom_World *= -1.0;
    }

    return wallNormalTowardRoom_World;
}

void Room::setWalls(Plane *p_wall_in)
{
    /* Confirm that input wall is valid */
    if (p_wall_in == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mMutexWalls);

    /* Deduplicate membership within this room; the manager owns global policy.
     */
    const bool alreadyAssociated =
        std::any_of(walls.begin(),
                    walls.end(),
                    [p_wall_in](const Plane *p_existingWall)
                    { return p_existingWall == p_wall_in; });

    if (!alreadyAssociated)
    {
        walls.push_back(p_wall_in);
    }
}

bool Room::replaceWall(Plane *p_retiredWall_in, Plane *p_retainedWall_in)
{
    if (p_retiredWall_in == nullptr || p_retainedWall_in == nullptr ||
        p_retiredWall_in == p_retainedWall_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexWalls);

    bool                 replacedRetiredWall = false;
    std::vector<Plane *> rebuiltWalls;
    rebuiltWalls.reserve(walls.size());

    for (Plane *p_existingWall : walls)
    {
        Plane *p_candidateWall = p_existingWall;

        if (p_existingWall == p_retiredWall_in)
        {
            p_candidateWall     = p_retainedWall_in;
            replacedRetiredWall = true;
        }

        if (p_candidateWall == nullptr ||
            std::find(rebuiltWalls.begin(),
                      rebuiltWalls.end(),
                      p_candidateWall) != rebuiltWalls.end())
        {
            continue;
        }

        rebuiltWalls.push_back(p_candidateWall);
    }

    if (replacedRetiredWall)
    {
        walls.swap(rebuiltWalls);
    }

    return replacedRetiredWall;
}

bool Room::removeWall(Plane *p_wall_in)
{
    if (p_wall_in == nullptr)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexWalls);

    const auto wallIterator =
        std::remove(walls.begin(), walls.end(), p_wall_in);
    const bool removedWall = wallIterator != walls.end();
    walls.erase(wallIterator, walls.end());

    return removedWall;
}

void Room::clearWalls()
{
    std::lock_guard<std::mutex> lock(mMutexWalls);
    walls.clear();
}

std::size_t Room::removeInvalidWalls()
{
    std::lock_guard<std::mutex> lock(mMutexWalls);

    walls.erase(std::remove_if(walls.begin(),
                               walls.end(),
                               [](Plane *p_wall) {
                                   return p_wall == nullptr || p_wall->isBad();
                               }),
                walls.end());

    return walls.size();
}

Plane *Room::getGroundPlane() const
{
    std::lock_guard<std::mutex> lock(mMutexWalls);
    return groundPlane;
}

void Room::setGroundPlane(Plane *p_groundPlane_in)
{
    std::lock_guard<std::mutex> lock(mMutexWalls);
    groundPlane = p_groundPlane_in;
}

bool Room::replaceGroundPlane(Plane *p_retiredGround_in,
                              Plane *p_retainedGround_in)
{
    if (p_retiredGround_in == nullptr || p_retainedGround_in == nullptr ||
        p_retiredGround_in == p_retainedGround_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexWalls);

    if (groundPlane != p_retiredGround_in)
    {
        return false;
    }

    groundPlane = p_retainedGround_in;
    return true;
}

Floor *Room::getFloor() const
{
    std::lock_guard<std::mutex> lock(mMutexFloor);
    return floor;
}

void Room::setFloor(Floor *p_floor_in)
{
    std::lock_guard<std::mutex> lock(mMutexFloor);
    floor = p_floor_in;
}

std::vector<ORB_SLAM3::Passage *> Room::getPassages() const
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return doorways;
}

void Room::setDoorways(ORB_SLAM3::Passage *value)
{
    if (value == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mMutexMap);

    const bool alreadyPresent =
        std::any_of(doorways.begin(),
                    doorways.end(),
                    [value](ORB_SLAM3::Passage *existingPassage)
                    {
                        return existingPassage != nullptr &&
                               existingPassage->getId() == value->getId();
                    });

    if (!alreadyPresent)
    {
        doorways.push_back(value);
    }
}

bool Room::replacePassageAssociation(ORB_SLAM3::Passage *p_retiredPassage_in,
                                     ORB_SLAM3::Passage *p_retainedPassage_in)
{
    if (p_retiredPassage_in == nullptr || p_retainedPassage_in == nullptr ||
        p_retiredPassage_in == p_retainedPassage_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexMap);

    bool                              replacedAssociation = false;
    std::vector<ORB_SLAM3::Passage *> rebuiltPassages;
    rebuiltPassages.reserve(doorways.size());

    for (ORB_SLAM3::Passage *p_existingPassage : doorways)
    {
        ORB_SLAM3::Passage *p_candidatePassage = p_existingPassage;

        if (p_existingPassage == p_retiredPassage_in)
        {
            p_candidatePassage  = p_retainedPassage_in;
            replacedAssociation = true;
        }

        if (p_candidatePassage == nullptr ||
            std::find(rebuiltPassages.begin(),
                      rebuiltPassages.end(),
                      p_candidatePassage) != rebuiltPassages.end())
        {
            continue;
        }

        rebuiltPassages.push_back(p_candidatePassage);
    }

    if (replacedAssociation)
    {
        doorways.swap(rebuiltPassages);
    }

    return replacedAssociation;
}

void Room::clearPassages()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    doorways.clear();
}

bool Room::removePassageAssociation(ORB_SLAM3::Passage *p_removedPassage_in)
{
    if (p_removedPassage_in == nullptr)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexMap);

    const auto removedIterator = std::find(doorways.begin(),
                                           doorways.end(),
                                           p_removedPassage_in);

    if (removedIterator == doorways.end())
    {
        return false;
    }

    doorways.erase(removedIterator);
    return true;
}

Eigen::Vector3d Room::getCentroid() const
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return centroid;
}

void Room::setCentroid(Eigen::Vector3d value)
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    centroid = value;
}

Map *Room::getMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void Room::setMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}
} // namespace ORB_SLAM3
