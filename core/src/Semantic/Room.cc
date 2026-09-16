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

/*!
 * @file         Room.cc
 *
 * @brief        Implements Room declared in Semantic/Room.h.
 */

#include "Semantic/Room.h"
#include <algorithm>
#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

Room::Room() = default;

Room::~Room() = default;

void Room::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    unique_lock<mutex> lock(mapMutex);

    centroid = transform_oldWorldToNewWorld_in.map(centroid);
}

int Room::getId() const
{
    return id;
}

void Room::setId(int id_in)
{
    id = id_in;
}

int Room::getOpId() const
{
    return opId;
}

void Room::setOpId(int opId_in)
{
    opId = opId_in;
}

int Room::getOpIdG() const
{
    return opIdG;
}

void Room::setOpIdG(int opIdG_in)
{
    opIdG = opIdG_in;
}

void Room::setBad()
{
    unique_lock<mutex> lock(mapMutex);
    isBadFlag = true;
}

bool Room::isBad()
{
    unique_lock<mutex> lock(mapMutex);
    return isBadFlag;
}

int Room::getMetaMarkerId() const
{
    return metaMarkerId;
}

void Room::setMetaMarkerId(int metaMarkerId_in)
{
    metaMarkerId = metaMarkerId_in;
}

Marker *Room::getMetaMarker() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return p_metaMarker;
}

void Room::setMetaMarker(Marker *p_metaMarker_in)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    p_metaMarker = p_metaMarker_in;
}

std::string Room::getName() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return name;
}

void Room::setName(std::string name_in)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    name = name_in;
}

std::string Room::getRoomTag() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return roomTag;
}

void Room::setRoomTag(const std::string &tag_in)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    roomTag = tag_in;
}

bool Room::hasRoomTag() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return !roomTag.empty();
}

void Room::setRecoveryProxy(const bool isRecoveryProxy_in)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    recoveryProxy = isRecoveryProxy_in;
}

bool Room::isRecoveryProxy() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return recoveryProxy;
}

void Room::setPreviouslyVisited(const bool visited_in)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    previouslyVisited = visited_in;
}

bool Room::hasPreviouslyVisited() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return previouslyVisited;
}

void Room::setMatchedContext(RoomContextSnapshot *p_matchedContext_in)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    p_matchedContext = p_matchedContext_in;
}

RoomContextSnapshot *Room::getMatchedContext() const
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return p_matchedContext;
}

Room::RoomVariant Room::getRoomVariant()
{
    std::lock_guard<std::mutex> lock(stateMutex);
    return variant;
}

void Room::setRoomVariant(Room::RoomVariant variant_in)
{
    std::lock_guard<std::mutex> lock(stateMutex);
    variant = variant_in;
}

Room::BoundaryStatus Room::getBoundaryStatus() const
{
    std::lock_guard<std::mutex> boundaryStatusLock(boundaryStatusMutex);
    return boundaryStatus;
}

void Room::setBoundaryStatus(const BoundaryStatus boundaryStatus_in)
{
    std::lock_guard<std::mutex> boundaryStatusLock(boundaryStatusMutex);
    boundaryStatus = boundaryStatus_in;
}

std::vector<Eigen::Vector3d> Room::getBoundaryCorners_World_m() const
{
    std::lock_guard<std::mutex> boundaryStatusLock(boundaryStatusMutex);
    return boundaryCorners_World_m;
}

void Room::setBoundaryCorners_World_m(
    std::vector<Eigen::Vector3d> corners_World_m_in)
{
    std::lock_guard<std::mutex> boundaryStatusLock(boundaryStatusMutex);
    boundaryCorners_World_m = std::move(corners_World_m_in);
}

std::vector<Room::ObservationGap> Room::getObservationGaps() const
{
    std::lock_guard<std::mutex> boundaryStatusLock(boundaryStatusMutex);
    return observationGaps;
}

void Room::setObservationGaps(std::vector<ObservationGap> gaps_in)
{
    std::lock_guard<std::mutex> boundaryStatusLock(boundaryStatusMutex);
    observationGaps = std::move(gaps_in);
}

bool Room::isBoundaryComplete() const
{
    return getBoundaryStatus() == BoundaryStatus::COMPLETE;
}

bool Room::getHasKnownLabel() const
{
    return hasKnownLabel;
}

void Room::setHasKnownLabel(bool hasKnownLabel_in)
{
    hasKnownLabel = hasKnownLabel_in;
}

std::vector<geometric::Plane *> Room::getWalls() const
{
    std::lock_guard<std::mutex> lock(wallsMutex);
    return walls;
}

std::optional<Eigen::Vector3d>
    Room::getWallNormalTowardRoom_World(const geometric::Plane *p_wall_in) const
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
        std::lock_guard<std::mutex> lock(mapMutex);
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

void Room::setWalls(geometric::Plane *p_wall_in)
{
    /* Confirm that input wall is valid */
    if (p_wall_in == nullptr)
    {
        return;
    }

    setRecoveryProxy(false);

    std::lock_guard<std::mutex> lock(wallsMutex);

    /* Deduplicate membership within this room; the manager owns global policy.
     */
    const bool alreadyAssociated =
        std::any_of(walls.begin(),
                    walls.end(),
                    [p_wall_in](const geometric::Plane *p_existingWall)
                    { return p_existingWall == p_wall_in; });

    if (!alreadyAssociated)
    {
        walls.push_back(p_wall_in);
    }
}

bool Room::replaceWall(geometric::Plane *p_retiredWall_in, geometric::Plane *p_retainedWall_in)
{
    if (p_retiredWall_in == nullptr || p_retainedWall_in == nullptr ||
        p_retiredWall_in == p_retainedWall_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(wallsMutex);

    bool                 replacedRetiredWall = false;
    std::vector<geometric::Plane *> rebuiltWalls;
    rebuiltWalls.reserve(walls.size());

    for (geometric::Plane *p_existingWall : walls)
    {
        geometric::Plane *p_candidateWall = p_existingWall;

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

bool Room::removeWall(geometric::Plane *p_wall_in)
{
    if (p_wall_in == nullptr)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(wallsMutex);

    const auto wallIterator =
        std::remove(walls.begin(), walls.end(), p_wall_in);
    const bool removedWall = wallIterator != walls.end();
    walls.erase(wallIterator, walls.end());

    return removedWall;
}

void Room::clearWalls()
{
    std::lock_guard<std::mutex> lock(wallsMutex);
    walls.clear();
}

std::size_t Room::removeInvalidWalls()
{
    std::lock_guard<std::mutex> lock(wallsMutex);

    walls.erase(std::remove_if(walls.begin(),
                               walls.end(),
                               [](geometric::Plane *p_wall) {
                                   return p_wall == nullptr || p_wall->isBad();
                               }),
                walls.end());

    return walls.size();
}

geometric::Plane *Room::getGroundPlane() const
{
    std::lock_guard<std::mutex> lock(wallsMutex);
    return p_groundPlane;
}

void Room::setGroundPlane(geometric::Plane *p_groundPlane_in)
{
    std::lock_guard<std::mutex> lock(wallsMutex);
    p_groundPlane = p_groundPlane_in;
}

bool Room::replaceGroundPlane(geometric::Plane *p_retiredGround_in,
                              geometric::Plane *p_retainedGround_in)
{
    if (p_retiredGround_in == nullptr || p_retainedGround_in == nullptr ||
        p_retiredGround_in == p_retainedGround_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(wallsMutex);

    if (p_groundPlane != p_retiredGround_in)
    {
        return false;
    }

    p_groundPlane = p_retainedGround_in;
    return true;
}

Floor *Room::getFloor() const
{
    std::lock_guard<std::mutex> lock(floorMutex);
    return p_floor;
}

void Room::setFloor(Floor *p_floor_in)
{
    std::lock_guard<std::mutex> lock(floorMutex);
    p_floor = p_floor_in;
}

std::vector<vs_graphs::core::semantic::Passage *> Room::getPassages() const
{
    std::lock_guard<std::mutex> lock(mapMutex);
    return doorways;
}

void Room::setDoorways(vs_graphs::core::semantic::Passage *p_passage_in)
{
    if (p_passage_in == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mapMutex);

    const bool alreadyPresent =
        std::any_of(doorways.begin(),
                    doorways.end(),
                    [p_passage_in](
                        vs_graphs::core::semantic::Passage *existingPassage)
                    {
                        return existingPassage != nullptr &&
                               existingPassage->getId() ==
                                   p_passage_in->getId();
                    });

    if (!alreadyPresent)
    {
        doorways.push_back(p_passage_in);
    }
}

bool Room::replacePassageAssociation(vs_graphs::core::semantic::Passage *p_retiredPassage_in,
                                     vs_graphs::core::semantic::Passage *p_retainedPassage_in)
{
    if (p_retiredPassage_in == nullptr || p_retainedPassage_in == nullptr ||
        p_retiredPassage_in == p_retainedPassage_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mapMutex);

    bool                              replacedAssociation = false;
    std::vector<vs_graphs::core::semantic::Passage *> rebuiltPassages;
    rebuiltPassages.reserve(doorways.size());

    for (vs_graphs::core::semantic::Passage *p_existingPassage : doorways)
    {
        vs_graphs::core::semantic::Passage *p_candidatePassage = p_existingPassage;

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
    std::lock_guard<std::mutex> lock(mapMutex);
    doorways.clear();
}

bool Room::removePassageAssociation(vs_graphs::core::semantic::Passage *p_removedPassage_in)
{
    if (p_removedPassage_in == nullptr)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mapMutex);

    const auto removedIterator =
        std::find(doorways.begin(), doorways.end(), p_removedPassage_in);

    if (removedIterator == doorways.end())
    {
        return false;
    }

    doorways.erase(removedIterator);
    return true;
}

Eigen::Vector3d Room::getCentroid() const
{
    std::lock_guard<std::mutex> lock(mapMutex);
    return centroid;
}

void Room::setCentroid(Eigen::Vector3d centroid_in)
{
    std::lock_guard<std::mutex> lock(mapMutex);
    centroid = centroid_in;
}

core::Map *Room::getMap()
{
    unique_lock<mutex> lock(mapMutex);
    return p_map;
}

void Room::setMap(core::Map *p_map_in)
{
    unique_lock<mutex> lock(mapMutex);
    p_map = p_map_in;
}
} // namespace semantic
} // namespace core
} // namespace vs_graphs
