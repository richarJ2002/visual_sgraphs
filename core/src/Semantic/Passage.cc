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

#include "Semantic/Passage.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ORB_SLAM3
{

Passage::Passage() :
    id(-1),
    opId(-1),
    opIdG(-1),
    width(0.0),
    height(0.0),
    passable(false),
    centroid(Eigen::Vector3d::Zero()),
    passageType(Passage::passageVariant::UNDEFINED),
    associateDoor(nullptr),
    prospectiveRoom(nullptr),
    traversalKnownToFarCount(0U),
    traversalFarToKnownCount(0U),
    traversalUnknownCount(0U),
    mpMap(nullptr)
{}

Passage::~Passage() {}

void Passage::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);

    centroid = transform_oldWorldToNewWorld_in.map(centroid);

    Eigen::Vector4d passageEquation = globalEquation.coeffs();
    const double    normalNorm      = passageEquation.head<3>().norm();

    if (std::isfinite(normalNorm) && normalNorm > 1e-8)
    {
        passageEquation /= normalNorm;

        const Eigen::Vector3d transformedNormal =
            transform_oldWorldToNewWorld_in.rotation().toRotationMatrix() *
            passageEquation.head<3>();

        const Eigen::Vector3d translation =
            transform_oldWorldToNewWorld_in.translation();

        const double transformedDistance =
            transform_oldWorldToNewWorld_in.scale() * passageEquation(3) -
            transformedNormal.dot(translation);

        globalEquation = g2o::Plane3D(Eigen::Vector4d(transformedNormal.x(),
                                                      transformedNormal.y(),
                                                      transformedNormal.z(),
                                                      transformedDistance));
    }

    if (knownSideProvenance.hasDirection())
    {
        const Eigen::Vector3d transformedDirection =
            transform_oldWorldToNewWorld_in.rotation().toRotationMatrix() *
            knownSideProvenance.direction_World;
        if (transformedDirection.allFinite() &&
            transformedDirection.norm() > 1e-8)
        {
            knownSideProvenance.direction_World =
                transformedDirection.normalized();
        }
    }

    const double absoluteScale =
        std::abs(transform_oldWorldToNewWorld_in.scale());

    width *= absoluteScale;
    height *= absoluteScale;
}

int Passage::getId() const
{
    return id;
}

void Passage::setId(int value)
{
    id = value;
}

int Passage::getOpId() const
{
    return opId;
}

void Passage::setOpId(int value)
{
    opId = value;
}

int Passage::getOpIdG() const
{
    return opIdG;
}

void Passage::setOpIdG(int value)
{
    opIdG = value;
}

bool Passage::isPassable() const
{
    std::lock_guard<std::mutex> lock(mMutexType);
    return passable;
}

void Passage::setPassable(bool value)
{
    std::lock_guard<std::mutex> lock(mMutexType);
    passable = value;
}

bool Passage::isBad()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    return mbBad;
}

void Passage::setBad()
{
    std::lock_guard<std::mutex> lock(mMutexMap);
    mbBad = true;
}

void Passage::setRecoveryProxy(const bool isRecoveryProxy_in)
{
    std::lock_guard<std::mutex> lock(mMutexType);
    recoveryProxy = isRecoveryProxy_in;
}

bool Passage::isRecoveryProxy() const
{
    std::lock_guard<std::mutex> lock(mMutexType);
    return recoveryProxy;
}

bool Passage::getTraversalEvidence() const
{
    std::lock_guard<std::mutex> lock(mMutexType);
    return traversalKnownToFarCount > 0U || traversalFarToKnownCount > 0U ||
           traversalUnknownCount > 0U;
}

void Passage::setTraversalEvidence(bool value)
{
    std::lock_guard<std::mutex> lock(mMutexType);
    if (value)
    {
        if (traversalKnownToFarCount == 0U && traversalFarToKnownCount == 0U &&
            traversalUnknownCount == 0U)
        {
            traversalUnknownCount = 1U;
        }
    }
    else
    {
        traversalKnownToFarCount = 0U;
        traversalFarToKnownCount = 0U;
        traversalUnknownCount    = 0U;
        traversalSegmentHistory.clear();
    }
}

std::size_t Passage::getTraversalObservationCount() const
{
    std::lock_guard<std::mutex> lock(mMutexType);
    return traversalKnownToFarCount + traversalFarToKnownCount +
           traversalUnknownCount;
}

void Passage::addTraversalObservation()
{
    addTraversalObservation(TraversalDirection::UNKNOWN);
}

void Passage::addTraversalObservation(TraversalDirection direction_in)
{
    std::lock_guard<std::mutex> lock(mMutexType);

    std::size_t *p_counter = &traversalUnknownCount;
    if (direction_in == TraversalDirection::KNOWN_TO_FAR)
    {
        p_counter = &traversalKnownToFarCount;
    }
    else if (direction_in == TraversalDirection::FAR_TO_KNOWN)
    {
        p_counter = &traversalFarToKnownCount;
    }

    if (*p_counter < std::numeric_limits<std::size_t>::max())
    {
        ++(*p_counter);
    }
}

bool Passage::addTraversalObservation(TraversalDirection direction_in,
                                      unsigned long      frameId_in,
                                      unsigned long      keyFrameId_in)
{
    std::lock_guard<std::mutex>                   lock(mMutexType);
    const std::pair<unsigned long, unsigned long> segmentId(frameId_in,
                                                            keyFrameId_in);
    if (std::find(traversalSegmentHistory.begin(),
                  traversalSegmentHistory.end(),
                  segmentId) != traversalSegmentHistory.end())
    {
        return false;
    }

    constexpr std::size_t maximumTraversalSegmentHistory = 128U;
    if (traversalSegmentHistory.size() >= maximumTraversalSegmentHistory)
    {
        traversalSegmentHistory.pop_front();
    }
    traversalSegmentHistory.push_back(segmentId);

    std::size_t *p_counter = &traversalUnknownCount;
    if (direction_in == TraversalDirection::KNOWN_TO_FAR)
    {
        p_counter = &traversalKnownToFarCount;
    }
    else if (direction_in == TraversalDirection::FAR_TO_KNOWN)
    {
        p_counter = &traversalFarToKnownCount;
    }
    if (*p_counter < std::numeric_limits<std::size_t>::max())
    {
        ++(*p_counter);
    }
    return true;
}

std::size_t Passage::getTraversalKnownToFarCount() const
{
    std::lock_guard<std::mutex> lock(mMutexType);
    return traversalKnownToFarCount;
}

std::size_t Passage::getTraversalFarToKnownCount() const
{
    std::lock_guard<std::mutex> lock(mMutexType);
    return traversalFarToKnownCount;
}

std::size_t Passage::getTraversalUnknownCount() const
{
    std::lock_guard<std::mutex> lock(mMutexType);
    return traversalUnknownCount;
}

bool Passage::hasBidirectionalTraversalEvidence() const
{
    std::lock_guard<std::mutex> lock(mMutexType);
    return traversalKnownToFarCount > 0U && traversalFarToKnownCount > 0U;
}

void Passage::setTraversalObservationCount(std::size_t value)
{
    std::lock_guard<std::mutex> lock(mMutexType);
    const std::size_t           directionalCount =
        traversalKnownToFarCount + traversalFarToKnownCount;
    traversalUnknownCount =
        value > directionalCount ? value - directionalCount : 0U;
}

double Passage::getWidth() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return width;
}

void Passage::setWidth(double value)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    width = value;
}

double Passage::getHeight() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return height;
}

void Passage::setHeight(double value)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    height = value;
}

Passage::passageVariant Passage::getPassageType()
{
    unique_lock<mutex> lock(mMutexType);
    return passageType;
}

void Passage::setPassageType(Passage::passageVariant newType)
{
    unique_lock<mutex> lock(mMutexType);
    passageType = newType;
}

Eigen::Vector3d Passage::getCentroid() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return centroid;
}

void Passage::setCentroid(const Eigen::Vector3d &value)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    centroid = value;
}

ORB_SLAM3::Plane *Passage::getAssociateDoor() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return associateDoor;
}

void Passage::setAssociateDoor(ORB_SLAM3::Plane *value)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    associateDoor = value;
}

g2o::Plane3D Passage::getGlobalEquation() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return globalEquation;
}

void Passage::setGlobalEquation(const g2o::Plane3D &value)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    globalEquation = value;
}

std::vector<ORB_SLAM3::Plane *> Passage::getAssociateWalls() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return associateWalls;
}

void Passage::addAssociateWall(ORB_SLAM3::Plane *p_wall_in)
{
    if (p_wall_in == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mMutexGeometry);

    const bool alreadyPresent =
        std::any_of(associateWalls.begin(),
                    associateWalls.end(),
                    [p_wall_in](ORB_SLAM3::Plane *p_existingWall)
                    { return p_existingWall == p_wall_in; });

    if (!alreadyPresent)
    {
        associateWalls.push_back(p_wall_in);
    }
}

bool Passage::replacePlaneAssociation(Plane *p_retiredPlane_in,
                                      Plane *p_retainedPlane_in)
{
    if (p_retiredPlane_in == nullptr || p_retainedPlane_in == nullptr ||
        p_retiredPlane_in == p_retainedPlane_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexGeometry);

    bool replacedAssociation = false;

    if (associateDoor == p_retiredPlane_in)
    {
        associateDoor       = p_retainedPlane_in;
        replacedAssociation = true;
    }

    std::vector<Plane *> rebuiltWalls;
    rebuiltWalls.reserve(associateWalls.size());

    for (Plane *p_existingWall : associateWalls)
    {
        Plane *p_candidateWall = p_existingWall;

        if (p_existingWall == p_retiredPlane_in)
        {
            p_candidateWall     = p_retainedPlane_in;
            replacedAssociation = true;
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

    if (replacedAssociation)
    {
        associateWalls.swap(rebuiltWalls);
    }

    return replacedAssociation;
}

ORB_SLAM3::Map *Passage::getMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void Passage::setMap(ORB_SLAM3::Map *pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}

ORB_SLAM3::Room *Passage::getProspectiveRoom() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return prospectiveRoom;
}

std::optional<int> Passage::getProspectiveRoomId() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    if (prospectiveRoom == nullptr)
    {
        return std::nullopt;
    }
    return prospectiveRoom->getId();
}

void Passage::setProspectiveRoom(ORB_SLAM3::Room *p_room_in)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    prospectiveRoom = p_room_in;
}

bool Passage::hasProspectiveRoom() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return prospectiveRoom != nullptr;
}

bool Passage::replaceProspectiveRoom(ORB_SLAM3::Room *p_retiredRoom_in,
                                     ORB_SLAM3::Room *p_retainedRoom_in)
{
    if (p_retiredRoom_in == nullptr || p_retainedRoom_in == nullptr ||
        p_retiredRoom_in == p_retainedRoom_in)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexGeometry);

    bool replaced = false;
    if (prospectiveRoom == p_retiredRoom_in)
    {
        prospectiveRoom = p_retainedRoom_in;
        replaced        = true;
    }

    if (knownSideProvenance.pRoom == p_retiredRoom_in)
    {
        knownSideProvenance.pRoom = p_retainedRoom_in;
        replaced                  = true;
    }

    return replaced;
}

Passage::KnownSideProvenance Passage::getKnownSideProvenance() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return knownSideProvenance;
}

bool Passage::setKnownSideDirection(const Eigen::Vector3d &direction_World_in)
{
    const double directionNorm = direction_World_in.norm();
    if (!direction_World_in.allFinite() || !std::isfinite(directionNorm) ||
        directionNorm < 1e-8)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mMutexGeometry);
    knownSideProvenance.direction_World = direction_World_in / directionNorm;
    return true;
}

void Passage::setKnownSideRoom(ORB_SLAM3::Room *p_room_in)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    knownSideProvenance.pRoom = p_room_in;
}

void Passage::mergeKnownSideProvenance(const KnownSideProvenance &provenance_in)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    if (!knownSideProvenance.hasDirection() && provenance_in.hasDirection())
    {
        knownSideProvenance.direction_World = provenance_in.direction_World;
    }
    if (knownSideProvenance.pRoom == nullptr)
    {
        knownSideProvenance.pRoom = provenance_in.pRoom;
    }
}

bool Passage::mergeFromDuplicate(Passage *p_duplicate_in)
{
    if (p_duplicate_in == nullptr || p_duplicate_in == this ||
        p_duplicate_in->getId() != getId())
    {
        return false;
    }

    bool canonicalIsRecoveryProxy = false;
    {
        std::lock_guard<std::mutex> typeLock(mMutexType);
        canonicalIsRecoveryProxy = recoveryProxy;
    }

    bool           duplicateIsRecoveryProxy = false;
    bool           duplicateIsPassable      = false;
    passageVariant duplicatePassageType     = passageVariant::UNDEFINED;
    std::size_t    duplicateKnownToFarCount = 0U;
    std::size_t    duplicateFarToKnownCount = 0U;
    std::size_t    duplicateUnknownCount    = 0U;
    {
        std::lock_guard<std::mutex> duplicateTypeLock(
            p_duplicate_in->mMutexType);
        duplicateIsRecoveryProxy = p_duplicate_in->recoveryProxy;
        duplicateIsPassable      = p_duplicate_in->passable;
        duplicatePassageType     = p_duplicate_in->passageType;
        duplicateKnownToFarCount = p_duplicate_in->traversalKnownToFarCount;
        duplicateFarToKnownCount = p_duplicate_in->traversalFarToKnownCount;
        duplicateUnknownCount    = p_duplicate_in->traversalUnknownCount;
    }

    Eigen::Vector3d      duplicateCentroid = Eigen::Vector3d::Zero();
    g2o::Plane3D         duplicateEquation;
    double               duplicateWidth_m  = 0.0;
    double               duplicateHeight_m = 0.0;
    Plane               *p_duplicateDoor   = nullptr;
    std::vector<Plane *> duplicateWalls;
    Room                *p_duplicateProspectiveRoom = nullptr;
    KnownSideProvenance  duplicateKnownSide;
    {
        std::lock_guard<std::mutex> duplicateGeometryLock(
            p_duplicate_in->mMutexGeometry);
        duplicateCentroid          = p_duplicate_in->centroid;
        duplicateEquation          = p_duplicate_in->globalEquation;
        duplicateWidth_m           = p_duplicate_in->width;
        duplicateHeight_m          = p_duplicate_in->height;
        p_duplicateDoor            = p_duplicate_in->associateDoor;
        duplicateWalls             = p_duplicate_in->associateWalls;
        p_duplicateProspectiveRoom = p_duplicate_in->prospectiveRoom;
        duplicateKnownSide         = p_duplicate_in->knownSideProvenance;
    }

    const Eigen::Vector4d duplicateEquationCoefficients =
        duplicateEquation.coeffs();
    const double duplicateNormalNorm =
        duplicateEquationCoefficients.head<3>().norm();
    const bool duplicateHasValidObservedGeometry =
        !duplicateIsRecoveryProxy && duplicateCentroid.allFinite() &&
        duplicateEquationCoefficients.allFinite() &&
        std::isfinite(duplicateNormalNorm) && duplicateNormalNorm > 1e-8 &&
        std::isfinite(duplicateWidth_m) && duplicateWidth_m > 0.0 &&
        std::isfinite(duplicateHeight_m) && duplicateHeight_m > 0.0;

    bool replacedGeometry = false;
    {
        std::lock_guard<std::mutex> geometryLock(mMutexGeometry);
        const Eigen::Vector4d       canonicalEquationCoefficients =
            globalEquation.coeffs();
        const double canonicalNormalNorm =
            canonicalEquationCoefficients.head<3>().norm();
        const bool canonicalHasValidGeometry =
            centroid.allFinite() && canonicalEquationCoefficients.allFinite() &&
            std::isfinite(canonicalNormalNorm) && canonicalNormalNorm > 1e-8 &&
            std::isfinite(width) && width > 0.0 && std::isfinite(height) &&
            height > 0.0;

        replacedGeometry =
            duplicateHasValidObservedGeometry &&
            (canonicalIsRecoveryProxy || !canonicalHasValidGeometry);
        if (replacedGeometry)
        {
            centroid       = duplicateCentroid;
            globalEquation = duplicateEquation;
            width          = duplicateWidth_m;
            height         = duplicateHeight_m;
            associateDoor  = p_duplicateDoor;
        }
        else if (associateDoor == nullptr)
        {
            associateDoor = p_duplicateDoor;
        }

        for (Plane *p_duplicateWall : duplicateWalls)
        {
            if (p_duplicateWall != nullptr &&
                std::find(associateWalls.begin(),
                          associateWalls.end(),
                          p_duplicateWall) == associateWalls.end())
            {
                associateWalls.push_back(p_duplicateWall);
            }
        }

        if (prospectiveRoom == nullptr)
        {
            prospectiveRoom = p_duplicateProspectiveRoom;
        }
        if (knownSideProvenance.pRoom == nullptr)
        {
            knownSideProvenance.pRoom = duplicateKnownSide.pRoom;
        }
        if (replacedGeometry)
        {
            knownSideProvenance.direction_World =
                duplicateKnownSide.direction_World;
        }
        else if (!knownSideProvenance.hasDirection() &&
                 duplicateKnownSide.hasDirection())
        {
            knownSideProvenance.direction_World =
                duplicateKnownSide.direction_World;
        }
    }

    {
        std::lock_guard<std::mutex> typeLock(mMutexType);
        traversalKnownToFarCount =
            std::max(traversalKnownToFarCount, duplicateKnownToFarCount);
        traversalFarToKnownCount =
            std::max(traversalFarToKnownCount, duplicateFarToKnownCount);
        traversalUnknownCount =
            std::max(traversalUnknownCount, duplicateUnknownCount);
        if (replacedGeometry)
        {
            passable      = duplicateIsPassable;
            passageType   = duplicatePassageType;
            recoveryProxy = false;
        }
        else if (passageType == passageVariant::UNDEFINED)
        {
            passageType = duplicatePassageType;
        }
    }

    return replacedGeometry;
}
} // namespace ORB_SLAM3
