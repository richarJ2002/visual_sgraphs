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

#include "Semantic/Marker.h"

namespace ORB_SLAM3
{
Marker::Marker() :
    id(-1),
    opId(-1),
    opIdG(-1),
    time(0.0),
    markerInGMap(false),
    localPose(Sophus::SE3f()),
    globalPose(Sophus::SE3f()),
    markerType(markerVariant::UNKNOWN),
    mpMap(nullptr)
{}
Marker::~Marker() {}

void Marker::applyTransform(const g2o::Sim3 &transform_oldWorldToNewWorld_in)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);

    const Eigen::Matrix3f rotation_oldWorldToNewWorld =
        transform_oldWorldToNewWorld_in.rotation()
            .toRotationMatrix()
            .cast<float>();

    const Eigen::Matrix3f rotation_markerToNewWorld =
        rotation_oldWorldToNewWorld * globalPose.rotationMatrix();

    const Eigen::Vector3f position_newWorld_m =
        transform_oldWorldToNewWorld_in
            .map(globalPose.translation().cast<double>())
            .cast<float>();

    globalPose = Sophus::SE3f(rotation_markerToNewWorld, position_newWorld_m);
}

int Marker::getId() const
{
    return id;
}

void Marker::setId(int value)
{
    id = value;
}

int Marker::getOpId() const
{
    return opId;
}

void Marker::setOpId(int value)
{
    opId = value;
}

int Marker::getOpIdG() const
{
    return opIdG;
}

void Marker::setOpIdG(int value)
{
    opIdG = value;
}

double Marker::getTime() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return time;
}

void Marker::setTime(double value)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    time = value;
}

Marker::markerVariant Marker::getMarkerType() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return markerType;
}

void Marker::setMarkerType(Marker::markerVariant newType)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    markerType = newType;
}

bool Marker::isMarkerInGMap() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return markerInGMap;
}

void Marker::setMarkerInGMap(bool value)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    markerInGMap = value;
}

Sophus::SE3f Marker::getLocalPose() const
{
    std::lock_guard<std::mutex> lock(mMutexState);
    return localPose;
}

void Marker::setLocalPose(const Sophus::SE3f &value)
{
    std::lock_guard<std::mutex> lock(mMutexState);
    localPose = value;
}

Sophus::SE3f Marker::getGlobalPose() const
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    return globalPose;
}

void Marker::setGlobalPose(const Sophus::SE3f &value)
{
    std::lock_guard<std::mutex> lock(mMutexGeometry);
    globalPose = value;
}

std::map<KeyFrame *, Sophus::SE3f> Marker::getObservations() const
{
    std::lock_guard<std::mutex> lock(mMutexObservations);
    return observations;
}

void Marker::addObservation(KeyFrame           *p_keyFrame_in,
                            const Sophus::SE3f &markerPose_markerToCamera_in)
{
    if (p_keyFrame_in == nullptr || p_keyFrame_in->isBad())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mMutexObservations);
    observations.insert_or_assign(p_keyFrame_in, markerPose_markerToCamera_in);
}

void Marker::eraseObservation(KeyFrame *p_keyFrame_in)
{
    if (p_keyFrame_in == nullptr)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(mMutexObservations);
    observations.erase(p_keyFrame_in);
}

Map *Marker::getMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mpMap;
}

void Marker::setMap(Map *pMap)
{
    unique_lock<mutex> lock(mMutexMap);
    mpMap = pMap;
}
}; // namespace ORB_SLAM3
