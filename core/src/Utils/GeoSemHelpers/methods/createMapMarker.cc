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
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNSS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "GeoSemHelpers.h"

namespace vs_graphs
{
namespace core
{

semantic::Marker *GeoSemHelpers::createMapMarker(
    Atlas                                   *p_atlas_inout,
    vs_graphs::core::KeyFrame               *pKF,
    const semantic::Marker *visitedMarker)
{
    vs_graphs::core::semantic::Marker *newMapMarker =
        new vs_graphs::core::semantic::Marker();

    newMapMarker->setId(visitedMarker->getId());
    newMapMarker->setMap(p_atlas_inout->getCurrentMap());
    newMapMarker->setOpId(visitedMarker->getOpId());
    newMapMarker->setTime(visitedMarker->getTime());
    newMapMarker->setLocalPose(visitedMarker->getLocalPose());
    newMapMarker->setGlobalPose(visitedMarker->getGlobalPose());
    newMapMarker->setMarkerType(visitedMarker->getMarkerType());
    newMapMarker->setMarkerInGMap(visitedMarker->isMarkerInGMap());
    newMapMarker->addObservation(pKF, visitedMarker->getLocalPose());

    pKF->addMapMarker(newMapMarker);
    p_atlas_inout->addMapMarker(newMapMarker);

    return newMapMarker;
}

} // namespace core
} // namespace vs_graphs
