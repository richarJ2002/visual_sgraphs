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

semantic::Marker *
    GeoSemHelpers::createMapMarker(Atlas                     *p_atlas_inout,
                                   vs_graphs::core::KeyFrame *p_keyFrame_inout,
                                   const semantic::Marker *p_visitedMarker_in)
{
    vs_graphs::core::semantic::Marker *p_newMapMarker =
        new vs_graphs::core::semantic::Marker();

    p_newMapMarker->setId(p_visitedMarker_in->getId());
    p_newMapMarker->setMap(p_atlas_inout->getCurrentMap());
    p_newMapMarker->setOpId(p_visitedMarker_in->getOpId());
    p_newMapMarker->setTime(p_visitedMarker_in->getTime());
    p_newMapMarker->setLocalPose(p_visitedMarker_in->getLocalPose());
    p_newMapMarker->setGlobalPose(p_visitedMarker_in->getGlobalPose());
    p_newMapMarker->setMarkerType(p_visitedMarker_in->getMarkerType());
    p_newMapMarker->setMarkerInGMap(p_visitedMarker_in->isMarkerInGMap());
    p_newMapMarker->addObservation(p_keyFrame_inout,
                                   p_visitedMarker_in->getLocalPose());

    p_keyFrame_inout->addMapMarker(p_newMapMarker);
    p_atlas_inout->addMapMarker(p_newMapMarker);

    return p_newMapMarker;
}

} // namespace core
} // namespace vs_graphs
