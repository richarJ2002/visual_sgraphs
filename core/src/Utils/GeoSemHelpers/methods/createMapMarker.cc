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

    int visitedMarker_inId{};
    if (p_visitedMarker_in->getId(visitedMarker_inId) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    if (p_newMapMarker->setId(visitedMarker_inId) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // setId cannot fail; continue as before.
    }
    if (p_newMapMarker->setMap(p_atlas_inout->getCurrentMap()) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // setMap cannot fail; continue as before.
    }
    int visitedMarker_inOpId{};
    if (p_visitedMarker_in->getOpId(visitedMarker_inOpId) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // getOpId cannot fail; continue as before.
    }
    if (p_newMapMarker->setOpId(visitedMarker_inOpId) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // setOpId cannot fail; continue as before.
    }
    double visitedMarker_inTime{};
    if (p_visitedMarker_in->getTime(visitedMarker_inTime) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // getTime cannot fail; continue as before.
    }
    if (p_newMapMarker->setTime(visitedMarker_inTime) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // setTime cannot fail; continue as before.
    }
    Sophus::SE3f visitedMarker_inLocalPose{};
    if (p_visitedMarker_in->getLocalPose(visitedMarker_inLocalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // getLocalPose cannot fail; continue as before.
    }
    if (p_newMapMarker->setLocalPose(visitedMarker_inLocalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // setLocalPose cannot fail; continue as before.
    }
    Sophus::SE3f visitedMarker_inGlobalPose{};
    if (p_visitedMarker_in->getGlobalPose(visitedMarker_inGlobalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // getGlobalPose cannot fail; continue as before.
    }
    if (p_newMapMarker->setGlobalPose(visitedMarker_inGlobalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // setGlobalPose cannot fail; continue as before.
    }
    semantic::Marker::MarkerVariant visitedMarker_inMarkerType{};
    if (p_visitedMarker_in->getMarkerType(visitedMarker_inMarkerType) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // getMarkerType cannot fail; continue as before.
    }
    if (p_newMapMarker->setMarkerType(visitedMarker_inMarkerType) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // setMarkerType cannot fail; continue as before.
    }
    bool visitedMarker_inIsMarkerInGMap{};
    if (p_visitedMarker_in->isMarkerInGMap(visitedMarker_inIsMarkerInGMap) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // isMarkerInGMap cannot fail; continue as before.
    }
    if (p_newMapMarker->setMarkerInGMap(visitedMarker_inIsMarkerInGMap) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // setMarkerInGMap cannot fail; continue as before.
    }
    Sophus::SE3f visitedMarker_inLocalPose2{};
    if (p_visitedMarker_in->getLocalPose(visitedMarker_inLocalPose2) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // getLocalPose cannot fail; continue as before.
    }
    if (p_newMapMarker->addObservation(p_keyFrame_inout,
                                       visitedMarker_inLocalPose2) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        // addObservation cannot fail; continue as before.
    }

    p_keyFrame_inout->addMapMarker(p_newMapMarker);
    p_atlas_inout->addMapMarker(p_newMapMarker);

    return p_newMapMarker;
}

} // namespace core
} // namespace vs_graphs
