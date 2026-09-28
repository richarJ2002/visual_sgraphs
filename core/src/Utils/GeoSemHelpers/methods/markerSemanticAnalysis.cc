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
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

#include "GeoSemHelpers.h"

namespace vs_graphs
{
namespace core
{

void GeoSemHelpers::markerSemanticAnalysis(
    Atlas                                         *p_atlas_in,
    vs_graphs::core::KeyFrame                     *p_keyFrame_in,
    std::vector<vs_graphs::core::semantic::Room *> envRooms_in)
{
    // Get the markers from the current KeyFrame
    std::vector<semantic::Marker *> mapMarkers =
        p_keyFrame_in->getCurrentFrameMarkers();

    for (semantic::Marker *p_currentMarker : mapMarkers)
    {
        // Variables
        vs_graphs::core::semantic::Marker *p_currentMapMarker;

        // Check the type of the marker
        std::pair<bool, std::string> result =
            checkIfMarkerIsDoorway(p_currentMarker->getId(), envRooms_in);
        bool        markerIsDoorway = result.first;
        std::string doorwayName     = result.second;

        // Change the marker type
        p_currentMarker->setMarkerType(
            markerIsDoorway
                ? vs_graphs::core::semantic::Marker::MarkerVariant::ON_DOOR
                : vs_graphs::core::semantic::Marker::MarkerVariant::
                      ON_ROOM_CENTER);

        // If the marker is not in the map, add it
        if (!p_currentMarker->isMarkerInGMap())
        {
            p_currentMarker->setMap(p_atlas_in->getCurrentMap());
            p_currentMarker->setGlobalPose(p_keyFrame_in->getPoseInverse() *
                                           p_currentMarker->getLocalPose());
            p_currentMarker->setMarkerInGMap(true);

            // Creating a new marker in the map
            p_currentMapMarker =
                createMapMarker(p_atlas_in, p_keyFrame_in, p_currentMarker);
        }
        // Else, add the observation to the existing marker
        else
            for (auto p_mappedMarker : p_atlas_in->getAllMarkers())
                if (p_mappedMarker->getId() == p_currentMarker->getId())
                {
                    p_currentMapMarker = p_mappedMarker;
                    p_currentMapMarker->addObservation(
                        p_keyFrame_in,
                        p_currentMarker->getLocalPose());
                }
    }
}

} // namespace core
} // namespace vs_graphs
