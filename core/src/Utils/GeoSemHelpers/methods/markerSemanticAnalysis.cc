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
    Atlas                                         *p_atlas_inout,
    vs_graphs::core::KeyFrame                     *pKF,
    std::vector<vs_graphs::core::semantic::Room *> envRooms)
{
    // Get the markers from the current KeyFrame
    std::vector<semantic::Marker *> mapMarkers = pKF->getCurrentFrameMarkers();

    for (semantic::Marker *p_currentMarker : mapMarkers)
    {
        // Variables
        vs_graphs::core::semantic::Marker *currentMapMarker;

        // Check the type of the marker
        std::pair<bool, std::string> result =
            checkIfMarkerIsDoorway(p_currentMarker->getId(), envRooms);
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
            p_currentMarker->setMap(p_atlas_inout->getCurrentMap());
            p_currentMarker->setGlobalPose(pKF->getPoseInverse() *
                                           p_currentMarker->getLocalPose());
            p_currentMarker->setMarkerInGMap(true);

            // Creating a new marker in the map
            currentMapMarker =
                createMapMarker(p_atlas_inout, pKF, p_currentMarker);
        }
        // Else, add the observation to the existing marker
        else
            for (auto mappedMarker : p_atlas_inout->getAllMarkers())
                if (mappedMarker->getId() == p_currentMarker->getId())
                {
                    currentMapMarker = mappedMarker;
                    currentMapMarker->addObservation(
                        pKF,
                        p_currentMarker->getLocalPose());
                }
    }
}

} // namespace core
} // namespace vs_graphs
