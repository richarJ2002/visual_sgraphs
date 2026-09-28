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

GeoSemHelpersStatus GeoSemHelpers::markerSemanticAnalysis(
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
        int currentMarkerId{};
        if (p_currentMarker->getId(currentMarkerId) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // getId cannot fail; continue as before.
        }
        std::pair<bool, std::string> result{};
        if (checkIfMarkerIsDoorway(currentMarkerId, envRooms_in, result) !=
            GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            // checkIfMarkerIsDoorway cannot fail; continue as before.
        }
        bool        markerIsDoorway = result.first;
        std::string doorwayName     = result.second;

        // Change the marker type
        if (p_currentMarker->setMarkerType(
                markerIsDoorway
                    ? vs_graphs::core::semantic::Marker::MarkerVariant::ON_DOOR
                    : vs_graphs::core::semantic::Marker::MarkerVariant::
                          ON_ROOM_CENTER) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // setMarkerType cannot fail; continue as before.
        }

        // If the marker is not in the map, add it
        bool currentMarkerIsMarkerInGMap{};
        if (p_currentMarker->isMarkerInGMap(currentMarkerIsMarkerInGMap) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            // isMarkerInGMap cannot fail; continue as before.
        }
        if (!currentMarkerIsMarkerInGMap)
        {
            if (p_currentMarker->setMap(p_atlas_in->getCurrentMap()) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // setMap cannot fail; continue as before.
            }
            Sophus::SE3f currentMarkerLocalPose{};
            if (p_currentMarker->getLocalPose(currentMarkerLocalPose) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // getLocalPose cannot fail; continue as before.
            }
            if (p_currentMarker->setGlobalPose(p_keyFrame_in->getPoseInverse() *
                                               currentMarkerLocalPose) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // setGlobalPose cannot fail; continue as before.
            }
            if (p_currentMarker->setMarkerInGMap(true) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                // setMarkerInGMap cannot fail; continue as before.
            }

            // Creating a new marker in the map
            semantic::Marker *p_mapMarker = nullptr;
            if (createMapMarker(p_atlas_in,
                                p_keyFrame_in,
                                p_currentMarker,
                                p_mapMarker) !=
                GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
            {
                // createMapMarker cannot fail; continue as before.
            }
            p_currentMapMarker = p_mapMarker;
        }
        // Else, add the observation to the existing marker
        else
        {
            for (auto p_mappedMarker : p_atlas_in->getAllMarkers())
            {
                int mappedMarkerId{};
                if (p_mappedMarker->getId(mappedMarkerId) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                int currentMarkerId2{};
                if (p_currentMarker->getId(currentMarkerId2) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    // getId cannot fail; continue as before.
                }
                if (mappedMarkerId == currentMarkerId2)
                {
                    p_currentMapMarker = p_mappedMarker;
                    Sophus::SE3f currentMarkerLocalPose2{};
                    if (p_currentMarker->getLocalPose(
                            currentMarkerLocalPose2) !=
                        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                    {
                        // getLocalPose cannot fail; continue as before.
                    }
                    if (p_currentMapMarker->addObservation(
                            p_keyFrame_in,
                            currentMarkerLocalPose2) !=
                        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                    {
                        // addObservation cannot fail; continue as before.
                    }
                }
            }
        }
    }

    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
