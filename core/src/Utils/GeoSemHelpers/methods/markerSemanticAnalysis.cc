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
#include <rclcpp/logging.hpp>

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
    std::vector<semantic::Marker *> mapMarkers{};
    if (p_keyFrame_in->getCurrentFrameMarkers(mapMarkers) !=
        KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentFrameMarkers returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    for (semantic::Marker *p_currentMarker : mapMarkers)
    {
        // Variables
        vs_graphs::core::semantic::Marker *p_currentMapMarker;

        // Check the type of the marker
        int currentMarkerId{};
        if (p_currentMarker->getId(currentMarkerId) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::pair<bool, std::string> result{};
        if (checkIfMarkerIsDoorway(currentMarkerId, envRooms_in, result) !=
            GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: checkIfMarkerIsDoorway returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
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
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setMarkerType returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }

        // If the marker is not in the map, add it
        bool currentMarkerIsMarkerInGMap{};
        if (p_currentMarker->isMarkerInGMap(currentMarkerIsMarkerInGMap) !=
            semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isMarkerInGMap returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        if (!currentMarkerIsMarkerInGMap)
        {
            if (p_currentMarker->setMap(p_atlas_in->getCurrentMap()) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            Sophus::SE3f currentMarkerLocalPose{};
            if (p_currentMarker->getLocalPose(currentMarkerLocalPose) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getLocalPose returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Sophus::SE3f keyFramePoseInverse{};
            if (p_keyFrame_in->getPoseInverse(keyFramePoseInverse) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getPoseInverse returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMarker->setGlobalPose(keyFramePoseInverse *
                                               currentMarkerLocalPose) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setGlobalPose returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentMarker->setMarkerInGMap(true) !=
                semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setMarkerInGMap returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }

            // Creating a new marker in the map
            semantic::Marker *p_mapMarker = nullptr;
            if (createMapMarker(p_atlas_in,
                                p_keyFrame_in,
                                p_currentMarker,
                                p_mapMarker) !=
                GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: createMapMarker returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
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
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                int currentMarkerId2{};
                if (p_currentMarker->getId(currentMarkerId2) !=
                    semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                if (mappedMarkerId == currentMarkerId2)
                {
                    p_currentMapMarker = p_mappedMarker;
                    Sophus::SE3f currentMarkerLocalPose2{};
                    if (p_currentMarker->getLocalPose(
                            currentMarkerLocalPose2) !=
                        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: getLocalPose returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_currentMapMarker->addObservation(
                            p_keyFrame_in,
                            currentMarkerLocalPose2) !=
                        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: addObservation returned a failure status "
                            "although it cannot fail; continuing as before.",
                            __func__);
                    }
                }
            }
        }
    }

    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
