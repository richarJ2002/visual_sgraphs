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

/*!
 * @file            createMapMarker.cc
 *
 * @brief           Implements GeoSemHelpers::createMapMarker(), declared in
 *                  GeoSemHelpers.h.
 */

#include "scene_understanding/geo_sem_helpers/geo_sem_helpers.h"
#include "map_state/atlas/atlas.h"
#include "map_state/atlas/objects/AtlasStatusEnum.h"
#include "map_state/keyframe/keyframe.h"
#include "map_state/keyframe/objects/KeyFrameStatusEnum.h"
#include "map_state/map/map.h"
#include "scene_understanding/semantic/marker/marker.h"
#include "scene_understanding/semantic/marker/objects/MarkerStatusEnum.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs::core::utils::geo_sem_helpers
{

GeoSemHelpersStatus
    GeoSemHelpers::createMapMarker(atlas::Atlas                     *p_atlas_inout,
                                   keyframe::KeyFrame *p_keyFrame_inout,
                                   const semantic::Marker *p_visitedMarker_in,
                                   semantic::Marker      *&p_mapMarker_out)
{
    vs_graphs::core::semantic::Marker *p_newMapMarker =
        new vs_graphs::core::semantic::Marker();

    int visitedMarker_inId{};
    if (p_visitedMarker_in->getId(visitedMarker_inId) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->setId(visitedMarker_inId) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    map::Map *p_atlasCurrentMap = nullptr;
    if (p_atlas_inout->getCurrentMap(p_atlasCurrentMap) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->setMap(p_atlasCurrentMap) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    int visitedMarker_inOpId{};
    if (p_visitedMarker_in->getOpId(visitedMarker_inOpId) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getOpId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->setOpId(visitedMarker_inOpId) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setOpId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    double visitedMarker_inTime{};
    if (p_visitedMarker_in->getTime(visitedMarker_inTime) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTime returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->setTime(visitedMarker_inTime) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setTime returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f visitedMarker_inLocalPose{};
    if (p_visitedMarker_in->getLocalPose(visitedMarker_inLocalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLocalPose returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->setLocalPose(visitedMarker_inLocalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setLocalPose returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f visitedMarker_inGlobalPose{};
    if (p_visitedMarker_in->getGlobalPose(visitedMarker_inGlobalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalPose returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->setGlobalPose(visitedMarker_inGlobalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setGlobalPose returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Marker::MarkerVariant visitedMarker_inMarkerType{};
    if (p_visitedMarker_in->getMarkerType(visitedMarker_inMarkerType) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMarkerType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->setMarkerType(visitedMarker_inMarkerType) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMarkerType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    bool visitedMarker_inIsMarkerInGMap{};
    if (p_visitedMarker_in->isMarkerInGMap(visitedMarker_inIsMarkerInGMap) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isMarkerInGMap returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->setMarkerInGMap(visitedMarker_inIsMarkerInGMap) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setMarkerInGMap returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Sophus::SE3f visitedMarker_inLocalPose2{};
    if (p_visitedMarker_in->getLocalPose(visitedMarker_inLocalPose2) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getLocalPose returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_newMapMarker->addObservation(p_keyFrame_inout,
                                       visitedMarker_inLocalPose2) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addObservation returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    if (p_keyFrame_inout->addMapMarker(p_newMapMarker) !=
        keyframe::KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addMapMarker returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (p_atlas_inout->addMapMarker(p_newMapMarker) !=
        atlas::AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: addMapMarker returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    p_mapMarker_out = p_newMapMarker;
    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} /* namespace vs_graphs::core::utils::geo_sem_helpers */
