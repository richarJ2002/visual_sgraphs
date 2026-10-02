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

/*!
 * @file            captureWall.cc
 *
 * @brief           Implements captureWall(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include "Map.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticGraphSnapshotStatus
    captureWall(geometric::Plane *p_wall_in,
                long unsigned int mapId_in,
                const std::map<geometric::Plane *, std::vector<EntityRef>>
                           &wallOwnersByPointer_in,
                WallRecord &wallRecord_out)
{
    WallRecord record;
    EntityKey  key2{};
    int        wallGetId{};
    if (p_wall_in->getId(wallGetId) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (makeKey(EntityKind::WALL, mapId_in, wallGetId, key2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeKey returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    record.key = key2;
    bool wallIsBad{};
    if (p_wall_in->isBad(wallIsBad) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    record.isLive = !wallIsBad;
    geometric::Plane::PlaneVariant wallPlaneType{};
    if (p_wall_in->getPlaneType(wallPlaneType) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneType returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.planeType = wallPlaneType;

    core::Map *p_declaredMap = nullptr;
    if (p_wall_in->getMap(p_declaredMap) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getMap returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (p_declaredMap != nullptr)
    {
        unsigned long declaredMapId2{};
        if (p_declaredMap->getId(declaredMapId2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        record.declaredMapId = declaredMapId2;
    }

    geometric::PlaneGeometryMetadataSnapshot geometry{};
    if (p_wall_in->getGeometryMetadataSnapshot(geometry) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGeometryMetadataSnapshot returned a failure "
                     "status although it cannot fail; continuing as before.",
                     __func__);
    }
    record.equation_world            = geometry.equation_world;
    record.centroid_world_m          = geometry.centroid_world_m;
    record.minPlaneU_m               = geometry.minPlaneU_m;
    record.maxPlaneU_m               = geometry.maxPlaneU_m;
    record.minPlaneV_m               = geometry.minPlaneV_m;
    record.maxPlaneV_m               = geometry.maxPlaneV_m;
    record.finiteSupportCount        = geometry.finiteSupportCount;
    record.observationCount          = geometry.observationCount;
    record.cloudGeneration           = geometry.cloudGeneration;
    record.successfulRefitGeneration = geometry.successfulRefitGeneration;

    std::optional<Eigen::Vector3d> wallObservationOrigin_world{};
    if (p_wall_in->getObservationOrigin_world(wallObservationOrigin_world) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getObservationOrigin_World returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    record.observationOrigin_world_m = wallObservationOrigin_world;
    RawPlaneRef       rawPlaneRef2{};
    geometric::Plane *p_wallGetTwinFace = nullptr;
    if (p_wall_in->getTwinFace(p_wallGetTwinFace) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getTwinFace returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (rawPlaneRef(p_wallGetTwinFace, rawPlaneRef2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rawPlaneRef returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    record.twinRef = rawPlaneRef2;

    const std::map<geometric::Plane *, std::vector<EntityRef>>::const_iterator
        ownerIt = wallOwnersByPointer_in.find(p_wall_in);
    if (ownerIt != wallOwnersByPointer_in.end())
    {
        record.ownerRoomRefs = ownerIt->second;
    }

    wallRecord_out = record;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
