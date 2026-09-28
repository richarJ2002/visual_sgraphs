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

#include <iostream>

namespace vs_graphs
{
namespace core
{

GeoSemHelpersStatus
    GeoSemHelpers::createMapFloor(vs_graphs::core::Atlas *p_atlas_inout,
                                  std::optional<int>      stableFloorId_in)
{
    vs_graphs::core::Map *p_currentMap = p_atlas_inout->getCurrentMap();

    if (p_currentMap == nullptr)
    {
        return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
    }

    // Create a new floor object
    Eigen::Vector3d                   centroid = Eigen::Vector3d::Zero();
    vs_graphs::core::semantic::Floor *p_newMapFloor =
        new vs_graphs::core::semantic::Floor();

    // Variables
    const int floorId = stableFloorId_in.has_value()
                            ? *stableFloorId_in
                            : p_atlas_inout->reserveFloorIdentity();
    p_atlas_inout->observeFloorIdentity(floorId);

    // Fill the floor entity
    if (p_newMapFloor->setOpId(-1) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // setOpId cannot fail; continue as before.
    }
    if (p_newMapFloor->setOpIdG(-1) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // setOpIdG cannot fail; continue as before.
    }
    if (p_newMapFloor->setId(floorId) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // setId cannot fail; continue as before.
    }
    if (p_newMapFloor->setCentroid(centroid) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // setCentroid cannot fail; continue as before.
    }
    if (p_newMapFloor->setMap(p_currentMap) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // setMap cannot fail; continue as before.
    }
    if (p_newMapFloor->setName("semantic::Floor#" + std::to_string(floorId)) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // setName cannot fail; continue as before.
    }

    // Add the floor to the map
    p_atlas_inout->addMapFloor(p_newMapFloor);

    int newMapFloorId{};
    if (p_newMapFloor->getId(newMapFloorId) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    std::cout << "[GeoSemHelper] Creating semantic::Floor#" << newMapFloorId
              << " ..." << std::endl;

    return GeoSemHelpersStatus::GEO_SEM_HELPERS_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
