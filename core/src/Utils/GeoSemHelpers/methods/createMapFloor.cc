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

void GeoSemHelpers::createMapFloor(vs_graphs::core::Atlas *p_atlas_inout,
                                   std::optional<int>      stableFloorId_in)
{
    vs_graphs::core::Map *p_currentMap = p_atlas_inout->getCurrentMap();

    if (p_currentMap == nullptr)
    {
        return;
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
    p_newMapFloor->setOpId(-1);
    p_newMapFloor->setOpIdG(-1);
    p_newMapFloor->setId(floorId);
    p_newMapFloor->setCentroid(centroid);
    p_newMapFloor->setMap(p_currentMap);
    p_newMapFloor->setName("semantic::Floor#" + std::to_string(floorId));

    // Add the floor to the map
    p_atlas_inout->addMapFloor(p_newMapFloor);

    std::cout << "[GeoSemHelper] Creating semantic::Floor#"
              << p_newMapFloor->getId() << " ..." << std::endl;
}

} // namespace core
} // namespace vs_graphs
