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

/*!
 * @file            printBootstrapRecord.cc
 *
 * @brief           Implements printBootstrapRecord(), declared in
 *                  bootstrap/private_functions/private_functions.h.
 */

#include "scene_understanding/semantics_manager/bootstrap/private_functions/private_functions.h"

#include <iostream>

namespace vs_graphs
{
namespace core
{

void printBootstrapRecord(
    const std::string                  &reason_in,
    const std::optional<unsigned long> &mapId_in,
    const std::optional<int>           &roomId_in,
    const std::optional<int>           &floorId_in,
    const std::optional<std::size_t>   &restoredPassageCount_in,
    std::uint64_t                       semanticCycle_in)
{
    /* The field names and order are read by offline tooling. */
    std::cout << "SG_PIPELINE {\"event\":\"initialization\"";
    if (mapId_in.has_value())
    {
        std::cout << ",\"map_id\":" << *mapId_in;
    }
    std::cout << ",\"reason\":\"" << reason_in << "\"";
    if (roomId_in.has_value())
    {
        std::cout << ",\"room_id\":" << *roomId_in;
    }
    if (floorId_in.has_value())
    {
        std::cout << ",\"floor_id\":" << *floorId_in;
    }
    if (restoredPassageCount_in.has_value())
    {
        std::cout << ",\"restored_passages\":" << *restoredPassageCount_in;
    }
    std::cout << ",\"semantic_cycle\":" << semanticCycle_in << "}" << std::endl;
}

} /* namespace core */
} /* namespace vs_graphs */
