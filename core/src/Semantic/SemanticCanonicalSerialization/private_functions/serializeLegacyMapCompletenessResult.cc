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
 * @file            serializeLegacyMapCompletenessResult.cc
 *
 * @brief           Implements serializeLegacyMapCompletenessResult(),
 *                  declared in private_functions.h.
 */

#include "Semantic/SemanticCanonicalSerialization/private_functions.h"

#include <algorithm>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

nlohmann::json serializeLegacyMapCompletenessResult(
    const LegacyMapCompletenessResult &value_in)
{
    nlohmann::json json;
    json["confirmedRoomCount"] = value_in.confirmedRoomCount;
    json["completeRoomCount"]  = value_in.completeRoomCount;

    /* Plain std::sort, never erase/unique: a repeated id here is genuine
     * legacy multiplicity evidence (a Room registered in both
     * Map::GetAllDetectedMapRooms() and Map::GetAllMarkerBasedMapRooms()),
     * not a duplicate to discard. */
    std::vector<int> incompleteRoomIds = value_in.incompleteRoomIds;
    std::sort(incompleteRoomIds.begin(), incompleteRoomIds.end());
    json["incompleteRoomIds"] = incompleteRoomIds;

    json["passageCount"]            = value_in.passageCount;
    json["fullyLinkedPassageCount"] = value_in.fullyLinkedPassageCount;

    std::vector<int> danglingPassageIds = value_in.danglingPassageIds;
    std::sort(danglingPassageIds.begin(), danglingPassageIds.end());
    json["danglingPassageIds"] = danglingPassageIds;

    json["mapFullyModeled"] = value_in.mapFullyModeled;
    return json;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
