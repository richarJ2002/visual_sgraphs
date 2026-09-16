/**
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
 * @file            MissionHealthTopologyJson.h
 *
 * @brief           Declares augmentMissionHealthTopologyJsonWithSemantics(),
 *                   the pure function extending
 *                   /vs_graphs/get_mission_health's existing schema-1
 *                   topology_json to schema 2 with copied-cache evaluator
 *                   additions.
 *                   Pure and ROS-free so it is directly unit-testable.
 */

#ifndef MISSION_HEALTH_TOPOLOGY_JSON_H
#define MISSION_HEALTH_TOPOLOGY_JSON_H

#include "Thirdparty/nlohmann/json.hpp"

#include "Semantic/SemanticReportCache/objects/SemanticReportCacheEntry.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief       Returns a copy of \p topologyJson_in with every existing
 *              schema-1 key and type preserved unchanged, "schema" set to
 *              2, and copied-cache evaluator additions appended.
 *
 *              Never reacquires an evaluator model pointer or mutates
 *              tracking/semantic state -- every added field is read
 *              straight from \p entry_in, itself already a copied,
 *              pointer-free value.
 *
 * @param[in]   topologyJson_in     The existing schema-1 topology object
 *                                  (unmodified fields/types are preserved).
 * @param[in]   entry_in            Latest copied semantic report cache
 *                                  entry (meaningless when \p
 *                                  cacheAvailable_in is false).
 * @param[in]   cacheAvailable_in   Whether \p entry_in reflects a real
 *                                  completed semantic cycle.
 *
 * @return      \p topologyJson_in extended to schema 2. When \p
 *              cacheAvailable_in is false, only "schema" and
 *              "semanticCacheAvailable" are added -- no evaluator field is
 *              fabricated from a meaningless entry.
 */
nlohmann::json augmentMissionHealthTopologyJsonWithSemantics(
    nlohmann::json                            topologyJson_in,
    const semantic::SemanticReportCacheEntry &entry_in,
    bool                                      cacheAvailable_in);

} // namespace core
} // namespace vs_graphs

#endif // MISSION_HEALTH_TOPOLOGY_JSON_H
