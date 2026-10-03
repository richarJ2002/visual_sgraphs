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
 * @file            SemanticGraphSnapshotTestHelpers.h
 *
 * @brief           Declares lookup helpers shared by SemanticGraphSnapshot
 *                  tests (CPP_CODING_STANDARD.md Section 5.4: one ordinary
 *                  free function per implementation file).
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_TEST_HELPERS_H
#define SEMANTIC_GRAPH_SNAPSHOT_TEST_HELPERS_H

#include "Semantic/SemanticGraphSnapshot.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Finds the RoomRecord in \p mapSnapshot_in with local id
 *                  \p entityId_in, or nullptr if absent.
 */
const RoomRecord *findRoomRecord(const MapSnapshot &mapSnapshot_in,
                                 int                entityId_in);

/*!
 * @brief           Finds the WallRecord in \p mapSnapshot_in with local id
 *                  \p entityId_in, or nullptr if absent.
 */
const WallRecord *findWallRecord(const MapSnapshot &mapSnapshot_in,
                                 int                entityId_in);

/*!
 * @brief           Finds the PassageRecord in \p mapSnapshot_in with local id
 *                  \p entityId_in, or nullptr if absent.
 */
const PassageRecord *findPassageRecord(const MapSnapshot &mapSnapshot_in,
                                       int                entityId_in);

/*!
 * @brief           Finds the MapSnapshot for \p mapId_in, or nullptr if absent.
 */
const MapSnapshot *findMapSnapshot(const SemanticGraphSnapshot &snapshot_in,
                                   long unsigned int            mapId_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_TEST_HELPERS_H
