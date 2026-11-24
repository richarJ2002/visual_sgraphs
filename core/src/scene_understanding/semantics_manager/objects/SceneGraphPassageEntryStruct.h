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
 * @file            SceneGraphPassageEntryStruct.h
 *
 * @brief           Declares SceneGraphPassageEntry, the plain-data copy of a
 *                  passage inside a SceneGraphSnapshot.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_PASSAGE_ENTRY_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_PASSAGE_ENTRY_STRUCT_H

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           What the publisher, the archive and the health report need
 *                  to know about one passage, copied out under the graph lock.
 *                  Holds no pointer into the graph: rooms and planes are ids.
 */
struct SceneGraphPassageEntry
{
  public:
    /*!
     * @brief           Passage id, unique within its map.
     */
    int id{-1};

    /*!
     * @brief           True when the passage is flagged bad.
     */
    bool isBad{false};

    /*!
     * @brief           True when the passage is open to pass.
     */
    bool isPassable{false};

    /*!
     * @brief           True when the passage keeps identity and topology but
     *                  has no positioned geometry until it is observed again.
     */
    bool isRecoveryProxy{false};

    /*!
     * @brief           True when a traversal through the passage was observed.
     */
    bool isTraversed{false};

    /*!
     * @brief           True when the passage is blocked.
     */
    bool isBlocked{false};

    /*!
     * @brief           Opening width, metres.
     */
    double width_m{0.0};

    /*!
     * @brief           Opening height, metres.
     */
    double height_m{0.0};

    /*!
     * @brief           Passage centroid in the map's world frame, metres.
     */
    Eigen::Vector3d centroid_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Id of the room known to occupy the observing side;
     *                  empty when none.
     */
    std::optional<int> knownSideRoomId;

    /*!
     * @brief           Id of the room beyond the passage; empty when none.
     */
    std::optional<int> prospectiveRoomId;

    /*!
     * @brief           Traversals from the known side to the far side.
     */
    std::size_t traversalKnownToFarCount{0U};

    /*!
     * @brief           Traversals from the far side to the known side.
     */
    std::size_t traversalFarToKnownCount{0U};

    /*!
     * @brief           Traversals whose direction is unknown.
     */
    std::size_t traversalUnknownCount{0U};

    /*!
     * @brief           Ids of the wall planes around the opening. Resolved by
     *                  the map that owns the planes.
     */
    std::vector<int> associatedWallIds;
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTICS_MANAGER_OBJECTS_SCENE_GRAPH_PASSAGE_ENTRY_STRUCT_H \
        */
