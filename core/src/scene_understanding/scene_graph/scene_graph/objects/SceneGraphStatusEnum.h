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
 * @file            SceneGraphStatusEnum.h
 *
 * @brief           Declares the status returned by every SceneGraph
 *                  operation.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_STATUS_ENUM_H
#define VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

/*!
 * @brief           Result of a SceneGraph operation. Values are fixed and
 *                  never reordered.
 */
enum class SceneGraphStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    SCENE_GRAPH_STATUS_SUCCESS = 0U,

    /*!
     * @brief           The tree has no scene map for the requested SLAM map
     *                  id; nothing was written.
     */
    SCENE_GRAPH_STATUS_MAP_NOT_FOUND = 1U
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_SCENE_GRAPH_STATUS_ENUM_H */
