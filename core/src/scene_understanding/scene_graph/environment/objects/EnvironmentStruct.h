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
 * @file            EnvironmentStruct.h
 *
 * @brief           Declares Environment, the top node of a scene graph.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_ENVIRONMENT_ENVIRONMENT_STRUCT_H
#define VS_GRAPHS_SCENE_GRAPH_ENVIRONMENT_ENVIRONMENT_STRUCT_H

#include <string>

#include "scene_understanding/scene_graph/environment/objects/StartEnvironmentEnum.h"

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

/*!
 * @brief           The top node of a scene graph: the place the mission
 *                  happens in. Its children are the buildings of the graph; an
 *                  outdoor start has none until the first room is detected.
 *                  Plain data, owned by its SceneGraph and guarded by the
 *                  SemanticsManager graph mutex.
 */
struct Environment
{
  public:
    /*!
     * @brief           Environment id; one environment per map, id 0.
     */
    int id{0};

    /*!
     * @brief           Display name of the environment.
     */
    std::string name{"Environment"};

    /*!
     * @brief           Kind of place the mission started in.
     */
    StartEnvironment startEnvironment{
        StartEnvironment::START_ENVIRONMENT_INDOOR};
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_ENVIRONMENT_ENVIRONMENT_STRUCT_H */
