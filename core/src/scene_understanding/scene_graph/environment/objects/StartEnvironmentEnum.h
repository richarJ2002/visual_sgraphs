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
 * @file            StartEnvironmentEnum.h
 *
 * @brief           Declares StartEnvironment, the kind of place a mission
 *                  starts in.
 */

#ifndef VS_GRAPHS_SCENE_GRAPH_ENVIRONMENT_START_ENVIRONMENT_ENUM_H
#define VS_GRAPHS_SCENE_GRAPH_ENVIRONMENT_START_ENVIRONMENT_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace scene_graph
{

/*!
 * @brief           Where the robot starts, set by the configuration key
 *                  semantics_manager.start_environment. Values are fixed and
 *                  never reordered.
 */
enum class StartEnvironment : std::uint8_t
{
    /*! The robot starts inside a building: the scene graph begins with
     *  building 0, floor 0 and room 0. */
    START_ENVIRONMENT_INDOOR = 0U,

    /*! The robot starts outside: the scene graph begins with the environment
     *  node only; a building, floor and room appear when the first room is
     *  detected. */
    START_ENVIRONMENT_OUTDOOR = 1U
};

} /* namespace scene_graph */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SCENE_GRAPH_ENVIRONMENT_START_ENVIRONMENT_ENUM_H */
