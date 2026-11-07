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
 * @file            getSceneGraph.cc
 *
 * @brief           Implements Atlas::getSceneGraph(), declared in
 *                  atlas/objects/AtlasClass.h.
 */

#include "map_state/atlas/objects/AtlasClass.h"

namespace vs_graphs
{
namespace core
{
namespace atlas
{

AtlasStatus Atlas::getSceneGraph(scene_graph::SceneGraph *&p_sceneGraph_out)
{
    p_sceneGraph_out = &sceneGraph;

    return AtlasStatus::ATLAS_STATUS_SUCCESS;
}

} /* namespace atlas */
} /* namespace core */
} /* namespace vs_graphs */
