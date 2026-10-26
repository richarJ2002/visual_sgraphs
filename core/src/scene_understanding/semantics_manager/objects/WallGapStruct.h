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
 * @file            WallGapStruct.h
 *
 * @brief           Declares one free-space opening measured in a wall slab.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_WALL_GAP_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_WALL_GAP_STRUCT_H

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           One opening through a wall, measured from a WallSlabProfile.
 *                  Along positions run from the slab origin along
 *                  WallSlabQuery::alongAxis_world; heights are above the floor.
 */
struct WallGap
{
  public:
    /*!
     * @brief           Left and right ends of the opening along the wall,
     *                  metres, refined at the TSDF zero crossing.
     */
    double alongStart_m{0.0};
    double alongEnd_m{0.0};

    /*!
     * @brief           Lowest and highest free height of the opening, metres.
     */
    double bottomHeight_m{0.0};
    double topHeight_m{0.0};

    /*!
     * @brief           Median width over the bounded door-band rows, metres.
     */
    double width_m{0.0};

    /*!
     * @brief           Opening centre on the wall plane, world frame, metres.
     */
    Eigen::Vector3d centre_world_m{Eigen::Vector3d::Zero()};
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_WALL_GAP_STRUCT_H */
