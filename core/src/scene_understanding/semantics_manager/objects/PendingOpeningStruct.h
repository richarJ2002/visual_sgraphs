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
 * @file            PendingOpeningStruct.h
 *
 * @brief           Declares an opening seen in a wall slab that is not yet a
 *                  Passage.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_PENDING_OPENING_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_PENDING_OPENING_STRUCT_H

#include <cstddef>

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           An opening measured OPEN in consecutive wall-slab profiles;
 *                  it becomes a Passage after minimumOpenVerdicts of them.
 */
struct PendingOpening
{
  public:
    /*!
     * @brief           Id of the wall the opening lies in.
     */
    int wallId{-1};

    /*!
     * @brief           Latest opening centre, world frame, metres.
     */
    Eigen::Vector3d centre_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Consecutive profiles of this wall that measured the
     *                  opening OPEN.
     */
    std::size_t consecutiveOpenVerdicts{0U};
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_PENDING_OPENING_STRUCT_H */
