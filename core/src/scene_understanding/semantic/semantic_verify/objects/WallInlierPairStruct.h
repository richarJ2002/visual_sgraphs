/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            WallInlierPairStruct.h
 *
 * @brief           Declares WallInlierPair, one agreeing wall pair under a
 * transform.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_WALL_INLIER_PAIR_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_WALL_INLIER_PAIR_STRUCT_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           One pair of walls, one from each room, that agrees under the
 *                  transform found by verify().
 */
struct WallInlierPair
{
    /*!
     * @brief           Identifier of the paired wall in room A
     *                  (VerifyWallObservation::wallId).
     */
    int    wallIdA{0};
    /*!
     * @brief           Identifier of the paired wall in room B
     *                  (VerifyWallObservation::wallId).
     */
    int    wallIdB{0};
    /*!
     * @brief           Angle between room A's wall normal, rotated into room
     *                  B's frame, and room B's wall normal, radians.
     */
    double normalAngleResidual_rad{0.0};
    /*!
     * @brief           Absolute difference between room A's wall offset, moved
     *                  into room B's frame, and room B's wall offset, metres.
     */
    double offsetResidual_m{0.0};
    /*!
     * @brief           Symmetric distance between the support samples of the
     *                  two walls under the transform, metres.
     */
    double supportDistResidual_m{0.0};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_WALL_INLIER_PAIR_STRUCT_H \
        */
