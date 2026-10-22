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
 * @file            VerifyWallObservationStruct.h
 *
 * @brief           Declares VerifyWallObservation, one wall observation in its
 * own map.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_VERIFY_WALL_OBSERVATION_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_VERIFY_WALL_OBSERVATION_STRUCT_H

#include <Eigen/Geometry>

#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           One wall's plane equation and centroid, already resolved
 *                  through verified getters (Room::getWalls(),
 *                  Room::getWallNormalTowardRoom_world(),
 *                  Plane::getGlobalEquation(), Plane::getCentroid(),
 *                  Plane::getGeometrySnapshot()) and expressed in the SOURCE
 *                  room's own map frame. Never compared raw against another
 *                  map's frame before a hypothesis transform exists.
 */
struct VerifyWallObservation
{
    /*!
     * @brief           Identifier of the wall plane this observation was taken
     *                  from (geometric::Plane::getId()). WallInlierPair refers
     *                  back to observations through it.
     */
    int wallId{0};

    /*!
     * @brief           Canonically oriented via getWallNormalTowardRoom_world
     *                  (n.c_room + d > 0).
     */
    Eigen::Vector3d wallNormal_world{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Plane equation offset paired with wallNormal_world:
     *                  getGlobalEquation() .coeffs()(3), i.e. n^T x + d = 0 --
     *                  NOT g2o::Plane3D::distance(), which returns -d
     *                  (Plane::transformPlaneEquation, Plane.cc:441-442).
     */
    double d{0.0};

    /*!
     * @brief           Centroid of the wall plane in the source room's map
     *                  world frame, metres (Plane::getCentroid()).
     */
    Eigen::Vector3d wallCentroid_world{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Bounded, deterministic stride-sampled points from
     *                  Plane::getGeometrySnapshot().supportCloud.
     */
    std::vector<Eigen::Vector3d> supportSample_world;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_VERIFY_WALL_OBSERVATION_STRUCT_H \
        */
