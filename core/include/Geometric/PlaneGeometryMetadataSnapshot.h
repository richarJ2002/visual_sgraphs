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
 * @file            PlaneGeometryMetadataSnapshot.h
 *
 * @brief           Declares the cheap, cloud-free scalar plane-geometry
 *                  value type returned by Plane::getGeometryMetadataSnapshot().
 */

#ifndef PLANE_GEOMETRY_METADATA_SNAPSHOT_H
#define PLANE_GEOMETRY_METADATA_SNAPSHOT_H

#include <cstddef>
#include <cstdint>

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{
namespace geometric
{
/*!
 * @brief       Immutable copy of the cheap scalar plane-geometry fields,
 *              without the point-cloud payload that
 *              Plane::getGeometrySnapshot() deep-copies.
 *
 *              A project-owned value type, not a nested member of Plane
 *              (CPP_CODING_STANDARD.md Section 5.2: one project-owned object
 *              type per header): a class shall not define a nested
 *              project-owned type merely because it uses that type.
 */
struct PlaneGeometryMetadataSnapshot
{
  public:
    /*! @brief World-frame plane equation (nx, ny, nz, d). */
    Eigen::Vector4d equation_World{Eigen::Vector4d::Zero()};

    /*! @brief World-frame plane centroid, meters. */
    Eigen::Vector3d centroid_World_m{Eigen::Vector3d::Zero()};

    /*! @brief Minimum in-plane grid coordinate along axis U, meters. */
    double minPlaneU_m{0.0};

    /*! @brief Maximum in-plane grid coordinate along axis U, meters. */
    double maxPlaneU_m{0.0};

    /*! @brief Minimum in-plane grid coordinate along axis V, meters. */
    double minPlaneV_m{0.0};

    /*! @brief Maximum in-plane grid coordinate along axis V, meters. */
    double maxPlaneV_m{0.0};

    /*! @brief Count of finite-support grid cells from the last successful
     *  refit. */
    std::size_t finiteSupportCount{0U};

    /*! @brief Total observation count accumulated for this plane. */
    std::size_t observationCount{0U};

    /*! @brief Monotonic generation counter, incremented each time the
     *  support cloud is regenerated. */
    std::uint64_t cloudGeneration{0U};

    /*! @brief Monotonic generation counter, incremented each time a refit
     *  succeeds. */
    std::uint64_t successfulRefitGeneration{0U};
};

} // namespace geometric
} // namespace core
} // namespace vs_graphs

#endif // PLANE_GEOMETRY_METADATA_SNAPSHOT_H
