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
 * @file            EdgeVertexPlaneProjectSE3KFStatus.h
 *
 * @brief           Declares the status returned by every
 *                  EdgeVertexPlaneProjectSE3KF operation.
 */

#ifndef EDGE_VERTEX_PLANE_PROJECT_SE3_KFSTATUS_H
#define EDGE_VERTEX_PLANE_PROJECT_SE3_KFSTATUS_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief           Result of a EdgeVertexPlaneProjectSE3KF operation. Values
 *                  are fixed and never reordered.
 */
enum class EdgeVertexPlaneProjectSE3KFStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    EDGE_VERTEX_PLANE_PROJECT_SE3_KFSTATUS_SUCCESS = 0U
};

} // namespace core
} // namespace vs_graphs

#endif // EDGE_VERTEX_PLANE_PROJECT_SE3_KFSTATUS_H
