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
 * @file            checkRoomBoundaryGeometry.cc
 *
 * @brief           Implements checkRoomBoundaryGeometry(), declared in
 *                  private_functions.h.
 *
 *                  Every corner is explicitly checked for finiteness
 *                  before any comparison-based degenerate/
 *                  self-intersection logic runs. A NaN operand makes every
 *                  `<`/`<=`/`>=` comparison false, so without that check a
 *                  non-finite corner could silently reach VALID -- see the
 *                  `NonFiniteCornerIsFail`-class tests.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <cmath>
#include <cstddef>

#include <Eigen/Geometry>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

namespace
{
constexpr double DEGENERATE_EDGE_LENGTH_SQUARED_M2 = 1e-12;
} // namespace

SemanticAxiomEvaluatorStatus
    checkRoomBoundaryGeometry(const std::vector<Eigen::Vector3d> &corners_in,
                              RoomBoundaryGeometryStatus &geometryStatus_out)
{
    const std::size_t cornerCount = corners_in.size();
    if (cornerCount < 3U)
    {
        geometryStatus_out = RoomBoundaryGeometryStatus::TOO_FEW_CORNERS;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    bool hasNonFiniteCoordinate2{};
    if (hasNonFiniteCoordinate(corners_in, hasNonFiniteCoordinate2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // hasNonFiniteCoordinate cannot fail; continue as before.
    }
    if (hasNonFiniteCoordinate2)
    {
        geometryStatus_out = RoomBoundaryGeometryStatus::NON_FINITE_CORNER;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    for (std::size_t cornerIndex = 0U; cornerIndex < cornerCount; ++cornerIndex)
    {
        const Eigen::Vector3d edge =
            corners_in[(cornerIndex + 1U) % cornerCount] -
            corners_in[cornerIndex];
        if (edge.squaredNorm() < DEGENERATE_EDGE_LENGTH_SQUARED_M2)
        {
            geometryStatus_out = RoomBoundaryGeometryStatus::DEGENERATE_EDGE;
            return SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
        }
    }

    Eigen::Vector3d normal{};
    if (newellNormal(corners_in, normal) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // newellNormal cannot fail; continue as before.
    }
    if (normal.squaredNorm() < DEGENERATE_NORMAL_NORM_SQUARED)
    {
        Eigen::Vector3d normal2{};
        if (planeNormalFallback(corners_in, normal2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            // planeNormalFallback cannot fail; continue as before.
        }
        normal = normal2;
    }
    if (normal.squaredNorm() < DEGENERATE_NORMAL_NORM_SQUARED)
    {
        /* Every corner is collinear (or coincident beyond the per-edge
         * check above): no well-defined polygon plane/area exists. */
        geometryStatus_out = RoomBoundaryGeometryStatus::DEGENERATE_EDGE;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    const Eigen::Vector3d unitNormal = normal.normalized();

    /* Any vector not parallel to unitNormal seeds an orthonormal in-plane
     * basis; the world X axis fails only when unitNormal is itself world X,
     * in which case world Y is used instead. */
    Eigen::Vector3d seed = Eigen::Vector3d::UnitX();
    if (std::abs(unitNormal.dot(seed)) > 0.9)
    {
        seed = Eigen::Vector3d::UnitY();
    }
    const Eigen::Vector3d axisU =
        (seed - unitNormal * unitNormal.dot(seed)).normalized();
    const Eigen::Vector3d axisV = unitNormal.cross(axisU);

    std::vector<Eigen::Vector2d> projected;
    projected.reserve(cornerCount);
    for (const Eigen::Vector3d &corner : corners_in)
    {
        projected.emplace_back(corner.dot(axisU), corner.dot(axisV));
    }

    for (std::size_t edgeIndexA = 0U; edgeIndexA < cornerCount; ++edgeIndexA)
    {
        const std::size_t edgeAStart = edgeIndexA;
        const std::size_t edgeAEnd   = (edgeIndexA + 1U) % cornerCount;
        for (std::size_t edgeIndexB = edgeIndexA + 1U; edgeIndexB < cornerCount;
             ++edgeIndexB)
        {
            const std::size_t edgeBStart = edgeIndexB;
            const std::size_t edgeBEnd   = (edgeIndexB + 1U) % cornerCount;

            /* Adjacent edges share exactly one vertex by construction; that
             * shared touch point is not a self-intersection. */
            if (edgeBStart == edgeAEnd || edgeBEnd == edgeAStart ||
                edgeBStart == edgeAStart)
            {
                continue;
            }

            bool doSegmentsIntersect2{};
            if (doSegmentsIntersect(projected[edgeAStart],
                                    projected[edgeAEnd],
                                    projected[edgeBStart],
                                    projected[edgeBEnd],
                                    doSegmentsIntersect2) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // doSegmentsIntersect cannot fail; continue as before.
            }
            if (doSegmentsIntersect2)
            {
                geometryStatus_out =
                    RoomBoundaryGeometryStatus::SELF_INTERSECTING;
                return SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
            }
        }
    }

    geometryStatus_out = RoomBoundaryGeometryStatus::VALID;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
