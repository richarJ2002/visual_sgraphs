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
 * @file            doSegmentsIntersect.cc
 *
 * @brief           Implements doSegmentsIntersect(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus doSegmentsIntersect(const Eigen::Vector2d &p1_in,
                                                 const Eigen::Vector2d &q1_in,
                                                 const Eigen::Vector2d &p2_in,
                                                 const Eigen::Vector2d &q2_in,
                                                 bool &doSegmentsIntersect_out)
{
    double orientation1{};
    if (orientation2d(p1_in, q1_in, p2_in, orientation1) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // orientation2d cannot fail; continue as before.
    }
    double orientation2{};
    if (orientation2d(p1_in, q1_in, q2_in, orientation2) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // orientation2d cannot fail; continue as before.
    }
    double orientation3{};
    if (orientation2d(p2_in, q2_in, p1_in, orientation3) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // orientation2d cannot fail; continue as before.
    }
    double orientation4{};
    if (orientation2d(p2_in, q2_in, q1_in, orientation4) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // orientation2d cannot fail; continue as before.
    }

    if (((orientation1 > 0.0) != (orientation2 > 0.0)) &&
        ((orientation3 > 0.0) != (orientation4 > 0.0)) && orientation1 != 0.0 &&
        orientation2 != 0.0 && orientation3 != 0.0 && orientation4 != 0.0)
    {
        doSegmentsIntersect_out = true;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    bool isOnSegmentBoundingBox2{};
    if ((orientation1 == 0.0) &&
        isOnSegmentBoundingBox(p1_in, q1_in, p2_in, isOnSegmentBoundingBox2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // isOnSegmentBoundingBox cannot fail; continue as before.
    }
    if (orientation1 == 0.0 && isOnSegmentBoundingBox2)
    {
        doSegmentsIntersect_out = true;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    bool isOnSegmentBoundingBox3{};
    if ((orientation2 == 0.0) &&
        isOnSegmentBoundingBox(p1_in, q1_in, q2_in, isOnSegmentBoundingBox3) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // isOnSegmentBoundingBox cannot fail; continue as before.
    }
    if (orientation2 == 0.0 && isOnSegmentBoundingBox3)
    {
        doSegmentsIntersect_out = true;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    bool isOnSegmentBoundingBox4{};
    if ((orientation3 == 0.0) &&
        isOnSegmentBoundingBox(p2_in, q2_in, p1_in, isOnSegmentBoundingBox4) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // isOnSegmentBoundingBox cannot fail; continue as before.
    }
    if (orientation3 == 0.0 && isOnSegmentBoundingBox4)
    {
        doSegmentsIntersect_out = true;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    bool isOnSegmentBoundingBox5{};
    if ((orientation4 == 0.0) &&
        isOnSegmentBoundingBox(p2_in, q2_in, q1_in, isOnSegmentBoundingBox5) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        // isOnSegmentBoundingBox cannot fail; continue as before.
    }
    if (orientation4 == 0.0 && isOnSegmentBoundingBox5)
    {
        doSegmentsIntersect_out = true;
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    doSegmentsIntersect_out = false;
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
