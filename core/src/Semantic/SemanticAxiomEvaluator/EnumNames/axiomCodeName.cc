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
 * @file            axiomCodeName.cc
 *
 * @brief           Implements axiomCodeName(), declared in EnumNames.h.
 */

#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus axiomCodeName(AxiomCode    code_in,
                                           std::string &axiomCodeName_out)
{
    switch (code_in)
    {
    case AxiomCode::AX_FRAME_01:
    {
        axiomCodeName_out = "AX_FRAME_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_WALL_01:
    {
        axiomCodeName_out = "AX_WALL_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_WALL_02:
    {
        axiomCodeName_out = "AX_WALL_02";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_WALL_03:
    {
        axiomCodeName_out = "AX_WALL_03";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_PASS_01:
    {
        axiomCodeName_out = "AX_PASS_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_PASS_02:
    {
        axiomCodeName_out = "AX_PASS_02";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_PASS_03:
    {
        axiomCodeName_out = "AX_PASS_03";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_PASS_04:
    {
        axiomCodeName_out = "AX_PASS_04";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_ROOM_01:
    {
        axiomCodeName_out = "AX_ROOM_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_ROOM_02:
    {
        axiomCodeName_out = "AX_ROOM_02";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_BOUND_01:
    {
        axiomCodeName_out = "AX_BOUND_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_FLOOR_01:
    {
        axiomCodeName_out = "AX_FLOOR_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_LIFE_01:
    {
        axiomCodeName_out = "AX_LIFE_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_TXN_01:
    {
        axiomCodeName_out = "AX_TXN_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_COMP_01:
    {
        axiomCodeName_out = "AX_COMP_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    case AxiomCode::AX_MERGE_01:
    {
        axiomCodeName_out = "AX_MERGE_01";
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    }
    axiomCodeName_out = "UNKNOWN_AXIOM_CODE";
    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
