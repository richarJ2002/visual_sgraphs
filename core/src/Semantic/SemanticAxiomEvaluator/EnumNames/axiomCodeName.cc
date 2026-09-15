/**
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

std::string axiomCodeName(AxiomCode code_in)
{
    switch (code_in)
    {
    case AxiomCode::AX_FRAME_01:
        return "AX_FRAME_01";
    case AxiomCode::AX_WALL_01:
        return "AX_WALL_01";
    case AxiomCode::AX_WALL_02:
        return "AX_WALL_02";
    case AxiomCode::AX_WALL_03:
        return "AX_WALL_03";
    case AxiomCode::AX_PASS_01:
        return "AX_PASS_01";
    case AxiomCode::AX_PASS_02:
        return "AX_PASS_02";
    case AxiomCode::AX_PASS_03:
        return "AX_PASS_03";
    case AxiomCode::AX_PASS_04:
        return "AX_PASS_04";
    case AxiomCode::AX_ROOM_01:
        return "AX_ROOM_01";
    case AxiomCode::AX_ROOM_02:
        return "AX_ROOM_02";
    case AxiomCode::AX_BOUND_01:
        return "AX_BOUND_01";
    case AxiomCode::AX_FLOOR_01:
        return "AX_FLOOR_01";
    case AxiomCode::AX_LIFE_01:
        return "AX_LIFE_01";
    case AxiomCode::AX_TXN_01:
        return "AX_TXN_01";
    case AxiomCode::AX_COMP_01:
        return "AX_COMP_01";
    case AxiomCode::AX_MERGE_01:
        return "AX_MERGE_01";
    }
    return "UNKNOWN_AXIOM_CODE";
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
