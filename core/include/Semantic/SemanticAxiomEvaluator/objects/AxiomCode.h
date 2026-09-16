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
 * @file         AxiomCode.h
 *
 * @brief        Declares the stable string-backed identity of each
 *               canonical semantic axiom.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_AXIOM_CODE_H
#define SEMANTIC_AXIOM_EVALUATOR_AXIOM_CODE_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief        One of the sixteen canonical axiom codes. Declaration
 *               order here has no semantic meaning; every consumer
 *               that needs a stable presentation order (the
 *               capability table, the aggregate report) sorts
 *               explicitly rather than relying on enumerator order.
 */
enum class AxiomCode : std::uint8_t
{
    /*! @brief Frame/transform equivariance. */
    AX_FRAME_01 = 0U,

    /*! @brief Unique live wall ownership. */
    AX_WALL_01 = 1U,

    /*! @brief Observation-room wall ownership beyond a passage crossing. */
    AX_WALL_02 = 2U,

    /*! @brief Twin wall-face plausibility. */
    AX_WALL_03 = 3U,

    /*! @brief Published-passage aperture/skeleton provenance. */
    AX_PASS_01 = 4U,

    /*! @brief Passage endpoint cardinality and reciprocity. */
    AX_PASS_02 = 5U,

    /*! @brief Passage known-side/prospective slot consistency. */
    AX_PASS_03 = 6U,

    /*! @brief Passage endpoint map/floor agreement. */
    AX_PASS_04 = 7U,

    /*! @brief Room bootstrap/creation-provenance uniqueness. */
    AX_ROOM_01 = 8U,

    /*! @brief Prospective-to-confirmed room promotion evidence. */
    AX_ROOM_02 = 9U,

    /*! @brief Room boundary completeness meaning. */
    AX_BOUND_01 = 10U,

    /*! @brief Room-floor reciprocity and passage floor identity. */
    AX_FLOOR_01 = 11U,

    /*! @brief Quarantine-not-erase evidence lifecycle. */
    AX_LIFE_01 = 12U,

    /*! @brief Deterministic, idempotent, all-or-nothing reconciliation. */
    AX_TXN_01 = 13U,

    /*! @brief Derived map-completeness aggregate. */
    AX_COMP_01 = 14U,

    /*! @brief Map-merge provenance/axiom preservation. */
    AX_MERGE_01 = 15U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_AXIOM_CODE_H
