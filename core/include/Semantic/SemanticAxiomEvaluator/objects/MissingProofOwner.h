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
 * @file            MissingProofOwner.h
 *
 * @brief           Declares which future plan phase (if any) owns supplying
 *                  the proof a PARTIAL/DEFERRED axiom code currently lacks.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_MISSING_PROOF_OWNER_H
#define SEMANTIC_AXIOM_EVALUATOR_MISSING_PROOF_OWNER_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Names the plan phase responsible for the evidence a
 *              PARTIAL/DEFERRED axiom code cannot yet prove.
 *
 *              Values reproduce semantic-axiom-reliability-plan.md's own
 *              phase numbering exactly, per this slice's fixed assignment
 *              table (axiomCapabilityTable()). NONE is used only for a FULL
 *              capability axiom, where nothing is missing. SCOPE_DECISION_
 *              REQUIRED is used only where the plan itself names no owner
 *              (never invented) -- see AxiomCapabilityEntry.h.
 */
enum class MissingProofOwner : std::uint8_t
{
    /*! @brief Nothing missing; the axiom's capability is FULL. */
    NONE = 0U,

    /*! @brief semantic-axiom-reliability-plan.md Phase 2 (frame/face
     *  provenance). */
    PHASE_2 = 1U,

    /*! @brief Phase 3 (passage endpoints and room origins authoritative). */
    PHASE_3 = 2U,

    /*! @brief Phase 4 (observation-based wall ownership). */
    PHASE_4 = 3U,

    /*! @brief Phase 4 or Phase 5, as applicable to the specific quarantine/
     *  reconciliation case. */
    PHASE_4_OR_5 = 4U,

    /*! @brief Phase 5 (full passage provenance/reconciliation). */
    PHASE_5 = 5U,

    /*! @brief Phase 6 (RViz/observability -- boundary geometry proof). */
    PHASE_6 = 6U,

    /*! @brief Phase 7 (unified reconciler/transaction, completeness
     *  authority switch). */
    PHASE_7 = 7U,

    /*! @brief Phase 8 (verified map merge). */
    PHASE_8 = 8U,

    /*! @brief The plan currently names no owner for this specific gap; do
     *  not infer one. */
    SCOPE_DECISION_REQUIRED = 9U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_MISSING_PROOF_OWNER_H
