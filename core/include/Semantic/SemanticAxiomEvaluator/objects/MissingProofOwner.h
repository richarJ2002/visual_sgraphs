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
 * @file            MissingProofOwner.h
 *
 * @brief           Declares which future evidence area (if any) owns
 *                  supplying the proof a PARTIAL/DEFERRED axiom code
 *                  currently lacks.
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
 * @brief           Names the evidence area responsible for the proof a
 *                  PARTIAL/DEFERRED axiom code cannot yet supply.
 *
 *                  Values name the missing-evidence area exactly, per
 *                  this module's fixed assignment table
 *                  (computeAxiomCapabilityTable()). NONE is used only
 *                  for a FULL capability axiom, where nothing is
 *                  missing. SCOPE_DECISION_REQUIRED is used only where
 *                  no owner area is named (never invented) -- see
 *                  AxiomCapabilityEntry.h.
 */
enum class MissingProofOwner : std::uint8_t
{
    /*!
     * @brief           Nothing missing; the axiom's capability is FULL.
     */
    NONE = 0U,

    /*!
     * @brief           Frame/face provenance.
     */
    PHASE_2 = 1U,

    /*!
     * @brief           Passage endpoints and room origins authoritative.
     */
    PHASE_3 = 2U,

    /*!
     * @brief           Observation-based wall ownership.
     */
    PHASE_4 = 3U,

    /*!
     * @brief           Quarantine/reconciliation case: observation-based
     *                  wall ownership or full passage
     *                  provenance/reconciliation, as applicable to the
     *                  specific case.
     */
    PHASE_4_OR_5 = 4U,

    /*!
     * @brief           Full passage provenance/reconciliation.
     */
    PHASE_5 = 5U,

    /*!
     * @brief           RViz/observability boundary-geometry proof.
     */
    PHASE_6 = 6U,

    /*!
     * @brief           Unified reconciler/transaction, completeness
     *                  authority switch.
     */
    PHASE_7 = 7U,

    /*!
     * @brief           Verified map merge.
     */
    PHASE_8 = 8U,

    /*!
     * @brief           No owner area is named for this specific gap; do
     *                  not infer one.
     */
    SCOPE_DECISION_REQUIRED = 9U
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_MISSING_PROOF_OWNER_H
