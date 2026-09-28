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
 * @file            evaluateAxPass01.cc
 *
 * @brief           Implements evaluateAxPass01(), declared in
 *                  private_functions.h.
 *
 *                  Passage::isPassable() is set true only when Voxblox
 *                  ESDF free-space crossing evidence is observed (confirmed
 *                  by direct source read: SemanticsManager.cc's passage-
 *                  detection pass calls setPassable(true) specifically
 *                  because "connected ESDF free space through the wall is
 *                  stronger evidence than a stale blocked-door
 *                  classification"), so a live, non-passable passage
 *                  directly contradicts "a closed door plane alone must not
 *                  create a passage" -- an observable FAIL, not merely
 *                  unproven. passable() is itself only a derived boolean,
 *                  not a retained chain-of-custody provenance record, so
 *                  full aperture/skeleton provenance remains unverifiable
 *                  either way.
 *
 *                  The per-passage logic lives in
 *                  evaluateOnePassageProvenance() so
 *                  computeConservativeMapCompleteness() can share the
 *                  identical leaf rather than re-deriving its own
 *                  passable() check.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateAxPass01(const SemanticGraphSnapshot &snapshot_in,
                     std::vector<Finding>        &findings_inout)
{
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        for (const PassageRecord &passage : mapSnapshot.passages)
        {
            if (!passage.isLive)
            {
                continue;
            }
            if (evaluateOnePassageProvenance(passage, findings_inout) !=
                SemanticAxiomEvaluatorStatus::
                    SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
            {
                // evaluateOnePassageProvenance cannot fail; continue as before.
            }
        }
    }

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
