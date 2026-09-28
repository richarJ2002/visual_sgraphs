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
 * @file            public_functions.h
 *
 * @brief           Declares the public entry points of the
 *                  SemanticAxiomEvaluator module (CPP_CODING_STANDARD.md
 *                  Section 5.4).
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_PUBLIC_FUNCTIONS_H
#define SEMANTIC_AXIOM_EVALUATOR_PUBLIC_FUNCTIONS_H

#include <vector>

#include "Semantic/SemanticAxiomEvaluator/SemanticAxiomEvaluatorStatus.h"
#include "Semantic/SemanticGraphSnapshot/objects/SemanticGraphSnapshot.h"

#include "Semantic/SemanticAxiomEvaluator/objects.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief        Pure, snapshot-only evaluation of every axiom code that
 *               a single static SemanticGraphSnapshot can prove
 *               anything about.
 *
 *               Never mutates \p snapshot_in or any model object,
 *               acquires a lock, calls ROS, logs, reads a clock, or
 *               depends on unordered iteration order -- every output
 *               field is a deterministic function of \p snapshot_in's
 *               own already-sorted content. AX-FRAME-01, AX-TXN-01, and
 *               AX-MERGE-01 (the dynamic-behaviour axioms) each
 *               contribute exactly one UNKNOWN placeholder Finding
 *               here, since a single snapshot cannot prove them -- see
 *               evaluateTransition().
 *
 * @param[in]    snapshot_in
 *               Snapshot to evaluate.
 *
 * @param[out] report_out Complete report: every raw Finding, sorted, plus
 * exactly one aggregate per axiom code.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    evaluateState(const SemanticGraphSnapshot &snapshot_in,
                  AxiomEvaluationReport       &report_out);

/*!
 * @brief        Pure evaluation of the dynamic, before/after axioms
 *               (AX-FRAME-01, AX-TXN-01, AX-MERGE-01) across one
 *               semantic transition, plus every snapshot-evaluable
 *               axiom re-run against \p after_in.
 *
 *               This interface does not yet implement real
 *               frame-equivariance, transaction-idempotence, or
 *               merge-preservation detection: each of the three
 *               dynamic axioms reports a fixed UNKNOWN placeholder
 *               regardless of \p before_in, \p after_in, or
 *               \p context_in (see computeAxiomCapabilityTable()).
 *               Every other axiom code's entries are exactly
 *               evaluateState(after_in)'s own findings/aggregates,
 *               since a static axiom's meaning does not depend on the
 *               prior snapshot. Never mutates either snapshot.
 *
 * @param[in]    before_in
 *               Snapshot immediately before the transition.
 *
 * @param[in]    after_in
 *               Snapshot immediately after the transition.
 *
 * @param[in]    context_in
 *               Reserved transition-specific facts; carries no
 *               fields yet.
 *
 * @param[out] report_out Complete report combining the above.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    evaluateTransition(const SemanticGraphSnapshot       &before_in,
                       const SemanticGraphSnapshot       &after_in,
                       const TransitionEvaluationContext &context_in,
                       AxiomEvaluationReport             &report_out);

/*!
 * @brief        Computes the shadow conservative-completeness result
 *               for every map in \p snapshot_in, paired with an exact
 *               reproduction of the current legacy
 *               (SemanticsManager::Run()) completeness calculation
 *               for the same map.
 *
 *               This is a shadow/comparison calculation only: it does
 *               not change SemanticsManager's own logging-only
 *               calculation or any runtime consumer of it. Never
 *               mutates \p snapshot_in.
 *
 * @param[in]    snapshot_in
 *               Snapshot to compute completeness for.
 *
 * @param[out] completenessResults_out One MapCompletenessResult per entry of \p
 * snapshot_in.maps, sorted by mapId (matching \p snapshot_in.maps's own
 * already-sorted order).
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus evaluateMapCompleteness(
    const SemanticGraphSnapshot        &snapshot_in,
    std::vector<MapCompletenessResult> &completenessResults_out);

/*!
 * @brief        Returns the fixed, snapshot-independent
 *               capability/ownership table for all sixteen axiom
 *               codes.
 *
 *               A pure function of no input: the table describes
 *               this evaluator's current implementation against the
 *               current SemanticGraphSnapshot schema, not any one
 *               evaluated snapshot.
 *
 * @param[out] axiomCapabilityTable_out Exactly sixteen entries, sorted by
 * AxiomCode.
 * @return SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus computeAxiomCapabilityTable(
    std::vector<AxiomCapabilityEntry> &axiomCapabilityTable_out);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_PUBLIC_FUNCTIONS_H
