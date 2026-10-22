/*!
 * @file            semantic_axiom_evaluator.h
 *
 * @brief           Public interface of the semantic_axiom_evaluator module.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_SEMANTIC_AXIOM_EVALUATOR_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_SEMANTIC_AXIOM_EVALUATOR_H

#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AggregateAxiomResultStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomCapabilityEntryStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomClassEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomCodeEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomEvaluationReportStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomResultEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/CapabilityLevelEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/FindingEvidenceStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/FindingStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/LegacyMapCompletenessResultStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/MapCompletenessResultStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/MissingProofOwnerEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/PassageFloorAgreementEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/ReasonCodeEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/ResolvedRoomEndpointStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/ReversePassageEndpointScanStruct.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/RoomBoundaryGeometryStatusEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/RoomBoundaryWallEvidenceStatusEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/SemanticAxiomEvaluatorStatusEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/TransitionEvaluationContextStruct.h"


/* Recovered public contracts from the pre-migration API. */
#include <string>
#include <vector>
#include "scene_understanding/semantic/semantic_graph_snapshot/semantic_graph_snapshot.h"

namespace vs_graphs::core::semantic
{
/*!
 * @brief           Pure, snapshot-only evaluation of every axiom code that
 *                  a single static SemanticGraphSnapshot can prove
 *                  anything about.
 *
 *                  Never mutates \p snapshot_in or any model object,
 *                  acquires a lock, calls ROS, logs, reads a clock, or
 *                  depends on unordered iteration order -- every output
 *                  field is a deterministic function of \p snapshot_in's
 *                  own already-sorted content. AX-FRAME-01, AX-TXN-01, and
 *                  AX-MERGE-01 (the dynamic-behaviour axioms) each
 *                  contribute exactly one UNKNOWN placeholder Finding
 *                  here, since a single snapshot cannot prove them -- see
 *                  evaluateTransition().
 *
 * @param[in]       snapshot_in
 *                  Snapshot to evaluate.
 *
 * @param[out]      report_out
 *                  Complete report: every raw Finding, sorted, plus exactly one
 *                  aggregate per axiom code.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    evaluateState(const SemanticGraphSnapshot &snapshot_in,
                  AxiomEvaluationReport       &report_out);

/*!
 * @brief           Pure evaluation of the dynamic, before/after axioms
 *                  (AX-FRAME-01, AX-TXN-01, AX-MERGE-01) across one
 *                  semantic transition, plus every snapshot-evaluable
 *                  axiom re-run against \p after_in.
 *
 *                  This interface does not yet implement real
 *                  frame-equivariance, transaction-idempotence, or
 *                  merge-preservation detection: each of the three
 *                  dynamic axioms reports a fixed UNKNOWN placeholder
 *                  regardless of \p before_in, \p after_in, or
 *                  \p context_in (see computeAxiomCapabilityTable()).
 *                  Every other axiom code's entries are exactly
 *                  evaluateState(after_in)'s own findings/aggregates,
 *                  since a static axiom's meaning does not depend on the
 *                  prior snapshot. Never mutates either snapshot.
 *
 * @param[in]       before_in
 *                  Snapshot immediately before the transition.
 *
 * @param[in]       after_in
 *                  Snapshot immediately after the transition.
 *
 * @param[in]       context_in
 *                  Reserved transition-specific facts; carries no
 *                  fields yet.
 *
 * @param[out]      report_out
 *                  Complete report combining the above.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    evaluateTransition(const SemanticGraphSnapshot       &before_in,
                       const SemanticGraphSnapshot       &after_in,
                       const TransitionEvaluationContext &context_in,
                       AxiomEvaluationReport             &report_out);

/*!
 * @brief           Computes the shadow conservative-completeness result
 *                  for every map in \p snapshot_in, paired with an exact
 *                  reproduction of the current legacy
 *                  (SemanticsManager::Run()) completeness calculation
 *                  for the same map.
 *
 *                  This is a shadow/comparison calculation only: it does
 *                  not change SemanticsManager's own logging-only
 *                  calculation or any runtime consumer of it. Never
 *                  mutates \p snapshot_in.
 *
 * @param[in]       snapshot_in
 *                  Snapshot to compute completeness for.
 *
 * @param[out]      completenessResults_out
 *                  One MapCompletenessResult per entry of \p snapshot_in.maps,
 *                  sorted by mapId (matching \p snapshot_in.maps's own
 *                  already-sorted order).
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus evaluateMapCompleteness(
    const SemanticGraphSnapshot        &snapshot_in,
    std::vector<MapCompletenessResult> &completenessResults_out);

/*!
 * @brief           Returns the fixed, snapshot-independent
 *                  capability/ownership table for all sixteen axiom
 *                  codes.
 *
 *                  A pure function of no input: the table describes
 *                  this evaluator's current implementation against the
 *                  current SemanticGraphSnapshot schema, not any one
 *                  evaluated snapshot.
 *
 * @param[out]      axiomCapabilityTable_out
 *                  Exactly sixteen entries, sorted by AxiomCode.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus computeAxiomCapabilityTable(
    std::vector<AxiomCapabilityEntry> &axiomCapabilityTable_out);

/*!
 * @brief           Return stable human-readable name for an axiom code.
 *
 * @param[in]       code_in
 *                  The axiom code enum value.
 *
 * @param[out]      axiomCodeName_out
 *                  Stable name string (e.g. "AX_FRAME_01", "AX_WALL_01"), or
 *                  "UNKNOWN_AXIOM_CODE" for a value outside the declared enum.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    axiomCodeName(AxiomCode code_in, std::string &axiomCodeName_out);

/*!
 * @brief           Return stable human-readable name for an axiom result.
 *
 * @param[in]       result_in
 *                  The result enum value.
 *
 * @param[out]      axiomResultName_out
 *                  Stable name string ("PASS", "FAIL", or "UNKNOWN"), or
 *                  "UNKNOWN_AXIOM_RESULT" for a value outside the declared
 *                  enum.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    axiomResultName(AxiomResult result_in, std::string &axiomResultName_out);

/*!
 * @brief           Return stable human-readable name for an axiom class.
 *
 * @param[in]       class_in
 *                  The class enum value.
 *
 * @param[out]      axiomClassName_out
 *                  Stable name string ("HARD" or "DERIVED"), or
 *                  "UNKNOWN_AXIOM_CLASS" for a value outside the declared enum.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    axiomClassName(AxiomClass class_in, std::string &axiomClassName_out);

/*!
 * @brief           Return stable human-readable name for a reason code.
 *
 * @param[in]       reason_in
 *                  The reason code enum value.
 *
 * @param[out]      reasonCodeName_out
 *                  Stable name string (e.g.
 *                  "FRAME_TRANSITION_EVALUATION_REQUIRED"), or
 *                  "UNKNOWN_REASON_CODE" for a value outside the declared enum.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    reasonCodeName(ReasonCode reason_in, std::string &reasonCodeName_out);

/*!
 * @brief           Return stable human-readable name for an entity kind.
 *
 * @param[in]       kind_in
 *                  The entity kind enum value.
 *
 * @param[out]      entityKindName_out
 *                  Stable name string ("ROOM", "WALL", "PASSAGE", or "FLOOR"),
 *                  or "UNKNOWN_ENTITY_KIND" for a value outside the declared
 *                  enum.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    entityKindName(EntityKind kind_in, std::string &entityKindName_out);

/*!
 * @brief           Return stable human-readable name for an unavailable reason.
 *
 * @param[in]       reason_in
 *                  The unavailable reason enum value.
 *
 * @param[out]      unavailableReasonName_out
 *                  Stable name string (e.g. "NONE",
 *                  "NOT_TRACKED_BY_CURRENT_SCHEMA"), or
 *                  "UNKNOWN_UNAVAILABLE_REASON" for a value outside the
 *                  declared enum.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    unavailableReasonName(UnavailableReason reason_in,
                          std::string      &unavailableReasonName_out);

/*!
 * @brief           Return stable human-readable name for a capability level.
 *
 * @param[in]       level_in
 *                  The capability level enum value.
 *
 * @param[out]      capabilityLevelName_out
 *                  Stable name string ("FULL", "PARTIAL", or "DEFERRED"), or
 *                  "UNKNOWN_CAPABILITY_LEVEL" for a value outside the declared
 *                  enum.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    capabilityLevelName(CapabilityLevel level_in,
                        std::string    &capabilityLevelName_out);

/*!
 * @brief           Return stable human-readable name for a missing-proof owner.
 *
 * @param[in]       owner_in
 *                  The missing-proof owner enum value.
 *
 * @param[out]      missingProofOwnerName_out
 *                  Stable name string (e.g. "NONE", "PHASE_2",
 *                  "SCOPE_DECISION_REQUIRED"), or "UNKNOWN_MISSING_PROOF_OWNER"
 *                  for a value outside the declared enum.
 *
 * @return          SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticAxiomEvaluatorStatus
    missingProofOwnerName(MissingProofOwner owner_in,
                          std::string      &missingProofOwnerName_out);
}

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_SEMANTIC_AXIOM_EVALUATOR_H \
        */
