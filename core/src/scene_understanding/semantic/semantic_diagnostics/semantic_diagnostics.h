/*!
 * @file            semantic_diagnostics.h
 *
 * @brief           Public interface of the semantic_diagnostics module.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_DIAGNOSTICS_SEMANTIC_DIAGNOSTICS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_DIAGNOSTICS_SEMANTIC_DIAGNOSTICS_H

#include "scene_understanding/semantic/semantic_diagnostics/objects/SemanticDiagnosticStateStruct.h"
#include "scene_understanding/semantic/semantic_diagnostics/objects/SemanticDiagnosticUpdateStruct.h"
#include "scene_understanding/semantic/semantic_diagnostics/objects/SemanticDiagnosticsStatusEnum.h"


/* Recovered public contracts from the pre-migration API. */
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include "scene_understanding/semantic/semantic_report_cache/semantic_report_cache.h"

namespace vs_graphs::core::semantic
{
/*!
 * @brief           Schema version of every SG_AXIOM/SG_VIOLATION JSON line this
 *                  module builds.
 */
inline constexpr int kSemanticDiagnosticSchemaVersion = 1;

/*!
 * @brief           A heartbeat is emitted when a caller's
 *                  SemanticDiagnosticState::cyclesSinceLastDiagnosticSummary
 *                  reaches this value with no discrete state change in between:
 *                  with a call on every semantic cycle and no changes, cycle 1
 *                  emits the initial summary, cycles 2 through
 *                  kDiagnosticHeartbeatCycles-1 emit nothing, and cycle
 *                  kDiagnosticHeartbeatCycles emits the heartbeat.
 */
inline constexpr std::uint64_t kDiagnosticHeartbeatCycles = 10U;

/*!
 * @brief           Maximum number of SG_VIOLATION detail objects returned for
 *                  one call; further transitions are counted in that call's
 *                  summary "emittedViolationCount"/"omittedViolationCount"
 *                  instead, so a pathological cycle with many simultaneous
 *                  transitions cannot produce an unbounded result.
 */
inline constexpr std::size_t kMaxViolationDetailsPerCycle = 50U;

/*!
 * @brief           Pure diagnostic-difference/JSON builder for one semantic
 *                  cycle's cache entry. Never performs I/O and never mutates \p
 *                  entry_in; the only mutation is to \p state_in_out, so that a
 *                  caller across many real cycles and a test across a crafted
 *                  sequence of entries observe identical, deterministic
 *                  behavior.
 *
 *                  Only \c AxiomResult::FAIL findings participate in
 *                  appeared/changed/resolved transition detection and
 *                  SG_VIOLATION detail output; PASS findings never appear as a
 *                  violation detail (including on the very first call, when \p
 *                  state_in_out is default-constructed and every current
 *                  finding would otherwise look "appeared"), and UNKNOWN
 *                  findings remain visible only through the summary's
 *                  unknownCount/completeness fields. A discrete state change is
 *                  any FAIL-finding transition or a canonical topology digest
 *                  change relative to \p state_in_out; a geometry-only change
 *                  (topology digest unchanged, full-geometry digest changed) is
 *                  carried in the next emitted summary's geometryRevision/
 *                  canonicalFullGeometryDigest fields but never itself forces
 *                  an emission or repeats prior violation details.
 *
 * @param[in]       entry_in
 *                  This cycle's copied cache entry.
 *
 * @param[out]      state_in_out
 *                  Caller-owned state from the previous call; updated in place
 *                  exactly when the returned update's \c emit is true.
 *
 * @param[out]      semanticDiagnosticUpdate_out
 *                  The bounded update to print (SG_AXIOM/SG_VIOLATION prefixes
 *                  are the caller's responsibility), or \c emit == false when
 *                  neither a discrete change nor a due heartbeat exists this
 *                  call.
 *
 * @return          SEMANTIC_DIAGNOSTICS_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticDiagnosticsStatus buildSemanticDiagnosticUpdate(
    const SemanticReportCacheEntry &entry_in,
    SemanticDiagnosticState        &state_in_out,
    SemanticDiagnosticUpdate       &semanticDiagnosticUpdate_out);
}

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_DIAGNOSTICS_SEMANTIC_DIAGNOSTICS_H \
        */
