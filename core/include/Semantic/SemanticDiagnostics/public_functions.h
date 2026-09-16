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
 * @file            public_functions.h
 *
 * @brief           Declares the public entry point of the
 *                  SemanticDiagnostics module (CPP_CODING_STANDARD.md
 *                  Section 5.4): a pure builder of bounded, transition-aware
 *                  SG_AXIOM/SG_VIOLATION diagnostic JSON, extracted from
 *                  SemanticsManager::logSemanticDiagnostics() so its
 *                  determinism, transition, cap, and cadence rules are
 *                  directly testable without SemanticsManager, Atlas, or
 *                  I/O of any kind.
 */

#ifndef SEMANTIC_DIAGNOSTICS_PUBLIC_FUNCTIONS_H
#define SEMANTIC_DIAGNOSTICS_PUBLIC_FUNCTIONS_H

#include <cstddef>
#include <cstdint>

#include "Semantic/SemanticDiagnostics/objects.h"
#include "Semantic/SemanticReportCache/objects/SemanticReportCacheEntry.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*! @brief Schema version of every SG_AXIOM/SG_VIOLATION JSON line this
 *  module builds. */
inline constexpr int kSemanticDiagnosticSchemaVersion = 1;

/*! @brief A heartbeat is emitted when a caller's
 *  SemanticDiagnosticState::cyclesSinceLastDiagnosticSummary reaches this
 *  value with no discrete state change in between: with a call on every
 *  semantic cycle and no changes, cycle 1 emits the initial summary, cycles
 *  2 through kDiagnosticHeartbeatCycles-1 emit nothing, and cycle
 *  kDiagnosticHeartbeatCycles emits the heartbeat. */
inline constexpr std::uint64_t kDiagnosticHeartbeatCycles = 10U;

/*! @brief Maximum number of SG_VIOLATION detail objects returned for one
 *  call; further transitions are counted in that call's summary
 *  "emittedViolationCount"/"omittedViolationCount" instead, so a
 *  pathological cycle with many simultaneous transitions cannot produce an
 *  unbounded result. */
inline constexpr std::size_t kMaxViolationDetailsPerCycle = 50U;

/*!
 * @brief       Pure diagnostic-difference/JSON builder for one semantic
 *              cycle's cache entry. Never performs I/O and never mutates \p
 *              entry_in; the only mutation is to \p state_in_out, so that a
 *              caller across many real cycles and a test across a crafted
 *              sequence of entries observe identical, deterministic
 *              behavior.
 *
 *              Only \c AxiomResult::FAIL findings participate in
 *              appeared/changed/resolved transition detection and
 *              SG_VIOLATION detail output; PASS findings never appear as a
 *              violation detail (including on the very first call, when \p
 *              state_in_out is default-constructed and every current
 *              finding would otherwise look "appeared"), and UNKNOWN
 *              findings remain visible only through the summary's
 *              unknownCount/completeness fields. A discrete state change is
 *              any FAIL-finding transition or a canonical topology digest
 *              change relative to \p state_in_out; a geometry-only change
 *              (topology digest unchanged, full-geometry digest changed) is
 *              carried in the next emitted summary's geometryRevision/
 *              canonicalFullGeometryDigest fields but never itself forces
 *              an emission or repeats prior violation details.
 *
 * @param[in]       entry_in        This cycle's copied cache entry.
 * @param[in,out]   state_in_out    Caller-owned state from the previous
 *                  call; updated in place exactly when the returned
 *                  update's \c emit is true.
 *
 * @return      The bounded update to print (SG_AXIOM/SG_VIOLATION
 *              prefixes are the caller's responsibility), or \c emit ==
 *              false when neither a discrete change nor a due heartbeat
 *              exists this call.
 */
SemanticDiagnosticUpdate
    buildSemanticDiagnosticUpdate(const SemanticReportCacheEntry &entry_in,
                                  SemanticDiagnosticState        &state_in_out);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_DIAGNOSTICS_PUBLIC_FUNCTIONS_H
