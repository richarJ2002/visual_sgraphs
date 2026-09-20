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
 *                  SemanticCanonicalSerialization module (CPP_CODING_
 *                  STANDARD.md Section 5.4).
 *
 *                  Every function here is a pure projection: it never
 *                  mutates its input, reads a clock, or depends on the
 *                  input's own container/pointer/iteration order -- each
 *                  serialized collection is sorted from a private copy (see
 *                  private_functions.h), never the caller's original
 *                  container, so two logically-identical inputs built with
 *                  permuted insertion order serialize byte-identically.
 *                  Every enumeration field is emitted as its underlying
 *                  integer value (this slice does not add a human-readable
 *                  name projection); every floating-point field uses
 *                  serializeDouble()'s explicit NaN/Infinity/-Infinity
 *                  string sentinels and nlohmann::json's own
 *                  locale-independent, sign-preserving numeric formatting
 *                  for every finite value, including signed zero.
 */

#ifndef SEMANTIC_CANONICAL_SERIALIZATION_PUBLIC_FUNCTIONS_H
#define SEMANTIC_CANONICAL_SERIALIZATION_PUBLIC_FUNCTIONS_H

#include <vector>

#include "Thirdparty/nlohmann/json.hpp"

#include "Semantic/SemanticAxiomEvaluator/objects.h"
#include "Semantic/SemanticGraphSnapshot/objects/SemanticGraphSnapshot.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*! @brief Schema version of serializeSnapshotTopologyOnly()'s and
 *  serializeSnapshotFullGeometry()'s "schema" field. Increment only when
 *  either function's own output shape changes incompatibly. */
inline constexpr int SEMANTIC_SNAPSHOT_SCHEMA_VERSION = 1;

/*! @brief Schema version of serializeEvaluationReport()'s "schema"
 *  field. */
inline constexpr int SEMANTIC_EVALUATION_REPORT_SCHEMA_VERSION = 1;

/*! @brief Schema version of serializeMapCompletenessResults()'s "schema"
 *  field. */
inline constexpr int SEMANTIC_COMPLETENESS_SCHEMA_VERSION = 1;

/*!
 * @brief       Canonically serializes \p snapshot_in without any geometric
 *              field (equations, centroids, extents, boundary corners,
 *              observation gaps, plane identity): identity, lifecycle,
 *              enums, and relationships only.
 *
 * @param[in]   snapshot_in     Snapshot to serialize; never mutated.
 *
 * @return      A JSON object with a top-level "schema" version field.
 */
nlohmann::json
    serializeSnapshotTopologyOnly(const SemanticGraphSnapshot &snapshot_in);

/*!
 * @brief       Canonically serializes \p snapshot_in including every
 *              geometric field, in addition to everything
 *              serializeSnapshotTopologyOnly() includes.
 *
 * @param[in]   snapshot_in     Snapshot to serialize; never mutated.
 *
 * @return      A JSON object with a top-level "schema" version field.
 */
nlohmann::json
    serializeSnapshotFullGeometry(const SemanticGraphSnapshot &snapshot_in);

/*!
 * @brief       Canonically serializes a complete AxiomEvaluationReport.
 *
 * @param[in]   report_in       Report to serialize; never mutated.
 *
 * @return      A JSON object with a top-level "schema" version field.
 */
nlohmann::json
    serializeEvaluationReport(const AxiomEvaluationReport &report_in);

/*!
 * @brief       Canonically serializes a complete evaluateMapCompleteness()
 *              result list.
 *
 * @param[in]   results_in      Results to serialize; never mutated.
 *
 * @return      A JSON object with a top-level "schema" version field.
 */
nlohmann::json serializeMapCompletenessResults(
    const std::vector<MapCompletenessResult> &results_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_CANONICAL_SERIALIZATION_PUBLIC_FUNCTIONS_H
