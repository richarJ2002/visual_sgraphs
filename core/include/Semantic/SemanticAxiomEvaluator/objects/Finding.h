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
 * @file            Finding.h
 *
 * @brief           Declares one axiom-evaluation finding: a single
 *                  PASS/FAIL/UNKNOWN outcome for one axiom code against one
 *                  observable clause and its involved entities.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_FINDING_H
#define SEMANTIC_AXIOM_EVALUATOR_FINDING_H

#include <string>
#include <vector>

#include "Semantic/SemanticGraphSnapshot/objects/EntityKey.h"

#include "Semantic/SemanticAxiomEvaluator/objects/AxiomClass.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomCode.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomResult.h"
#include "Semantic/SemanticAxiomEvaluator/objects/FindingEvidence.h"
#include "Semantic/SemanticAxiomEvaluator/objects/ReasonCode.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           One axiom-evaluation finding.
 *
 *                  \c id is deterministically derived from exactly
 *                  \c axiomCode, \c reasonCode, and \c involvedKeys (already
 *                  sorted ascending by EntityKey when this Finding is
 *                  constructed via makeFinding(), the only production
 *                  constructor) -- see makeFinding.cc for the exact, stable
 *                  textual encoding. It never contains prose, wall-clock time,
 *                  a pointer address, an unordered-iteration artifact, or an
 *                  unstable hash.
 */
struct Finding
{
  public:
    /*!
     * @brief           Deterministic identity string; see makeFinding.cc. Two
     *                  findings with the same axiomCode, reasonCode, and
     *                  involvedKeys always have the same id, regardless of when
     *                  or in what order they were produced.
     */
    std::string id;

    /*!
     * @brief           Which of the sixteen Section-5 axiom codes this finding
     *                  is for.
     */
    AxiomCode axiomCode{AxiomCode::AX_FRAME_01};

    /*!
     * @brief           The observed outcome for the specific clause reasonCode
     *                  names.
     */
    AxiomResult result{AxiomResult::UNKNOWN};

    /*!
     * @brief           Severity/classification of axiomCode (HARD for every
     *                  code except the DERIVED AX-COMP-01); duplicated here
     *                  (rather than looked up separately) so a Finding is
     *                  self-describing.
     */
    AxiomClass classification{AxiomClass::HARD};

    /*!
     * @brief           Stable reason identifying which observable clause
     *                  produced result.
     */
    ReasonCode reasonCode{ReasonCode::FRAME_TRANSITION_EVALUATION_REQUIRED};

    /*!
     * @brief           Every entity this finding is about, sorted ascending by
     *                  EntityKey with no duplicates. May be empty for a finding
     *                  that is inherently snapshot-scoped rather than
     *                  entity-specific (e.g. a DEFERRED axiom's placeholder).
     */
    std::vector<EntityKey> involvedKeys;

    /*!
     * @brief           Bounded, typed evidence for result, interpreted per
     *                  reasonCode.
     */
    FindingEvidence evidence;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_FINDING_H
