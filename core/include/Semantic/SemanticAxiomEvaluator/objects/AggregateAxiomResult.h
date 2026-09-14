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
 * @file            AggregateAxiomResult.h
 *
 * @brief           Declares one axiom code's aggregated outcome across every
 *                   Finding produced for it in one evaluation.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_AGGREGATE_AXIOM_RESULT_H
#define SEMANTIC_AXIOM_EVALUATOR_AGGREGATE_AXIOM_RESULT_H

#include <cstddef>

#include "Semantic/SemanticAxiomEvaluator/objects/AxiomClass.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomCode.h"
#include "Semantic/SemanticAxiomEvaluator/objects/AxiomResult.h"

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       One axiom code's aggregated outcome: exactly one instance of
 *              this type exists per axiom code in AxiomEvaluationReport::
 *              aggregates, computed by aggregateFindings() from
 *              AxiomEvaluationReport::findings using FAIL > UNKNOWN > PASS
 *              precedence (a single FAIL finding for a code makes its
 *              aggregate FAIL; otherwise a single UNKNOWN finding makes it
 *              UNKNOWN; only unanimous PASS, including a vacuously empty
 *              finding set, yields PASS).
 */
struct AggregateAxiomResult
{
  public:
    /*! @brief Which of the sixteen axiom codes this aggregate is for. */
    AxiomCode axiomCode{AxiomCode::AX_FRAME_01};

    /*! @brief The aggregated tri-state outcome. */
    AxiomResult result{AxiomResult::UNKNOWN};

    /*! @brief Section 5's fixed "Class" column value for axiomCode. */
    AxiomClass classification{AxiomClass::HARD};

    /*! @brief How many AxiomEvaluationReport::findings entries have this
     *  axiomCode; always >= 1 in this slice (every code emits at least one
     *  placeholder or per-instance finding). */
    std::size_t contributingFindingCount{0U};
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_AXIOM_EVALUATOR_AGGREGATE_AXIOM_RESULT_H
