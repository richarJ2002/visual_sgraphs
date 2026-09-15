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
 * @file            AxiomEvaluationReport.h
 *
 * @brief           Declares the complete, immutable output of one
 *                  evaluateState()/evaluateTransition() call.
 */

#ifndef SEMANTIC_AXIOM_EVALUATOR_AXIOM_EVALUATION_REPORT_H
#define SEMANTIC_AXIOM_EVALUATOR_AXIOM_EVALUATION_REPORT_H

#include <vector>

#include "Semantic/SemanticAxiomEvaluator/objects/AggregateAxiomResult.h"
#include "Semantic/SemanticAxiomEvaluator/objects/Finding.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief       Complete, immutable, value-only output of one pure
 *              evaluation call.
 *
 *              Both members are sorted, deterministic, and independent of
 *              the input snapshot's own container/iteration order:
 *              \c findings by Finding::id (ties cannot occur -- see
 *              makeFinding.cc); \c aggregates by AxiomCode, with exactly one
 *              entry for every one of the sixteen Section-5 codes,
 *              regardless of how many (if any) findings contributed to it.
 */
struct AxiomEvaluationReport
{
  public:
    /*! @brief Every raw finding produced by this evaluation, sorted by
     *  Finding::id. */
    std::vector<Finding> findings;

    /*! @brief Exactly sixteen entries, one per Section-5 axiom code,
     *  sorted by AxiomCode. */
    std::vector<AggregateAxiomResult> aggregates;
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_AXIOM_EVALUATOR_AXIOM_EVALUATION_REPORT_H
