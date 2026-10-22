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
 * @file            AggregateAxiomResultStruct.h
 *
 * @brief           Declares one axiom code's aggregated outcome across every
 *                  Finding produced for it in one evaluation.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_OBJECTS_AGGREGATEAXIOMRESULT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_OBJECTS_AGGREGATEAXIOMRESULT_STRUCT_H

#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomClassEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomCodeEnum.h"
#include "scene_understanding/semantic/semantic_axiom_evaluator/objects/AxiomResultEnum.h"

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           One axiom code's aggregated outcome: exactly one instance of
 *                  this type exists per axiom code in AxiomEvaluationReport::
 *                  aggregates, computed by aggregateFindings() from
 *                  AxiomEvaluationReport::findings using FAIL > UNKNOWN > PASS
 *                  precedence (a single FAIL finding for a code makes its
 *                  aggregate FAIL; otherwise a single UNKNOWN finding makes it
 *                  UNKNOWN; only unanimous PASS, including a vacuously empty
 *                  finding set, yields PASS).
 */
struct AggregateAxiomResult
{
  public:
    /*!
     * @brief           Which of the sixteen axiom codes this aggregate is for.
     */
    AxiomCode axiomCode{AxiomCode::AX_FRAME_01};

    /*!
     * @brief           The aggregated tri-state outcome.
     */
    AxiomResult result{AxiomResult::UNKNOWN};

    /*!
     * @brief           Fixed "Class" column value for axiomCode.
     */
    AxiomClass classification{AxiomClass::HARD};

    /*!
     * @brief           How many AxiomEvaluationReport::findings entries have
     *                  this axiomCode; always >= 1 in this slice (every code
     *                  emits at least one placeholder or per-instance finding).
     */
    std::size_t contributingFindingCount{0U};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_AXIOM_EVALUATOR_OBJECTS_AGGREGATEAXIOMRESULT_STRUCT_H \
        */
