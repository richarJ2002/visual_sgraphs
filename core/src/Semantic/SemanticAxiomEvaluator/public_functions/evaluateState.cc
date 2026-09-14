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
 * @file            evaluateState.cc
 *
 * @brief           Implements evaluateState(), declared in
 *                  public_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/public_functions.h"

#include <utility>

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

AxiomEvaluationReport evaluateState(const SemanticGraphSnapshot &snapshot_in)
{
    std::vector<Finding> findings;

    evaluateAxFrame01(snapshot_in, findings);
    evaluateAxWall01(snapshot_in, findings);
    evaluateAxWall02(snapshot_in, findings);
    evaluateAxWall03(snapshot_in, findings);
    evaluateAxPass01(snapshot_in, findings);
    evaluateAxPass02(snapshot_in, findings);
    evaluateAxPass03(snapshot_in, findings);
    evaluateAxPass04(snapshot_in, findings);
    evaluateAxRoom01(snapshot_in, findings);
    evaluateAxRoom02(snapshot_in, findings);
    evaluateAxBound01(snapshot_in, findings);
    evaluateAxFloor01(snapshot_in, findings);
    evaluateAxLife01(snapshot_in, findings);
    evaluateAxTxn01(snapshot_in, findings);

    /* AX-COMP-01 is derived from the same completeness calculation
     * evaluateMapCompleteness() exposes on its own; computed here, not
     * inside computeConservativeMapCompleteness(), to keep that function
     * free of any dependency back on this one (see its own Doxygen). */
    const std::vector<MapCompletenessResult> completeness =
        evaluateMapCompleteness(snapshot_in);
    evaluateAxComp01(completeness, findings);

    evaluateAxMerge01(snapshot_in, findings);

    sortFindings(findings);

    AxiomEvaluationReport report;
    report.aggregates = aggregateFindings(findings);
    report.findings   = std::move(findings);
    return report;
}

} // namespace semantic
} // namespace ORB_SLAM3
