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
 * @file            evaluateOnePassageProvenance.cc
 *
 * @brief           Implements evaluateOnePassageProvenance(), declared in
 *                  private_functions.h.
 *
 *                  2026-09-07 second proof-closure repair: extracted from
 *                  evaluateAxPass01.cc so evaluateAxPass01() and
 *                  computeConservativeMapCompleteness() share the identical
 *                  AX-PASS-01 leaf rather than the completeness path
 *                  re-deriving its own hand-written passable() check.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateOnePassageProvenance(const PassageRecord  &passage_in,
                                  std::vector<Finding> &findings_inout)
{
    if (!passage_in.passable)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_PASS_01,
                        AxiomResult::FAIL,
                        ReasonCode::PASSAGE_PROVENANCE_NOT_PASSABLE,
                        {passage_in.key}));
        return;
    }
    findings_inout.push_back(
        makeFinding(AxiomCode::AX_PASS_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::PASSAGE_PROVENANCE_FULL_CHAIN_UNVERIFIABLE,
                    {passage_in.key}));
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
