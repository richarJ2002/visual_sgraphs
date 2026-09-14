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
 * @file            evaluateAxTxn01.cc
 *
 * @brief           Implements evaluateAxTxn01(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace ORB_SLAM3
{
namespace semantic
{

void evaluateAxTxn01(const SemanticGraphSnapshot &snapshot_in,
                     std::vector<Finding>        &findings_inout)
{
    (void)snapshot_in;
    findings_inout.push_back(
        makeFinding(AxiomCode::AX_TXN_01,
                    AxiomResult::UNKNOWN,
                    ReasonCode::TRANSACTION_EVALUATION_REQUIRES_TRANSITION,
                    {}));
}

} // namespace semantic
} // namespace ORB_SLAM3
