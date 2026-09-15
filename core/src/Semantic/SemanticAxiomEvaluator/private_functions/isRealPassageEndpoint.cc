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
 * @file            isRealPassageEndpoint.cc
 *
 * @brief           Implements isRealPassageEndpoint(), declared in
 *                  private_functions.h.
 *
 *                  Consolidates what was, before the 2026-09-07
 *                  proof-correctness repair, four independent copies of the
 *                  identical `isFoundInSnapshot && isLive &&
 *                  isConfirmedRoomVariant` predicate scattered across
 *                  evaluateAxPass02.cc, evaluateAxPass04.cc,
 *                  evaluatePassageFloorAgreement.cc, and
 *                  computeConservativeMapCompleteness.cc.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isRealPassageEndpoint(const ResolvedRoomEndpoint &endpoint_in)
{
    return endpoint_in.isFoundInSnapshot && endpoint_in.isLive &&
           endpoint_in.isConfirmedRoomVariant &&
           !endpoint_in.isDuplicateIdentity;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
