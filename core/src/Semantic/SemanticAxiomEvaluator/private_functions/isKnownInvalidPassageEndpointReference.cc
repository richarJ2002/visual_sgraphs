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
 * @file            isKnownInvalidPassageEndpointReference.cc
 *
 * @brief           Implements isKnownInvalidPassageEndpointReference(),
 *                  declared in private_functions.h.
 *
 *                  Deliberately excludes ResolvedRoomEndpoint::isCrossMap:
 *                  AX-PASS-02 (cardinality) and AX-PASS-04 (map/floor
 *                  agreement) each already validate cross-map placement
 *                  explicitly with their own dedicated, more specific
 *                  reason code, so folding it into this generic predicate
 *                  would only replace that specific diagnosis with a vaguer
 *                  one wherever this predicate is checked first.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

bool isKnownInvalidPassageEndpointReference(
    const ResolvedRoomEndpoint &endpoint_in)
{
    if (!endpoint_in.referencePresent)
    {
        /* Either an ordinary absent reference, or a non-null-but-mapless
         * reference (referenceUnresolvable) -- both handled by dedicated
         * callers via ResolvedRoomEndpoint::referenceUnresolvable, which
         * this predicate deliberately treats as "known invalid" too, since
         * an attempted-but-mapless reference is itself an observable
         * contradiction, not an ordinary absence. */
        return endpoint_in.referenceUnresolvable;
    }
    return endpoint_in.isWrongKind || endpoint_in.isDuplicateIdentity ||
           endpoint_in.isReasonInconsistent ||
           endpoint_in.isContainingMapAmbiguous ||
           (endpoint_in.isFoundInSnapshot &&
            endpoint_in.isTargetDeclaredMapMismatch) ||
           (endpoint_in.isLiveAvailable && !endpoint_in.isLive);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
