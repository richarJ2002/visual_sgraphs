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
 * @file            evaluateAxPass02.cc
 *
 * @brief           Implements evaluateAxPass02(), declared in
 *                  private_functions.h. The per-passage logic lives in
 *                  evaluateOnePassageCardinality.cc (CPP_CODING_STANDARD.md
 *                  Section 5.4: one ordinary function per .cc).
 *
 *                  Also runs evaluateRoomMalformedPassageReferences()
 *                  once per map, independent of any specific passage (see
 *                  that function's own Doxygen for why this evidence
 *                  cannot be attributed to one passage).
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateAxPass02(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout)
{
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        evaluateRoomMalformedPassageReferences(mapSnapshot, findings_inout);
        for (const PassageRecord &passage : mapSnapshot.passages)
        {
            if (!passage.isLive)
            {
                continue;
            }
            evaluateOnePassageCardinality(passage, snapshot_in, findings_inout);
        }
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
