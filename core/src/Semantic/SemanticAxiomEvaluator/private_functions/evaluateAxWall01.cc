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
 * @file            evaluateAxWall01.cc
 *
 * @brief           Implements evaluateAxWall01(), declared in
 *                  private_functions.h. The per-wall logic lives in
 *                  evaluateOneWall.cc (CPP_CODING_STANDARD.md Section 5.4:
 *                  one ordinary function per .cc).
 *
 *                  2026-09-07 proof-correctness repair: retired
 *                  (non-live) WallRecords are skipped -- the contract
 *                  applies to live committed walls; a live room still
 *                  referencing a retired wall is a distinct, observable
 *                  contradiction caught by AX-BOUND-01's own wall-evidence
 *                  check, not by re-evaluating the dead wall here.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateAxWall01(const SemanticGraphSnapshot &snapshot_in,
                      std::vector<Finding>        &findings_inout)
{
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        for (const WallRecord &wall : mapSnapshot.walls)
        {
            if (!wall.isLive)
            {
                continue;
            }
            evaluateOneWall(wall, snapshot_in, findings_inout);
        }
    }
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
