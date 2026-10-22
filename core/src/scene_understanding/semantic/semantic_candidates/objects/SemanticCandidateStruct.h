/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            SemanticCandidateStruct.h
 *
 * @brief           Declares SemanticCandidate, a proposed room pairing across
 * maps.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_STRUCT_H

#include "scene_understanding/semantic/semantic_candidates/objects/CandidateCueBreakdownStruct.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Proposed pairing of a room from one map with a room from
 *                  another map, scored by how alike their saved context is.
 *                  A proposal only; it is verified later.
 */
struct SemanticCandidate
{
    /*!
     * @brief           Atlas map id of the first room's map.
     */
    long unsigned int     mapAId{0U};
    /*!
     * @brief           Id of the first room.
     */
    int                   roomAId{0};
    /*!
     * @brief           Atlas map id of the second room's map.
     */
    long unsigned int     mapBId{0U};
    /*!
     * @brief           Id of the second room.
     */
    int                   roomBId{0};
    /*!
     * @brief           Combined cue distance (weightedNumerator over
     *                  weightDenominator); smaller means more alike.
     */
    double                distance{0.0};
    /*!
     * @brief           Individual cue distances and weights behind distance.
     */
    CandidateCueBreakdown cues;
    /*!
     * @brief           True when either room has fewer than two valid wall
     *                  normals or more than half of its walls lack valid
     *                  bounds.
     */
    bool                  hasLowConfidence{false};
    /*!
     * @brief           True when distance is within ambiguityMargin of the best
     *                  candidate's distance (the best one included).
     */
    bool                  isAmbiguous{false};
    /*!
     * @brief           True when both rooms met the minimum evidence (two valid
     *                  walls, or one wall and one passage). Pairs that fail are
     *                  not produced, so this is true for every generated
     *                  candidate.
     */
    bool                  isMinimumEvidenceSatisfied{false};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_STRUCT_H \
        */
