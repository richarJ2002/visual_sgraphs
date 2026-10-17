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
 * @file            CandidateCueBreakdownStruct.h
 *
 * @brief           Declares CandidateCueBreakdown, the per-cue distances behind
 * one candidate distance.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_CANDIDATE_CUE_BREAKDOWN_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_CANDIDATE_CUE_BREAKDOWN_STRUCT_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Per-cue distances between two rooms and the weights that
 *                  combined them into one candidate distance.
 */
struct CandidateCueBreakdown
{
    /*!
     * @brief           Distance between the rooms' wall-angle descriptors
     *                  (mean absolute difference, radians).
     */
    double angleDistance{0.0};
    /*!
     * @brief           Distance between the rooms' wall-extent descriptors
     *                  (extents divided by each room's median extent, so
     *                  unitless).
     */
    double extentDistance{0.0};
    /*!
     * @brief           Distance between the rooms' passage width/height
     *                  descriptors (normalised like the extents, unitless).
     */
    double apertureDistance{0.0};
    /*!
     * @brief           Fraction of mismatching entries between the rooms'
     *                  passage-graph signatures, in [0, 1]; 0 when
     *                  isTopologyAvailable is false.
     */
    double topologyDistance{0.0};
    /*!
     * @brief           True when both rooms produced a topology signature, so
     *                  topologyDistance took part in the distance.
     */
    bool   isTopologyAvailable{false};
    /*!
     * @brief           Reserved for a truncation flag; nothing sets it yet, so
     *                  it is always false.
     */
    bool   isTruncated{false};
    /*!
     * @brief           Reserved for a runtime-budget flag; nothing sets it yet,
     *                  so it is always false.
     */
    bool   isRuntimeBudgetExceeded{false};
    /*!
     * @brief           Sum of each used cue's weight times its distance.
     */
    double weightedNumerator{0.0};
    /*!
     * @brief           Sum of the weights of the cues that were used; the
     *                  candidate distance is weightedNumerator divided by it.
     */
    double weightDenominator{0.0};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_CANDIDATE_CUE_BREAKDOWN_STRUCT_H \
        */
