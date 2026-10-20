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
 * @file            SemanticCandidateConfigStruct.h
 *
 * @brief           Declares SemanticCandidateConfig, the limits of candidate
 * generation.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_CONFIG_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_CONFIG_STRUCT_H

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Limits, weights and thresholds of candidate generation.
 *                  validateConfig() rejects invalid values.
 */
struct SemanticCandidateConfig
{
    /*!
     * @brief           Maximum number of candidates returned, best first.
     */
    std::size_t  topK{10U};
    /*!
     * @brief           Maximum number of room snapshots taken from the history
     *                  (lowest map and room ids first) and of room pairs scored
     *                  in the adjacency-prioritised pass.
     */
    std::size_t  candidatePairCap{1000U};
    /*!
     * @brief           Maximum number of room pairs scored in the global
     *                  fallback pass.
     */
    std::size_t  globalFallbackCap{1000U};
    /*!
     * @brief           Maximum nodes of a room's passage graph; a room needing
     *                  more gets no topology signature.
     */
    std::size_t  topologyNodesCap{128U};
    /*!
     * @brief           Weight of the angle cue in the combined distance.
     */
    double       weightAngle{1.0};
    /*!
     * @brief           Weight of the extent cue in the combined distance.
     */
    double       weightExtent{1.0};
    /*!
     * @brief           Weight of the aperture cue in the combined distance.
     */
    double       weightAperture{1.0};
    /*!
     * @brief           Weight of the topology cue in the combined distance.
     */
    double       weightTopology{1.0};
    /*!
     * @brief           Value used for angle elements one room lacks when the
     *                  shorter descriptor is padded, radians.
     */
    double       angleMissingPenalty{1.0};
    /*!
     * @brief           Value used for extent elements one room lacks when the
     *                  shorter descriptor is padded (normalised, unitless).
     */
    double       extentMissingPenalty{1.0};
    /*!
     * @brief           Value used for both width and height of aperture pairs
     *                  one room lacks when the shorter descriptor is padded
     *                  (normalised, unitless).
     */
    double       apertureMissingPenalty{1.0};
    /*!
     * @brief           Distance margin above the best candidate within which a
     *                  candidate is marked ambiguous.
     */
    double       ambiguityMargin{0.05};
    /*!
     * @brief           Angles at or below this are treated as exactly parallel
     *                  (0), radians.
     */
    double       angleTolerance_rad{1e-9};
    /*!
     * @brief           Runtime budget, milliseconds. Validated but not read by
     *                  generation, which never consults a clock.
     */
    double       runtimeBudget_ms{0.0};
    /*!
     * @brief           Maximum number of elements in each room descriptor.
     */
    std::size_t  descriptorElementsCap{4096U};
    /*!
     * @brief           Number of neighbour-label refinement rounds used to
     *                  build the topology signature.
     */
    unsigned int topoRefinementIters{3U};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_CONFIG_STRUCT_H \
        */
