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
 * @file            SemanticCandidateConfigRejectionReasonEnum.h
 *
 * @brief           Declares SemanticCandidateConfigRejectionReason, why a
 * configuration was rejected.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_CONFIG_REJECTION_REASON_ENUM_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_CONFIG_REJECTION_REASON_ENUM_H

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Why validateConfig() rejected a SemanticCandidateConfig;
 *                  NONE means it is valid.
 */
enum class SemanticCandidateConfigRejectionReason
{
    NONE,
    TOP_K_ZERO,
    PAIR_CAP_ZERO,
    TOPOLOGY_CAP_ZERO,
    GLOBAL_FALLBACK_CAP_ZERO,
    TOP_K_EXCEEDS_PAIR_CAP,
    GLOBAL_FALLBACK_EXCEEDS_PAIR_CAP,
    DESCRIPTOR_CAP_ZERO,
    TOPO_REFINEMENT_ITERS_ZERO,
    NONFINITE_WEIGHT,
    NEGATIVE_WEIGHT,
    WEIGHT_SUM_OVERFLOW,
    ALL_WEIGHTS_ZERO,
    NONFINITE_PENALTY,
    NEGATIVE_PENALTY,
    NONFINITE_AMBIGUITY_MARGIN,
    NEGATIVE_AMBIGUITY_MARGIN,
    NONFINITE_ANGLE_TOLERANCE,
    NEGATIVE_ANGLE_TOLERANCE,
    NONFINITE_RUNTIME_BUDGET,
    NEGATIVE_RUNTIME_BUDGET
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_CONFIG_REJECTION_REASON_ENUM_H \
        */
