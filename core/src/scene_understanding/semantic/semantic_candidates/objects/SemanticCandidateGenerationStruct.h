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
 * @file            SemanticCandidateGenerationStruct.h
 *
 * @brief           Declares SemanticCandidateGeneration, the output of
 * generation.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_GENERATION_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_GENERATION_STRUCT_H

#include "scene_understanding/semantic/semantic_candidates/objects/SemanticCandidateConfigRejectionReasonEnum.h"
#include "scene_understanding/semantic/semantic_candidates/objects/SemanticCandidateStruct.h"

#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Output of candidate generation: the candidates, or the
 *                  reason the configuration was rejected.
 */
struct SemanticCandidateGeneration
{
    /*!
     * @brief           Candidates sorted by ascending distance, at most topK;
     *                  empty when the configuration was rejected.
     */
    std::vector<SemanticCandidate>         candidates;
    /*!
     * @brief           NONE when the configuration was valid, otherwise the
     *                  first failed check.
     */
    SemanticCandidateConfigRejectionReason rejectionReason{
        SemanticCandidateConfigRejectionReason::NONE};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATE_GENERATION_STRUCT_H \
        */
