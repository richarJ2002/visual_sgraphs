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
 * @file            MergePairingStruct.h
 *
 * @brief           Declares MergePairing, pairs and contradictions.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_MERGE_PAIRING_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_MERGE_PAIRING_STRUCT_H

#include "scene_understanding/semantic/semantic_verify/objects/MergePairStruct.h"

#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{
/*!
 * @brief           Result of pairing the elements of two maps by geometry.
 *                  Elements without a counterpart appear in neither list: an
 *                  element seen in only one map gives no answer.
 */
struct MergePairing
{
  public:
    /*!
     * @brief           Element pairs whose geometry agrees. An element may
     *                  appear in more than one pair; the caller decides
     *                  whether that is a contradiction.
     */
    std::vector<MergePair> pairs;
    /*!
     * @brief           Element pairs that occupy the same place but disagree,
     *                  for example two walls that overlap but lie at different
     *                  offsets while neither has an agreeing partner.
     */
    std::vector<MergePair> contradictions;
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_VERIFY_OBJECTS_MERGE_PAIRING_STRUCT_H \
        */
