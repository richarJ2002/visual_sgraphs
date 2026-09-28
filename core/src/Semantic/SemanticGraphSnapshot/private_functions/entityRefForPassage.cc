/*!
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
 * @file            entityRefForPassage.cc
 *
 * @brief           Implements entityRefForPassage(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticGraphSnapshot/private_functions.h"

#include "Map.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticGraphSnapshotStatus entityRefForPassage(Passage   *p_passage_in,
                                                EntityRef &entityRef_out)
{
    EntityRef reference;
    if (p_passage_in == nullptr)
    {
        entityRef_out = reference;
        return SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
    }
    int passage_inId{};
    if (p_passage_in->getId(passage_inId) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    reference.localId = passage_inId;
    bool passage_inIsBad{};
    if (p_passage_in->isBad(passage_inIsBad) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // isBad cannot fail; continue as before.
    }
    reference.isLive                    = !passage_inIsBad;
    reference.livenessUnavailableReason = UnavailableReason::NONE;

    core::Map *p_map = nullptr;
    if (p_passage_in->getMap(p_map) != PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // getMap cannot fail; continue as before.
    }
    if (p_map == nullptr)
    {
        reference.reason = UnavailableReason::ENTITY_HAS_NO_MAP;
        entityRef_out    = reference;
        return SemanticGraphSnapshotStatus::
            SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
    }
    int passage_inId2{};
    if (p_passage_in->getId(passage_inId2) !=
        PassageStatus::PASSAGE_STATUS_SUCCESS)
    {
        // getId cannot fail; continue as before.
    }
    EntityKey key2{};
    if (makeKey(EntityKind::PASSAGE, p_map->getId(), passage_inId2, key2) !=
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS)
    {
        // makeKey cannot fail; continue as before.
    }
    reference.key    = key2;
    reference.reason = UnavailableReason::NONE;
    entityRef_out    = reference;
    return SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
