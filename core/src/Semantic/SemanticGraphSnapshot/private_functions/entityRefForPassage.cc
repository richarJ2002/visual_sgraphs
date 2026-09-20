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

EntityRef entityRefForPassage(Passage *p_passage_in)
{
    EntityRef ref;
    if (p_passage_in == nullptr)
    {
        return ref;
    }
    ref.localId                   = p_passage_in->getId();
    ref.isLive                    = !p_passage_in->isBad();
    ref.livenessUnavailableReason = UnavailableReason::NONE;

    core::Map *p_map = p_passage_in->getMap();
    if (p_map == nullptr)
    {
        ref.reason = UnavailableReason::ENTITY_HAS_NO_MAP;
        return ref;
    }
    ref.key =
        makeKey(EntityKind::PASSAGE, p_map->getId(), p_passage_in->getId());
    ref.reason = UnavailableReason::NONE;
    return ref;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
