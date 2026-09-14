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
 * @file            EntityKey.h
 *
 * @brief           Declares the stable, orderable identity for one snapshot
 *                  entity, and its comparison operators.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_ENTITY_KEY_H
#define SEMANTIC_GRAPH_SNAPSHOT_ENTITY_KEY_H

#include "Semantic/SemanticGraphSnapshot/objects/EntityKind.h"

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Stable, orderable identity for one snapshot entity.
 *
 *              Relationships in this snapshot always use this key, never a
 *              bare local ID, because local IDs may repeat across different
 *              maps (Section 6.3 of the semantic-axiom-reliability plan).
 *              This is the *containing* map's identity -- the map whose
 *              enumeration produced this record -- which may differ from the
 *              entity's own declared map; see the record types' declaredMapId
 *              field for that separate fact.
 */
struct EntityKey
{
  public:
    /*! @brief Which kind of graph entity entityId names. */
    EntityKind kind{EntityKind::ROOM};

    /*! @brief Atlas::Map::GetId() of the map this record was enumerated
     *  from. */
    long unsigned int mapId{0U};

    /*! @brief The entity's local id (Room::getId(), Plane::getId(),
     *  Passage::getId(), or Floor::getId()), unique only within mapId. */
    int entityId{0};
};

/*! @brief True when kind, mapId, and entityId all match. */
bool operator==(const EntityKey &lhs_in, const EntityKey &rhs_in);

/*! @brief Inverse of operator==(). */
bool operator!=(const EntityKey &lhs_in, const EntityKey &rhs_in);

/*! @brief Total order by (kind, mapId, entityId), used to sort every
 *  snapshot record and relationship collection deterministically. */
bool operator<(const EntityKey &lhs_in, const EntityKey &rhs_in);

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_GRAPH_SNAPSHOT_ENTITY_KEY_H
