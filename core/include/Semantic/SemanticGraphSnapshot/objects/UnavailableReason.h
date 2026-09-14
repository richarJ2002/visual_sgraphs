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
 * @file            UnavailableReason.h
 *
 * @brief           Declares why an EntityRef, RawPlaneRef, or dropped
 *                  relationship-vector entry carries no valid value, when the
 *                  absence itself is not ordinary.
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_UNAVAILABLE_REASON_H
#define SEMANTIC_GRAPH_SNAPSHOT_UNAVAILABLE_REASON_H

#include <cstdint>

namespace ORB_SLAM3
{
namespace semantic
{
/*!
 * @brief       Explains why a reference field carries no valid value.
 *
 *              A plain "no such reference at all" case (e.g. a room with no
 *              floor yet) is represented by UnavailableReason::NULL_REFERENCE,
 *              which is also every reference field's default value -- see
 *              EntityRef.h and RawPlaneRef.h for why NONE is never a valid
 *              default. Consumers interpret which values are ordinary versus
 *              a hard violation per field; this snapshot only reports what
 *              capture actually observed.
 *
 *              Whether a *present* reference is additionally bad or cross-map
 *              is an evaluator judgement (P1.2) made from the plain
 *              EntityKey/liveness data this snapshot already carries -- it is
 *              not decided at capture time and is not a value of this enum.
 */
enum class UnavailableReason : std::uint8_t
{
    /*! @brief A value is present; this reason is unused as a live value,
     *  though it remains the type's required non-default-safe sentinel --
     *  see EntityRef.h/RawPlaneRef.h. */
    NONE = 0U,

    /*! @brief The underlying model pointer was nullptr at capture time. */
    NULL_REFERENCE = 1U,

    /*! @brief The referenced object was non-null, but its own GetMap()/
     *  getMap() returned nullptr, so no valid map-qualified key could be
     *  formed for it (confirmed reachable: SemanticFixtures.h documents
     *  "some tests intentionally exercise unregistered planes" that are
     *  never given a map). */
    ENTITY_HAS_NO_MAP = 2U,

    /*! @brief No field on the current model class records this. */
    NOT_TRACKED_BY_CURRENT_SCHEMA = 3U,

    /*! @brief No public Atlas/Map API exposes this state at all. */
    NOT_EXPOSED_BY_CURRENT_API = 4U,

    /*! @brief Deferred to a later Phase-1 slice; not read by this one. */
    NOT_CAPTURED_IN_FOUNDATION_SLICE = 5U
};

} // namespace semantic
} // namespace ORB_SLAM3

#endif // SEMANTIC_GRAPH_SNAPSHOT_UNAVAILABLE_REASON_H
