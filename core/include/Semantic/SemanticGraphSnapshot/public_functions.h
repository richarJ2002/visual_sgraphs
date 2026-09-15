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
 * @file            public_functions.h
 *
 * @brief           Declares the public entry point of the
 *                  SemanticGraphSnapshot module (CPP_CODING_STANDARD.md
 *                  Section 5.4).
 */

#ifndef SEMANTIC_GRAPH_SNAPSHOT_PUBLIC_FUNCTIONS_H
#define SEMANTIC_GRAPH_SNAPSHOT_PUBLIC_FUNCTIONS_H

#include "Semantic/SemanticGraphSnapshot/objects.h"

namespace vs_graphs
{
namespace core
{
/* Forward declaration only: captureSemanticGraphSnapshot() takes a
 * non-owning Atlas pointer purely to read from it. Nothing in this module
 * stores an Atlas/Map/Room/Passage/Floor/Plane pointer -- every struct in
 * objects.h is a plain value type, safe to copy and to outlive the source
 * model objects. */
class Atlas;

/* Forward declaration only: rawPlaneRef() takes a non-owning Plane pointer
 * purely to read from it into a pointer-free RawPlaneRef value. */
namespace geometric { class Plane; }

namespace semantic
{
/*!
 * @brief       Captures an immutable, value-only, sorted snapshot of every
 *              live map's semantic graph state currently owned by
 *              \p p_atlas_in.
 *
 * @pre         The caller must already hold the Atlas semantic-update lock
 *              (see \ref Atlas::acquireSemanticUpdateLock()); this function
 *              does not acquire it, and Atlas's semantic-update mutex is a
 *              plain, non-recursive std::mutex that would deadlock if
 *              re-entered on the same thread.
 *
 *              Coherence relies on that precondition composed with a
 *              source-verified fact about the current production system:
 *              every reachable writer of Room/Passage/Floor *topology*
 *              (walls, doorways, floor links, boundary status/corners/gaps,
 *              room variant, ground plane) also holds the same lock while
 *              writing (SemanticsManager::Run(), map-merge fusion in
 *              Utils.cc, Floor.cc's addRoom()/setRooms()/replaceRoom() when
 *              called from those paths, and LoopClosing.cc's
 *              collapseMergedFloors() under MergeLocal()/
 *              MergeLocalInertial()). Each individual Room/Passage/Plane/
 *              Floor getter this function calls is independently
 *              self-locked (memory-safe on its own), so the composed
 *              guarantee is: no topology field can change *between* two
 *              getter calls made during one capture, because no writer can
 *              run at all while the caller holds the lock this function
 *              requires.
 *
 *              Two verified exceptions to "every writer holds the lock",
 *              neither of which this function works around by adding a new
 *              mutex (out of scope for this slice):
 *              - Room/Plane/Passage/Floor *centroid* setters are also
 *                called from the bundle-adjustment path (Optimizer.cc)
 *                without the semantic lock. This is pre-existing,
 *                per-field mutex-protected (never a data race), and
 *                already how every other centroid consumer in this
 *                codebase (RViz publishers, common.cc) treats it: a
 *                captured centroid is best-effort/point-in-time, not
 *                jointly atomic with the topology fields above.
 *              - Atlas::matchRoomsToContext() calls Room::setWalls() while
 *                holding only Atlas::mRoomContextMutex, a different mutex
 *                from the semantic-update lock. It currently has **no
 *                production caller** (verified by full-tree grep; its only
 *                caller is test_atlas_lock_order.cpp, invoked directly and
 *                single-threaded), despite a stale comment in System.cc
 *                claiming otherwise. If this function is ever wired to a
 *                live caller, this precondition must be re-verified before
 *                relying on it again.
 *
 * @param[in]   p_atlas_in  Atlas to read from. Non-const because the
 *                          current Atlas/Map enumeration API
 *                          (GetCoherentMapView(), Map::GetAllRooms(), ...)
 *                          is non-const even though this function only
 *                          reads through it. Passing nullptr returns a
 *                          default-constructed (empty) snapshot.
 */
SemanticGraphSnapshot captureSemanticGraphSnapshot(Atlas *p_atlas_in);

/*! @brief Builds a RawPlaneRef to \p p_plane_in, of any Plane::planeVariant;
 *  reason is UnavailableReason::NULL_REFERENCE when \p p_plane_in is
 *  nullptr and UnavailableReason::NONE otherwise (mapId remains
 *  independently absent when the plane itself has none, and wallKey
 *  remains independently absent unless the plane is genuinely WALL-typed
 *  and mapped). Public (not private_functions.h) because SemanticsManager
 *  also needs it to convert its own manager-private Plane* evidence
 *  (openPassageEvidence_/undefendedWalls_) into pointer-free value
 *  records at the semantic transaction boundary -- see P1.4/P1.7 of
 *  semantic-axiom-reliability-plan.md. */
RawPlaneRef rawPlaneRef(geometric::Plane *p_plane_in);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_GRAPH_SNAPSHOT_PUBLIC_FUNCTIONS_H
