/*!
 * @file            semantic_graph_snapshot.h
 *
 * @brief           Public interface of the semantic_graph_snapshot module.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_GRAPH_SNAPSHOT_SEMANTIC_GRAPH_SNAPSHOT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_GRAPH_SNAPSHOT_SEMANTIC_GRAPH_SNAPSHOT_H

#include "semantic/semantic_graph_snapshot/objects/EntityKeyStruct.h"
#include "semantic/semantic_graph_snapshot/objects/EntityKindEnum.h"
#include "semantic/semantic_graph_snapshot/objects/EntityRefStruct.h"
#include "semantic/semantic_graph_snapshot/objects/FloorRecordStruct.h"
#include "semantic/semantic_graph_snapshot/objects/MapSnapshotStruct.h"
#include "semantic/semantic_graph_snapshot/objects/OpenPassageHypothesisRecordStruct.h"
#include "semantic/semantic_graph_snapshot/objects/PassageRecordStruct.h"
#include "semantic/semantic_graph_snapshot/objects/RawPlaneRefStruct.h"
#include "semantic/semantic_graph_snapshot/objects/RoomRecordStruct.h"
#include "semantic/semantic_graph_snapshot/objects/SemanticGraphSnapshotStatusEnum.h"
#include "semantic/semantic_graph_snapshot/objects/SemanticGraphSnapshotStruct.h"
#include "semantic/semantic_graph_snapshot/objects/UnavailableReasonEnum.h"
#include "semantic/semantic_graph_snapshot/objects/UnresolvedWallHypothesisRecordStruct.h"
#include "semantic/semantic_graph_snapshot/objects/WallRecordStruct.h"


/* Recovered public contracts from the pre-migration API. */
#include <string>
#include <vector>
namespace vs_graphs::core::atlas { class Atlas; }
namespace vs_graphs::core::geometric { class Plane; }

namespace vs_graphs::core::semantic
{
/*!
 * @brief           Captures an immutable, value-only, sorted snapshot of every
 *                  live map's semantic graph state currently owned by
 *                  \p p_atlas_in.
 *
 * @pre             The caller must already hold the Atlas semantic-update lock
 *                  (see \ref Atlas::acquireSemanticUpdateLock()); this function
 *                  does not acquire it, and Atlas's semantic-update mutex is a
 *                  plain, non-recursive std::mutex that would deadlock if
 *                  re-entered on the same thread.
 *
 *                  Coherence relies on that precondition composed with a
 *                  source-verified fact about the current production system:
 *                  every reachable writer of Room/Passage/Floor *topology*
 *                  (walls, doorways, floor links, boundary status/corners/gaps,
 *                  room variant, ground plane) also holds the same lock while
 *                  writing (SemanticsManager::Run(), map-merge fusion in
 *                  Utils.cc, Floor.cc's addRoom()/setRooms()/replaceRoom() when
 *                  called from those paths, and LoopClosing.cc's
 *                  collapseMergedFloors() under MergeLocal()/
 *                  MergeLocalInertial()). Each individual Room/Passage/Plane/
 *                  Floor getter this function calls is independently
 *                  self-locked (memory-safe on its own), so the composed
 *                  guarantee is: no topology field can change *between* two
 *                  getter calls made during one capture, because no writer can
 *                  run at all while the caller holds the lock this function
 *                  requires.
 *
 *                  Two verified exceptions to "every writer holds the lock",
 *                  neither of which this function works around by adding a new
 *                  mutex (out of scope for this slice):
 *                  - Room/Plane/Passage/Floor *centroid* setters are also
 *                    called from the bundle-adjustment path (Optimizer.cc)
 *                    without the semantic lock. This is pre-existing,
 *                    per-field mutex-protected (never a data race), and
 *                    already how every other centroid consumer in this
 *                    codebase (RViz publishers, common.cc) treats it: a
 *                    captured centroid is best-effort/point-in-time, not
 *                    jointly atomic with the topology fields above.
 *                  - Atlas::matchRoomsToContext() calls Room::setWalls() while
 *                    holding only Atlas::roomContextMutex, a different mutex
 *                    from the semantic-update lock. It currently has **no
 *                    production caller** (verified by full-tree grep; its only
 *                    caller is test_atlas_lock_order.cpp, invoked directly and
 *                    single-threaded), despite a stale comment in System.cc
 *                    claiming otherwise. If this function is ever wired to a
 *                    live caller, this precondition must be re-verified before
 *                    relying on it again.
 *
 * @param[in]       p_atlas_in
 *                  Atlas to read from. Non-const because the
 *                  current Atlas/Map enumeration API
 *                  (GetCoherentMapView(), Map::GetAllRooms(), ...)
 *                  is non-const even though this function only
 *                  reads through it. Passing nullptr returns a
 *                  default-constructed (empty) snapshot.
 *
 * @param[out]      snapshot_out
 *                  The captured snapshot.
 */
[[nodiscard]] SemanticGraphSnapshotStatus
    captureSemanticGraphSnapshot(atlas::Atlas                 *p_atlas_in,
                                 SemanticGraphSnapshot &snapshot_out);

/*!
 * @brief           Builds a RawPlaneRef to \p p_plane_in, of any
 *                  Plane::PlaneVariant; reason is
 *                  UnavailableReason::NULL_REFERENCE when \p p_plane_in is
 *                  nullptr and UnavailableReason::NONE otherwise (mapId remains
 *                  independently absent when the plane itself has none, and
 *                  wallKey remains independently absent unless the plane is
 *                  genuinely WALL-typed and mapped). Public (not
 *                  private_functions.h) because SemanticsManager also needs it
 *                  to convert its own manager-private Plane* evidence
 *                  (openPassageEvidence/undefendedWalls) into pointer-free
 *                  value records at the semantic transaction boundary.
 */
[[nodiscard]] SemanticGraphSnapshotStatus
    rawPlaneRef(geometric::Plane *p_plane_in, RawPlaneRef &rawPlaneRef_out);
}

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_GRAPH_SNAPSHOT_SEMANTIC_GRAPH_SNAPSHOT_H \
        */
