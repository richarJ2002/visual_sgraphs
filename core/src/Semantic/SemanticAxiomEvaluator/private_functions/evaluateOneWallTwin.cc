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
 * @file            evaluateOneWallTwin.cc
 *
 * @brief           Implements evaluateOneWallTwin(), declared in
 *                  private_functions.h.
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

void evaluateOneWallTwin(const WallRecord            &wall_in,
                         const SemanticGraphSnapshot &snapshot_in,
                         std::vector<Finding>        &findings_inout)
{
    const RawPlaneRef &twin = wall_in.twinRef;
    if (twin.reason != UnavailableReason::NONE)
    {
        /* Checkpoint-A residual repair (D2): RawPlaneRef documents
         * reason == NONE exactly when the underlying plane pointer was
         * non-null; a "reason claims absent" value that nonetheless carries
         * populated data (mapId/wallKey/a real planeType) is an invariant
         * violation and a known contradiction, not an ordinary absent
         * twin. */
        if (twin.mapId.has_value() || twin.wallKey.has_value() ||
            twin.planeType != geometric::Plane::planeVariant::UNDEFINED)
        {
            findings_inout.push_back(
                makeFinding(AxiomCode::AX_WALL_03,
                            AxiomResult::FAIL,
                            ReasonCode::WALL_TWIN_REASON_INCONSISTENT,
                            {wall_in.key}));
            return;
        }
        findings_inout.push_back(makeFinding(AxiomCode::AX_WALL_03,
                                             AxiomResult::PASS,
                                             ReasonCode::WALL_TWIN_ABSENT,
                                             {wall_in.key}));
        return;
    }

    if (twin.planeType != geometric::Plane::planeVariant::WALL)
    {
        findings_inout.push_back(makeFinding(AxiomCode::AX_WALL_03,
                                             AxiomResult::FAIL,
                                             ReasonCode::WALL_TWIN_WRONG_TYPE,
                                             {wall_in.key}));
        return;
    }

    if (!twin.wallKey.has_value())
    {
        /* planeType == WALL but wallKey absent means the twin has no map
         * (rawPlaneRef()'s documented invariant); same-map/asymmetric/
         * shared-owner cannot be checked without one. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::UNKNOWN,
                        ReasonCode::WALL_TWIN_MAP_UNAVAILABLE,
                        {wall_in.key}));
        return;
    }

    if (*twin.wallKey == wall_in.key)
    {
        findings_inout.push_back(makeFinding(AxiomCode::AX_WALL_03,
                                             AxiomResult::FAIL,
                                             ReasonCode::WALL_TWIN_SELF,
                                             {wall_in.key}));
        return;
    }

    if (twin.wallKey->mapId != wall_in.key.mapId)
    {
        findings_inout.push_back(makeFinding(AxiomCode::AX_WALL_03,
                                             AxiomResult::FAIL,
                                             ReasonCode::WALL_TWIN_CROSS_MAP,
                                             {wall_in.key, *twin.wallKey}));
        return;
    }

    if (countWallRecordsWithKey(snapshot_in, *twin.wallKey) > 1U)
    {
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_DUPLICATE_IDENTITY,
                        {wall_in.key, *twin.wallKey}));
        return;
    }
    if (countMapSnapshotsWithId(snapshot_in, twin.wallKey->mapId) > 1U)
    {
        /* Checkpoint-A residual repair: which MapSnapshot actually holds
         * the twin is itself ambiguous when its own containing map id is
         * duplicated -- findWallByKeyInSnapshot()'s first-match lookup may
         * not supply positive proof in that case. */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_CONTAINING_MAP_AMBIGUOUS,
                        {wall_in.key, *twin.wallKey}));
        return;
    }

    const WallRecord *p_twinRecord =
        findWallByKeyInSnapshot(snapshot_in, *twin.wallKey);
    if (p_twinRecord == nullptr)
    {
        /* Same map id as this wall, WALL-typed, but not locatable among the
         * captured active maps' wall records -- an anomaly this schema
         * cannot further diagnose (see badRetiredMapVisibility). */
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::UNKNOWN,
                        ReasonCode::WALL_TWIN_MAP_UNAVAILABLE,
                        {wall_in.key, *twin.wallKey}));
        return;
    }

    if (!p_twinRecord->isLive)
    {
        findings_inout.push_back(makeFinding(AxiomCode::AX_WALL_03,
                                             AxiomResult::FAIL,
                                             ReasonCode::WALL_TWIN_BAD,
                                             {wall_in.key, *twin.wallKey}));
        return;
    }

    const bool isSymmetric = p_twinRecord->twinRef.wallKey.has_value() &&
                             (*p_twinRecord->twinRef.wallKey == wall_in.key);
    if (!isSymmetric)
    {
        findings_inout.push_back(makeFinding(AxiomCode::AX_WALL_03,
                                             AxiomResult::FAIL,
                                             ReasonCode::WALL_TWIN_ASYMMETRIC,
                                             {wall_in.key, *twin.wallKey}));
        return;
    }

    std::vector<EntityKey> sharedLiveOwnerKeys;
    for (const EntityRef &ownerA : wall_in.ownerRoomRefs)
    {
        if (!ownerA.key.has_value() ||
            !(ownerA.isLive.has_value() && *ownerA.isLive))
        {
            continue;
        }
        for (const EntityRef &ownerB : p_twinRecord->ownerRoomRefs)
        {
            if (ownerB.key.has_value() && *ownerB.key == *ownerA.key)
            {
                sharedLiveOwnerKeys.push_back(*ownerA.key);
            }
        }
    }
    if (!sharedLiveOwnerKeys.empty())
    {
        std::vector<EntityKey> involvedKeys{wall_in.key, *twin.wallKey};
        involvedKeys.insert(involvedKeys.end(),
                            sharedLiveOwnerKeys.begin(),
                            sharedLiveOwnerKeys.end());
        findings_inout.push_back(
            makeFinding(AxiomCode::AX_WALL_03,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_TWIN_SHARED_OWNER_FORBIDDEN,
                        std::move(involvedKeys)));
        return;
    }

    findings_inout.push_back(makeFinding(
        AxiomCode::AX_WALL_03,
        AxiomResult::UNKNOWN,
        ReasonCode::WALL_TWIN_STRUCTURALLY_VALID_GEOMETRY_UNVERIFIED,
        {wall_in.key, *twin.wallKey}));
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
