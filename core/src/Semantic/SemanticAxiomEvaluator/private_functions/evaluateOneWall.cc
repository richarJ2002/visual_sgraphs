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
 * @file         evaluateOneWall.cc
 *
 * @brief        Implements evaluateOneWall(), declared in
 *               private_functions.h.
 *
 *               A single owner reference is not accepted merely for
 *               being keyed, same-map, and not explicitly bad. Full
 *               positive proof additionally requires: owner liveness
 *               explicitly available (not merely absent-and-assumed-fine);
 *               a unique matching live RoomRecord (no duplicate same-key
 *               ambiguity); that RoomRecord's own variant is ROOM (a
 *               prospective handle cannot own a committed wall); that
 *               RoomRecord's own declaredMapId agreeing with the wall's
 *               containing map; and that RoomRecord's own wallRefs
 *               resolving back to this wall (reciprocity). Missing
 *               liveness or a genuinely unenumerable owner record is
 *               UNKNOWN; every other check is a proven contradiction
 *               (FAIL).
 *
 *               The wall-record -> owner-ref -> live room-record ->
 *               reciprocal wall-ref chain is fully covered: this
 *               WallRecord's own duplicate-identity check (which wall this
 *               evaluation is even about must itself be unambiguous); the
 *               wall's own declaredMapId consistency with its containing
 *               map; the resolved owner RoomRecord's own isLive
 *               (independent of the owning reference's own captured
 *               liveness, catching a record/reference disagreement no
 *               earlier check observes); and the reciprocal wallRefs
 *               entry's own isLive (not merely its wallKey identity). The
 *               owner key's kind is always EntityKind::ROOM by
 *               construction (WallRecord::ownerRoomRefs is built
 *               exclusively from captureSemanticGraphSnapshot.cc's
 *               wallOwnersByPointer inversion pass, which always assigns
 *               makeKey(EntityKind::ROOM, ...)), the same structural
 *               exclusion scanReversePassageEndpoints.cc documents for
 *               RoomRecord::passageRefs -- validated as data anyway below
 *               (see the wall/owner key-kind checks), since public
 *               snapshot records are deliberately mutable adversarial
 *               inputs.
 *
 *               This wall's own key.kind == WALL and planeType == WALL
 *               checks; the owner reference's own key.kind == ROOM check; caps
 *               positive proof at UNKNOWN (rather than PASS) when the
 *               wall's own declaredMapId is genuinely absent, checked
 *               only after every other clause is affirmatively
 *               satisfied; and replaces the first-equal-wallKey
 *               reciprocal scan with a full scan of every wallRefs entry
 *               sharing this wall's own raw mapId/planeId identity --
 *               any such entry that is not simultaneously WALL-typed,
 *               live, and wallKey-consistent is a known contradiction
 *               (WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY) even beside an
 *               otherwise well-formed entry, and more than one
 *               well-formed entry is itself ambiguous
 *               (WALL_OWNERSHIP_RECIPROCAL_DUPLICATE).
 */

#include "Semantic/SemanticAxiomEvaluator/private_functions.h"

#include <cstddef>
#include <rclcpp/logging.hpp>
#include <utility>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticAxiomEvaluatorStatus
    evaluateOneWall(const WallRecord            &wall_in,
                    const SemanticGraphSnapshot &snapshot_in,
                    std::vector<Finding>        &findings_inout)
{
    if (wall_in.key.kind != EntityKind::WALL)
    {
        Finding finding{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_WALL_WRONG_KEY_KIND,
                        {wall_in.key},
                        finding) != SemanticAxiomEvaluatorStatus::
                                        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (wall_in.planeType != geometric::Plane::PlaneVariant::WALL)
    {
        Finding finding2{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_WALL_WRONG_PLANE_TYPE,
                        {wall_in.key},
                        finding2) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding2);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    std::size_t wallRecords{};
    if (countWallRecordsWithKey(snapshot_in, wall_in.key, wallRecords) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countWallRecordsWithKey returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (wallRecords > 1U)
    {
        Finding finding3{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_WALL_DUPLICATE_IDENTITY,
                        {wall_in.key},
                        finding3) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding3);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (wall_in.declaredMapId.has_value() &&
        *wall_in.declaredMapId != wall_in.key.mapId)
    {
        Finding finding4{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_MISMATCH,
                        {wall_in.key},
                        finding4) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding4);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    const std::size_t ownerCount = wall_in.ownerRoomRefs.size();

    if (ownerCount == 0U)
    {
        Finding finding5{};
        if (makeFinding(
                AxiomCode::AX_WALL_01,
                AxiomResult::UNKNOWN,
                ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE,
                {wall_in.key},
                finding5) != SemanticAxiomEvaluatorStatus::
                                 SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding5);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (ownerCount > 1U)
    {
        std::vector<EntityKey> involvedKeys{wall_in.key};
        for (const EntityRef &owner : wall_in.ownerRoomRefs)
        {
            if (owner.key.has_value())
            {
                involvedKeys.push_back(*owner.key);
            }
        }
        FindingEvidence evidence;
        evidence.observedCount = ownerCount;
        Finding finding6{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS,
                        std::move(involvedKeys),
                        finding6,
                        evidence) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding6);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    const EntityRef &owner = wall_in.ownerRoomRefs.front();
    if (!owner.key.has_value())
    {
        Finding finding7{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_UNRESOLVABLE,
                        {wall_in.key},
                        finding7) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding7);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (owner.key->kind != EntityKind::ROOM)
    {
        Finding finding8{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_KEY_KIND,
                        {wall_in.key, *owner.key},
                        finding8) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding8);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (owner.reason != UnavailableReason::NONE)
    {
        /* EntityRef documents
         * key.has_value() <=> reason == NONE as an invariant; a keyed
         * owner reference whose own reason is not NONE is a known
         * contradiction, not an ordinary valid reference. */
        Finding finding9{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_REASON_INCONSISTENT,
                        {wall_in.key, *owner.key},
                        finding9) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding9);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (owner.key->mapId != wall_in.key.mapId)
    {
        Finding finding10{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_CROSS_MAP,
                        {wall_in.key, *owner.key},
                        finding10) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding10);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    std::size_t roomRecords{};
    if (countRoomRecordsWithKey(snapshot_in, *owner.key, roomRecords) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countRoomRecordsWithKey returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (roomRecords > 1U)
    {
        Finding finding11{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_DUPLICATE_IDENTITY,
                        {wall_in.key, *owner.key},
                        finding11) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding11);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    std::size_t mapSnapshots{};
    if (countMapSnapshotsWithId(snapshot_in, owner.key->mapId, mapSnapshots) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: countMapSnapshotsWithId returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    if (mapSnapshots > 1U)
    {
        /* Which MapSnapshot actually holds
         * the owner room is itself ambiguous when its own containing map id
         * is duplicated -- no first-match RoomRecord lookup below may
         * supply positive proof. */
        Finding finding12{};
        if (makeFinding(
                AxiomCode::AX_WALL_01,
                AxiomResult::FAIL,
                ReasonCode::WALL_OWNERSHIP_OWNER_CONTAINING_MAP_AMBIGUOUS,
                {wall_in.key, *owner.key},
                finding12) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding12);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (owner.isLive.has_value() && !(*owner.isLive))
    {
        Finding finding13{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_BAD,
                        {wall_in.key, *owner.key},
                        finding13) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding13);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    const RoomRecord *p_owner = nullptr;
    for (const MapSnapshot &mapSnapshot : snapshot_in.maps)
    {
        if (mapSnapshot.mapId != owner.key->mapId)
        {
            continue;
        }
        const RoomRecord *p_record = nullptr;
        if (findRecordByKey(mapSnapshot.rooms, *owner.key, p_record) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: findRecordByKey returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        p_owner = p_record;
        break;
    }
    if (p_owner == nullptr)
    {
        Finding finding14{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_UNAVAILABLE,
                        {wall_in.key, *owner.key},
                        finding14) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding14);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (!p_owner->isLive)
    {
        Finding finding15{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_NOT_LIVE,
                        {wall_in.key, *owner.key},
                        finding15) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding15);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (p_owner->variant != Room::RoomVariant::ROOM)
    {
        Finding finding16{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_VARIANT,
                        {wall_in.key, *owner.key},
                        finding16) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding16);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    if (p_owner->declaredMapId.has_value() &&
        *p_owner->declaredMapId != wall_in.key.mapId)
    {
        Finding finding17{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH,
                        {wall_in.key, *owner.key},
                        finding17) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding17);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    /* A genuinely absent
     * p_owner->declaredMapId is missing evidence, not a proven mismatch --
     * the UNKNOWN cap for it is deferred to after the PASS below (see
     * wall_in.declaredMapId's own analogous cap), preserving FAIL >
     * UNKNOWN precedence against the reciprocal checks still to come. */

    /* The source EntityRef's own liveness being
     * genuinely unproven is deferred to a cap appended after the terminal
     * PASS below (see wall_in.declaredMapId's own analogous cap), not an
     * early UNKNOWN return here. The reciprocal-scan FAILs immediately below
     * depend only on p_owner->wallRefs (already resolved above) and
     * wall_in.key -- never on owner.isLive -- so an early return here would
     * mask them too, one step further than the declared-map-mismatch
     * check ordering already established for the checks above it. */

    /* Every wallRefs entry that
     * shares this wall's own raw mapId/planeId identity is inspected, not
     * only the first equal-wallKey entry -- a well-formed reciprocal
     * reference has reason == NONE (implied by wallKey.has_value(), see
     * RawPlaneRef.h's invariant), WALL type, live state, and a wallKey that
     * agrees with that same mapId/planeId. Any entry sharing this wall's
     * raw identity but failing that full check is a known contradiction
     * (WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY), dominating even when
     * another entry is well-formed; more than one well-formed entry is
     * itself ambiguous (WALL_OWNERSHIP_RECIPROCAL_DUPLICATE). */
    std::size_t wellFormedReciprocalCount  = 0U;
    bool        anyContradictoryReciprocal = false;
    for (const RawPlaneRef &ownedWallReference : p_owner->wallRefs)
    {
        /* Relevance is no longer keyed on raw
         * mapId/planeId alone -- an entry whose own wallKey already names
         * this exact wall is about this wall too, even when its raw
         * mapId/planeId field has been adversarially mutated to disagree
         * with that same wallKey (an internal RawPlaneRef inconsistency,
         * not evidence of an unrelated wall). Checking wallKey alone would
         * conversely miss a raw-identity match whose wallKey was never
         * populated (e.g. reason != NONE); either signal alone is enough to
         * pull the entry into this loop's scope. */
        const bool aboutThisWallByRaw =
            ownedWallReference.mapId.has_value() &&
            (*ownedWallReference.mapId == wall_in.key.mapId) &&
            (ownedWallReference.planeId == wall_in.key.entityId);
        const bool aboutThisWallByKey =
            ownedWallReference.wallKey.has_value() &&
            (*ownedWallReference.wallKey == wall_in.key);
        if (!aboutThisWallByRaw && !aboutThisWallByKey)
        {
            continue;
        }
        if (ownedWallReference.reason != UnavailableReason::NONE)
        {
            /* This entry's own
             * mapId/planeId match this wall, but its own reason claims
             * "absent" (RawPlaneRef::reason != NONE) -- an invariant
             * violation and a known contradiction, not an ordinary unrelated
             * reference to skip. Populated data with a non-NONE reason must
             * never be silently masked. */
            anyContradictoryReciprocal = true;
            continue;
        }
        if (aboutThisWallByRaw != aboutThisWallByKey)
        {
            /* Exactly one of the two identity signals names this wall: the
             * entry's own wallKey and its own raw mapId/planeId disagree
             * about which wall it references -- an internal RawPlaneRef
             * inconsistency, a known contradiction. */
            anyContradictoryReciprocal = true;
            continue;
        }
        const bool wellFormed = (ownedWallReference.planeType ==
                                 geometric::Plane::PlaneVariant::WALL) &&
                                ownedWallReference.isLive &&
                                ownedWallReference.wallKey.has_value() &&
                                (*ownedWallReference.wallKey == wall_in.key);
        if (wellFormed)
        {
            ++wellFormedReciprocalCount;
        }
        else
        {
            anyContradictoryReciprocal = true;
        }
    }
    if (anyContradictoryReciprocal)
    {
        Finding finding18{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY,
                        {wall_in.key, *owner.key},
                        finding18) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding18);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (wellFormedReciprocalCount > 1U)
    {
        Finding finding19{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_DUPLICATE,
                        {wall_in.key, *owner.key},
                        finding19) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding19);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }
    if (wellFormedReciprocalCount == 0U)
    {
        Finding finding20{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::FAIL,
                        ReasonCode::WALL_OWNERSHIP_OWNER_NOT_RECIPROCAL,
                        {wall_in.key, *owner.key},
                        finding20) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding20);
        return SemanticAxiomEvaluatorStatus::
            SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
    }

    Finding finding21{};
    if (makeFinding(AxiomCode::AX_WALL_01,
                    AxiomResult::PASS,
                    ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER,
                    {wall_in.key, *owner.key},
                    finding21) !=
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: makeFinding returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    findings_inout.push_back(finding21);
    if (!wall_in.declaredMapId.has_value())
    {
        /* Every other clause is affirmatively satisfied, but the wall's own
         * declared map is genuinely absent evidence, not a contradiction:
         * cap positive ownership proof at UNKNOWN rather than PASS. */
        Finding finding22{};
        if (makeFinding(
                AxiomCode::AX_WALL_01,
                AxiomResult::UNKNOWN,
                ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_UNAVAILABLE,
                {wall_in.key, *owner.key},
                finding22) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding22);
    }
    if (!p_owner->declaredMapId.has_value())
    {
        /* The owner room's own declared
         * map is genuinely absent evidence, not a contradiction -- cap
         * positive ownership proof at UNKNOWN rather than PASS, mirroring
         * the wall's own missing-declared-map cap immediately above. */
        Finding finding23{};
        if (makeFinding(
                AxiomCode::AX_WALL_01,
                AxiomResult::UNKNOWN,
                ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_UNAVAILABLE,
                {wall_in.key, *owner.key},
                finding23) != SemanticAxiomEvaluatorStatus::
                                  SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding23);
    }
    if (!owner.isLive.has_value())
    {
        /* Every other clause -- including every
         * record-level FAIL and the full reciprocal-scan FAIL set above --
         * is affirmatively satisfied, but the source reference's own
         * liveness was never itself proven either way: cap positive
         * ownership proof at UNKNOWN rather than PASS, mirroring the
         * wall's/owner's own missing-declared-map caps immediately above. */
        Finding finding24{};
        if (makeFinding(AxiomCode::AX_WALL_01,
                        AxiomResult::UNKNOWN,
                        ReasonCode::WALL_OWNERSHIP_OWNER_LIVENESS_UNAVAILABLE,
                        {wall_in.key, *owner.key},
                        finding24) !=
            SemanticAxiomEvaluatorStatus::
                SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: makeFinding returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        findings_inout.push_back(finding24);
    }

    return SemanticAxiomEvaluatorStatus::
        SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
