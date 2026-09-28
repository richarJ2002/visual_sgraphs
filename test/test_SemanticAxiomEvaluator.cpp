/*!
 * Focused, ROS/Gazebo-free tests for the pure SemanticAxiomEvaluator module
 * (evaluateState(), evaluateTransition(), evaluateMapCompleteness(),
 * computeAxiomCapabilityTable()).
 *
 * Every test builds real Atlas/Map/Room/geometric::Plane/Passage/Floor objects
 * through SemanticFixtures and the model's own setters, captures a genuine
 * SemanticGraphSnapshot via the production captureSemanticGraphSnapshot()
 * entry point (never hand-constructing a snapshot), then evaluates it
 * through the production evaluator entry points -- matching
 * test_SemanticGraphSnapshot.cpp's own methodology.
 */

#include "Semantic/SemanticAxiomEvaluator.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "Semantic/Floor.h"
#include "Semantic/Passage.h"
#include "Semantic/Room.h"
#include "Semantic/SemanticAxiomEvaluator/private_functions.h"
#include "Semantic/SemanticGraphSnapshot.h"
#include "SemanticFixtures.h"
#include "SemanticGraphSnapshotTestHelpers.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

namespace
{
const AggregateAxiomResult *
    findAggregate(const AxiomEvaluationReport &report_in, AxiomCode code_in)
{
    for (const AggregateAxiomResult &aggregate : report_in.aggregates)
    {
        if (aggregate.axiomCode == code_in)
        {
            return &aggregate;
        }
    }
    return nullptr;
}

std::vector<const Finding *> findingsFor(const AxiomEvaluationReport &report_in,
                                         AxiomCode                    code_in)
{
    std::vector<const Finding *> matches;
    for (const Finding &finding : report_in.findings)
    {
        if (finding.axiomCode == code_in)
        {
            matches.push_back(&finding);
        }
    }
    return matches;
}

const Finding *findFindingWithReason(const AxiomEvaluationReport &report_in,
                                     AxiomCode                    code_in,
                                     ReasonCode                   reason_in)
{
    for (const Finding &finding : report_in.findings)
    {
        if (finding.axiomCode == code_in && finding.reasonCode == reason_in)
        {
            return &finding;
        }
    }
    return nullptr;
}

const AxiomCapabilityEntry *
    findCapability(const std::vector<AxiomCapabilityEntry> &table_in,
                   AxiomCode                                code_in)
{
    for (const AxiomCapabilityEntry &entry : table_in)
    {
        if (entry.axiomCode == code_in)
        {
            return &entry;
        }
    }
    return nullptr;
}
} // namespace

/* ------------------------------------------------------------------------
 * Structural / aggregation / determinism
 * ---------------------------------------------------------------------- */

TEST(SemanticAxiomEvaluator, AggregateReportContainsExactlyOneEntryPerAxiomCode)
{
    Atlas                 atlas(0);
    AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(report.aggregates.size(), 16U);

    std::vector<AxiomCode> codes;
    for (const AggregateAxiomResult &aggregate : report.aggregates)
    {
        codes.push_back(aggregate.axiomCode);
    }
    std::sort(codes.begin(),
              codes.end(),
              [](AxiomCode lhs_in, AxiomCode rhs_in)
              {
                  return static_cast<unsigned int>(lhs_in) <
                         static_cast<unsigned int>(rhs_in);
              });
    for (std::size_t index = 1U; index < codes.size(); ++index)
    {
        EXPECT_NE(codes[index - 1U], codes[index])
            << "duplicate axiom code in aggregate report";
    }
    EXPECT_EQ(codes.front(), AxiomCode::AX_FRAME_01);
    EXPECT_EQ(codes.back(), AxiomCode::AX_MERGE_01);
}

TEST(SemanticAxiomEvaluator,
     AxiomCapabilityTableHasSixteenEntriesSortedByCodeWithKnownAssignments)
{
    const std::vector<AxiomCapabilityEntry> table =
        computeAxiomCapabilityTable();
    ASSERT_EQ(table.size(), 16U);
    for (std::size_t index = 1U; index < table.size(); ++index)
    {
        EXPECT_LT(static_cast<unsigned int>(table[index - 1U].axiomCode),
                  static_cast<unsigned int>(table[index].axiomCode));
    }

    const AxiomCapabilityEntry *p_frame =
        findCapability(table, AxiomCode::AX_FRAME_01);
    ASSERT_NE(p_frame, nullptr);
    EXPECT_EQ(p_frame->capability, CapabilityLevel::DEFERRED);
    EXPECT_EQ(p_frame->owner, MissingProofOwner::PHASE_2);
    EXPECT_EQ(p_frame->classification, AxiomClass::HARD);

    const AxiomCapabilityEntry *p_pass02 =
        findCapability(table, AxiomCode::AX_PASS_02);
    ASSERT_NE(p_pass02, nullptr);
    EXPECT_EQ(p_pass02->capability, CapabilityLevel::PARTIAL);
    EXPECT_EQ(p_pass02->owner, MissingProofOwner::PHASE_3);

    const AxiomCapabilityEntry *p_floor01 =
        findCapability(table, AxiomCode::AX_FLOOR_01);
    ASSERT_NE(p_floor01, nullptr);
    EXPECT_EQ(p_floor01->capability, CapabilityLevel::PARTIAL);
    EXPECT_EQ(p_floor01->owner, MissingProofOwner::PHASE_3);

    const AxiomCapabilityEntry *p_comp01 =
        findCapability(table, AxiomCode::AX_COMP_01);
    ASSERT_NE(p_comp01, nullptr);
    EXPECT_EQ(p_comp01->classification, AxiomClass::DERIVED);
    EXPECT_EQ(p_comp01->capability, CapabilityLevel::PARTIAL);
    EXPECT_EQ(p_comp01->owner, MissingProofOwner::PHASE_7);

    const AxiomCapabilityEntry *p_merge01 =
        findCapability(table, AxiomCode::AX_MERGE_01);
    ASSERT_NE(p_merge01, nullptr);
    EXPECT_EQ(p_merge01->capability, CapabilityLevel::DEFERRED);
    EXPECT_EQ(p_merge01->owner, MissingProofOwner::PHASE_8);
}

/* Every DEFERRED axiom code (per computeAxiomCapabilityTable()) reports
 * exactly one placeholder Finding, always UNKNOWN, on an otherwise-empty
 * snapshot. */
TEST(SemanticAxiomEvaluator, DeferredAxiomsAlwaysReportExactlyOneUnknownFinding)
{
    Atlas                 atlas(0);
    AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));

    /* AX_ROOM_01 is deliberately excluded: it is a genuine per-room leaf
     * (evaluateOneRoomCreationProvenance() via evaluateAxRoom01()), like
     * AX-WALL-01/03, AX-BOUND-01, and AX-PASS-01/02/03/04 -- vacuously zero
     * findings (and a PASS-default aggregate) on this empty-Atlas fixture,
     * not a context-free placeholder. See
     * RoomCreationProvenanceIsPerRoomAndVacuousWithNoRooms below. */
    const AxiomCode deferredCodes[] = {AxiomCode::AX_FRAME_01,
                                       AxiomCode::AX_WALL_02,
                                       AxiomCode::AX_ROOM_02,
                                       AxiomCode::AX_LIFE_01,
                                       AxiomCode::AX_TXN_01,
                                       AxiomCode::AX_MERGE_01};
    for (const AxiomCode code : deferredCodes)
    {
        const std::vector<const Finding *> findings = findingsFor(report, code);
        ASSERT_EQ(findings.size(), 1U);
        EXPECT_EQ(findings.front()->result, AxiomResult::UNKNOWN);
        const AggregateAxiomResult *p_aggregate = findAggregate(report, code);
        ASSERT_NE(p_aggregate, nullptr);
        EXPECT_EQ(p_aggregate->result, AxiomResult::UNKNOWN);
    }

    const std::vector<const Finding *> roomProvenanceFindings =
        findingsFor(report, AxiomCode::AX_ROOM_01);
    EXPECT_TRUE(roomProvenanceFindings.empty());
    const AggregateAxiomResult *p_roomAggregate =
        findAggregate(report, AxiomCode::AX_ROOM_01);
    ASSERT_NE(p_roomAggregate, nullptr);
    EXPECT_EQ(p_roomAggregate->result, AxiomResult::PASS);
}

/* evaluateAxRoom01()/
 * computeConservativeMapCompleteness() must share one per-room
 * room-creation-provenance leaf (evaluateOneRoomCreationProvenance()), not a
 * blanket, non-per-room placeholder Finding. */
TEST(SemanticAxiomEvaluator,
     RoomCreationProvenanceIsPerRoomAndVacuousWithNoRooms)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  roomA;
    test::makeRoom(roomA, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&roomA);
    Room roomB;
    test::makeRoom(roomB, 2, p_map, nullptr, Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&roomB);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const std::vector<const Finding *> findings =
        findingsFor(report, AxiomCode::AX_ROOM_01);
    ASSERT_EQ(findings.size(), 2U);
    for (const Finding *p_finding : findings)
    {
        EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
        EXPECT_EQ(p_finding->reasonCode,
                  ReasonCode::ROOM_CREATION_PROVENANCE_UNAVAILABLE);
        ASSERT_EQ(p_finding->involvedKeys.size(), 1U);
    }
    EXPECT_NE(findings.front()->involvedKeys.front(),
              findings.back()->involvedKeys.front());
    const AggregateAxiomResult *p_aggregate =
        findAggregate(report, AxiomCode::AX_ROOM_01);
    ASSERT_NE(p_aggregate, nullptr);
    EXPECT_EQ(p_aggregate->result, AxiomResult::UNKNOWN);
}

TEST(SemanticAxiomEvaluator, AggregationPrecedenceFailBeatsUnknownBeatsPass)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    /* Wall 1: zero owners -> UNKNOWN. Wall 2: two owners -> FAIL. Wall 3:
     * one valid owner -> PASS. AX-WALL-01's aggregate must be FAIL. */
    geometric::Plane wallUnknown;
    test::makeWallPlane(wallUnknown,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wallUnknown);

    geometric::Plane wallMultiOwner;
    test::makeWallPlane(wallMultiOwner,
                        2,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wallMultiOwner);

    geometric::Plane wallValid;
    test::makeWallPlane(wallValid,
                        3,
                        p_map,
                        Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitY(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wallValid);

    Room ownerA;
    test::makeRoom(ownerA, 1, p_map, &wallMultiOwner);
    ASSERT_EQ((ownerA.setWalls(&wallValid)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&ownerA);

    Room ownerB;
    test::makeRoom(ownerB, 2, p_map, &wallMultiOwner);
    p_map->addDetectedMapRoom(&ownerB);

    std::unique_lock<std::mutex> lock    = atlas.acquireSemanticUpdateLock();
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    lock.unlock();

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const AggregateAxiomResult *p_wall01 =
        findAggregate(report, AxiomCode::AX_WALL_01);
    ASSERT_NE(p_wall01, nullptr);
    EXPECT_EQ(p_wall01->result, AxiomResult::FAIL);
}

/* Prove FAIL > UNKNOWN > PASS combining all three results for one SINGLE
 * entity/axiom (not merely three different entities), by exercising
 * aggregateFindings() directly with three findings that share the same
 * axiomCode and the same involvedKeys. */
TEST(SemanticAxiomEvaluator, SameEntityPrecedenceFailBeatsUnknownBeatsPass)
{
    const EntityKey        oneWallKey{EntityKind::WALL, 1UL, 1};
    const EntityKey        oneOwnerKey{EntityKind::ROOM, 1UL, 1};
    std::vector<EntityKey> involvedKeys{oneWallKey, oneOwnerKey};

    std::vector<Finding> findings;
    findings.push_back(
        makeFinding(AxiomCode::AX_WALL_01,
                    AxiomResult::PASS,
                    ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER,
                    involvedKeys));
    findings.push_back(makeFinding(
        AxiomCode::AX_WALL_01,
        AxiomResult::UNKNOWN,
        ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE,
        involvedKeys));
    findings.push_back(makeFinding(AxiomCode::AX_WALL_01,
                                   AxiomResult::FAIL,
                                   ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS,
                                   involvedKeys));

    const std::vector<AggregateAxiomResult> aggregates =
        aggregateFindings(findings);
    const AggregateAxiomResult *p_wall01 = nullptr;
    for (const AggregateAxiomResult &aggregate : aggregates)
    {
        if (aggregate.axiomCode == AxiomCode::AX_WALL_01)
        {
            p_wall01 = &aggregate;
        }
    }
    ASSERT_NE(p_wall01, nullptr);
    EXPECT_EQ(p_wall01->result, AxiomResult::FAIL);
    EXPECT_EQ(p_wall01->contributingFindingCount, 3U);

    /* Removing the FAIL finding must expose UNKNOWN dominating PASS for the
     * exact same entity/axiom. */
    std::vector<Finding> findingsWithoutFail{findings[0], findings[1]};
    const std::vector<AggregateAxiomResult> aggregatesWithoutFail =
        aggregateFindings(findingsWithoutFail);
    const AggregateAxiomResult *p_wall01WithoutFail = nullptr;
    for (const AggregateAxiomResult &aggregate : aggregatesWithoutFail)
    {
        if (aggregate.axiomCode == AxiomCode::AX_WALL_01)
        {
            p_wall01WithoutFail = &aggregate;
        }
    }
    ASSERT_NE(p_wall01WithoutFail, nullptr);
    EXPECT_EQ(p_wall01WithoutFail->result, AxiomResult::UNKNOWN);
}

/* SameEntityPrecedenceFailBeatsUnknown
 * BeatsPass above proves FAIL > UNKNOWN > PASS only via aggregateFindings()
 * called directly on hand-built findings. This test instead exercises the
 * same precedence through the real evaluateState() orchestrator over three
 * genuine WallRecord objects in one map -- one PASS (valid single owner),
 * one UNKNOWN (zero owners), one FAIL (multiple owners) -- proving AX-WALL-01's
 * aggregate is FAIL once a real contradiction exists anywhere in the map,
 * not merely a property of the aggregation function in isolation. */
TEST(SemanticAxiomEvaluator,
     AxWall01PrecedenceThroughEvaluateStateOverRealEntities)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    /* PASS: a genuinely valid, single-owner wall. */
    geometric::Plane passWall;
    test::makeWallPlane(passWall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&passWall);
    Room passRoom;
    test::makeRoom(passRoom, 1, p_map, &passWall);
    p_map->addDetectedMapRoom(&passRoom);

    /* UNKNOWN: a wall with zero owners -- commitment unverifiable. */
    geometric::Plane unknownWall;
    test::makeWallPlane(unknownWall,
                        2,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(5.0, 0.0, 0.0));
    p_map->addMapPlane(&unknownWall);

    /* FAIL: a wall with two owners. */
    geometric::Plane failWall;
    test::makeWallPlane(failWall,
                        3,
                        p_map,
                        Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitY(),
                        1.0,
                        1.0,
                        Eigen::Vector3d(10.0, 0.0, 0.0));
    p_map->addMapPlane(&failWall);
    Room failOwnerA;
    test::makeRoom(failOwnerA,
                   2,
                   p_map,
                   &failWall,
                   Eigen::Vector3d(10.0, -1.0, 0.0));
    p_map->addDetectedMapRoom(&failOwnerA);
    Room failOwnerB;
    test::makeRoom(failOwnerB,
                   3,
                   p_map,
                   &failWall,
                   Eigen::Vector3d(10.0, 1.0, 0.0));
    p_map->addDetectedMapRoom(&failOwnerB);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));

    ASSERT_NE(
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER),
        nullptr);
    ASSERT_NE(
        findFindingWithReason(
            report,
            AxiomCode::AX_WALL_01,
            ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE),
        nullptr);
    ASSERT_NE(findFindingWithReason(report,
                                    AxiomCode::AX_WALL_01,
                                    ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS),
              nullptr);

    const AggregateAxiomResult *p_wall01Aggregate =
        findAggregate(report, AxiomCode::AX_WALL_01);
    ASSERT_NE(p_wall01Aggregate, nullptr);
    EXPECT_EQ(p_wall01Aggregate->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator,
     FindingIdsAreDeterministicRegardlessOfConstructionOrder)
{
    /* A separate Atlas assigns its Map a different id (Map::nextId is a
     * process-wide monotonic counter), which would make two independently
     * constructed "equivalent" fixtures genuinely differ in every mapId-
     * bearing field -- not the permutation-independence this test targets.
     * Proving order-independence instead uses one shared snapshot and a
     * manually re-ordered in-memory copy of its own record vectors, so
     * every id, including mapId, is identical between the two evaluated
     * inputs and only container/vector order differs. */
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room1;
    test::makeRoom(room1, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room1);
    Room room2;
    test::makeRoom(room2, 2, p_map, &wall);
    p_map->addDetectedMapRoom(&room2);

    const SemanticGraphSnapshot original = captureSemanticGraphSnapshot(&atlas);
    SemanticGraphSnapshot       reordered = original;
    ASSERT_EQ(reordered.maps.size(), 1U);
    std::reverse(reordered.maps.front().rooms.begin(),
                 reordered.maps.front().rooms.end());
    std::reverse(reordered.maps.front().walls.begin(),
                 reordered.maps.front().walls.end());

    const AxiomEvaluationReport reportOriginal  = evaluateState(original);
    const AxiomEvaluationReport reportReordered = evaluateState(reordered);

    ASSERT_EQ(reportOriginal.findings.size(), reportReordered.findings.size());
    for (std::size_t index = 0U; index < reportOriginal.findings.size();
         ++index)
    {
        EXPECT_EQ(reportOriginal.findings[index].id,
                  reportReordered.findings[index].id);
        EXPECT_EQ(reportOriginal.findings[index].result,
                  reportReordered.findings[index].result);
    }
}

TEST(SemanticAxiomEvaluator, FindingsAreSortedById)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    for (std::size_t index = 1U; index < report.findings.size(); ++index)
    {
        EXPECT_LE(report.findings[index - 1U].id, report.findings[index].id);
    }
}

TEST(SemanticAxiomEvaluator, EvaluateStateIsIdempotentAndDoesNotMutateInput)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    const AxiomEvaluationReport reportFirst  = evaluateState(snapshot);
    const AxiomEvaluationReport reportSecond = evaluateState(snapshot);

    ASSERT_EQ(reportFirst.findings.size(), reportSecond.findings.size());
    for (std::size_t index = 0U; index < reportFirst.findings.size(); ++index)
    {
        EXPECT_EQ(reportFirst.findings[index].id,
                  reportSecond.findings[index].id);
        EXPECT_EQ(reportFirst.findings[index].result,
                  reportSecond.findings[index].result);
    }
    ASSERT_EQ(reportFirst.aggregates.size(), reportSecond.aggregates.size());
    for (std::size_t index = 0U; index < reportFirst.aggregates.size(); ++index)
    {
        EXPECT_EQ(reportFirst.aggregates[index].result,
                  reportSecond.aggregates[index].result);
    }

    /* snapshot itself is unaffected: re-capturing after both evaluations
     * still reports one wall owned by one live room. */
    EXPECT_EQ(snapshot.maps.size(), 1U);
    EXPECT_EQ(snapshot.maps.front().walls.size(), 1U);
    EXPECT_EQ(snapshot.maps.front().walls.front().ownerRoomRefs.size(), 1U);
}

TEST(SemanticAxiomEvaluator,
     EvaluateTransitionReplacesFrameAndTxnPlaceholdersButKeepsRestFromAfter)
{
    Atlas            atlasBefore(0);
    Atlas            atlasAfter(0);
    Map             *p_mapAfter = atlasAfter.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapAfter,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_mapAfter->addMapPlane(&wall);

    const SemanticGraphSnapshot before =
        captureSemanticGraphSnapshot(&atlasBefore);
    const SemanticGraphSnapshot after =
        captureSemanticGraphSnapshot(&atlasAfter);

    const TransitionEvaluationContext context;
    const AxiomEvaluationReport       transitionReport =
        evaluateTransition(before, after, context);
    const AxiomEvaluationReport afterStateReport = evaluateState(after);

    const std::vector<const Finding *> frameFindings =
        findingsFor(transitionReport, AxiomCode::AX_FRAME_01);
    ASSERT_EQ(frameFindings.size(), 1U);
    EXPECT_EQ(frameFindings.front()->reasonCode,
              ReasonCode::FRAME_EQUIVARIANCE_NOT_YET_IMPLEMENTED);

    const std::vector<const Finding *> txnFindings =
        findingsFor(transitionReport, AxiomCode::AX_TXN_01);
    ASSERT_EQ(txnFindings.size(), 1U);
    EXPECT_EQ(txnFindings.front()->reasonCode,
              ReasonCode::TRANSACTION_POSTCONDITION_NOT_YET_IMPLEMENTED);

    /* AX-WALL-01 (a static axiom) is exactly evaluateState(after)'s own
     * finding, unaffected by transition mode. */
    const std::vector<const Finding *> wall01Transition =
        findingsFor(transitionReport, AxiomCode::AX_WALL_01);
    const std::vector<const Finding *> wall01State =
        findingsFor(afterStateReport, AxiomCode::AX_WALL_01);
    ASSERT_EQ(wall01Transition.size(), wall01State.size());
    for (std::size_t index = 0U; index < wall01Transition.size(); ++index)
    {
        EXPECT_EQ(wall01Transition[index]->id, wall01State[index]->id);
    }
}

/* ------------------------------------------------------------------------
 * AX-WALL-01: unique live wall ownership
 * ---------------------------------------------------------------------- */

TEST(SemanticAxiomEvaluator, OwnerlessWallIsUnknown)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

TEST(SemanticAxiomEvaluator, SingleValidSameMapOwnerIsPass)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::PASS);
}

TEST(SemanticAxiomEvaluator, MultipleOwnersIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room roomA;
    test::makeRoom(roomA, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&roomA);
    Room roomB;
    test::makeRoom(roomB, 2, p_map, &wall);
    p_map->addDetectedMapRoom(&roomB);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    ASSERT_TRUE(p_finding->evidence.observedCount.has_value());
    EXPECT_EQ(*p_finding->evidence.observedCount, 2U);
}

TEST(SemanticAxiomEvaluator, BadOwnerIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);
    ASSERT_EQ((room.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, CrossMapOwnerIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();
    ASSERT_NE(p_mapA->getId(), p_mapB->getId());

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_mapA->addMapPlane(&wall);

    /* Deliberately register a mapB room that owns mapA's wall: an upstream
     * contract violation this evaluator must still observe, not assume
     * away. */
    Room room;
    test::makeRoom(room, 1, p_mapB, &wall);
    p_mapB->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The owner reference is keyed same-map and live, but the located
 * RoomRecord's own variant is UNDEFINED (a prospective handle, not a
 * confirmed room): a committed wall cannot be owned by a prospective. */
TEST(SemanticAxiomEvaluator, WallOwnerWrongVariantIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room,
                   1,
                   p_map,
                   &wall,
                   Eigen::Vector3d::Zero(),
                   Room::RoomVariant::UNDEFINED);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_VARIANT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The owner reference is keyed in the wall's own containing map, but the
 * located RoomRecord's own declaredMapId names a different map -- the room
 * object itself disagrees about which map it belongs to. */
TEST(SemanticAxiomEvaluator, WallOwnerDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();
    ASSERT_NE(p_mapA->getId(), p_mapB->getId());

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_mapA->addMapPlane(&wall);

    /* room's own declared map is mapB (makeRoom's p_map_in), but it is
     * enumerated (registered) under mapA's collection, so its containing-map
     * key matches the wall's map while its declaredMapId does not. */
    Room room;
    test::makeRoom(room, 1, p_mapB, &wall);
    p_mapA->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Two distinct RoomRecord objects share the exact key the owner reference
 * names: which room actually owns the wall is ambiguous. */
TEST(SemanticAxiomEvaluator, WallOwnerDuplicateIdentityIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);

    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);
    /* A second, distinct Room object deliberately reuses local id 1 -- an
     * upstream collision this evaluator must observe, not assume away. */
    Room duplicateIdRoom;
    test::makeRoom(duplicateIdRoom, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&duplicateIdRoom);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_DUPLICATE_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The owner key is present, live, same-map, and uniquely keyed, but no
 * RoomRecord with that exact key exists anywhere in the captured maps --
 * genuinely missing enumeration evidence, not a proven contradiction.
 * WallRecord::ownerRoomRefs is itself derived by inverting the enumerated
 * RoomRecord list, so no public setter sequence can produce a keyed, live
 * owner reference this snapshot cannot also locate; manufactured directly
 * on the captured value snapshot, the sanctioned technique for a
 * corrupt/collision case no setter can represent. */
TEST(SemanticAxiomEvaluator, WallOwnerRecordUnavailableIsUnknown)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.front().ownerRoomRefs.size(), 1U);
    /* Retarget the owner reference at a local id no RoomRecord in this
     * snapshot actually has, keeping key.mapId/isLive unchanged. */
    snapshot.maps.front().walls.front().ownerRoomRefs.front().key->entityId =
        999;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_UNAVAILABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* The owner resolves to a located, live, ROOM-variant, same-map RoomRecord,
 * but that room's own wallRefs does not resolve back to this wall.
 * RoomRecord::wallRefs and WallRecord::ownerRoomRefs are both derived from
 * the same underlying Room::getWalls() forward list at capture time, so no
 * public setter sequence can desynchronize them; manufactured directly on
 * the captured value snapshot, same sanctioned technique as
 * WallOwnerRecordUnavailableIsUnknown. */
TEST(SemanticAxiomEvaluator, WallOwnerNotReciprocalIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    RoomRecord *p_roomRecord = nullptr;
    for (RoomRecord &roomRecord : snapshot.maps.front().rooms)
    {
        if (roomRecord.key.entityId == 1)
        {
            p_roomRecord = &roomRecord;
        }
    }
    ASSERT_NE(p_roomRecord, nullptr);
    ASSERT_FALSE(p_roomRecord->wallRefs.empty());
    p_roomRecord->wallRefs.clear();

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_NOT_RECIPROCAL);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Two distinct WallRecord objects share the exact key this evaluation is
 * about: which wall this is even for is ambiguous. Manufactured via direct
 * snapshot mutation -- no public setter sequence creates a duplicate-key
 * live WallRecord. */
TEST(SemanticAxiomEvaluator, WallDuplicateIdentityIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    WallRecord duplicateWall = snapshot.maps.front().walls.front();
    duplicateWall.ownerRoomRefs.clear();
    snapshot.maps.front().walls.push_back(duplicateWall);

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_WALL_DUPLICATE_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The wall's own declaredMapId disagrees with the containing map used to
 * enumerate and key it. Manufactured via direct snapshot mutation. */
TEST(SemanticAxiomEvaluator, WallDeclaredMapMismatchIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    snapshot.maps.front().walls.front().declaredMapId =
        snapshot.maps.front().walls.front().key.mapId + 1U;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The owner resolves to a located RoomRecord whose own isLive is false,
 * even though the owning reference's own captured liveness reported it as
 * live -- a record/reference disagreement no earlier check observes.
 * Manufactured via direct snapshot mutation. */
TEST(SemanticAxiomEvaluator, WallOwnerRecordNotLiveIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    snapshot.maps.front().rooms.front().isLive = false;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_NOT_LIVE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Manufactured via direct snapshot mutation -- captureSemanticGraphSnapshot()
 * never enumerates a non-WALL-typed geometric::Plane into MapSnapshot::walls at
 * all (see its own filter), so WallRecord::key.kind is always EntityKind::WALL
 * through the production capture path; validated as data anyway. */
TEST(SemanticAxiomEvaluator, WallWrongKeyKindIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    snapshot.maps.front().walls.front().key.kind = EntityKind::ROOM;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_WALL_WRONG_KEY_KIND);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Manufactured via direct snapshot mutation -- the same structural
 * exclusion as WallWrongKeyKindIsFail applies to WallRecord::planeType. */
TEST(SemanticAxiomEvaluator, WallWrongPlaneTypeIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    snapshot.maps.front().walls.front().planeType =
        geometric::Plane::PlaneVariant::DOOR;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_WALL_WRONG_PLANE_TYPE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Manufactured via direct snapshot mutation -- WallRecord::ownerRoomRefs is
 * built exclusively via makeKey(EntityKind::ROOM, ...), so an owner key of
 * any other kind is unreachable through the public API. */
TEST(SemanticAxiomEvaluator, WallOwnerWrongKeyKindIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.front().ownerRoomRefs.size(), 1U);
    snapshot.maps.front().walls.front().ownerRoomRefs.front().key->kind =
        EntityKind::WALL;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_KEY_KIND);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Every other AX-WALL-01 clause is affirmatively satisfied, but the wall's
 * own declaredMapId is genuinely absent: positive proof is capped at
 * UNKNOWN, not PASS -- the PASS finding itself still appears (the clause it
 * names is genuinely satisfied), but the aggregate is UNKNOWN. */
TEST(SemanticAxiomEvaluator, WallDeclaredMapUnavailableCapsAggregateAtUnknown)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    snapshot.maps.front().walls.front().declaredMapId.reset();

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_passFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER);
    ASSERT_NE(p_passFinding, nullptr);
    EXPECT_EQ(p_passFinding->result, AxiomResult::PASS);
    const Finding *p_unknownFinding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_UNAVAILABLE);
    ASSERT_NE(p_unknownFinding, nullptr);
    EXPECT_EQ(p_unknownFinding->result, AxiomResult::UNKNOWN);
    const AggregateAxiomResult *p_aggregate =
        findAggregate(report, AxiomCode::AX_WALL_01);
    ASSERT_NE(p_aggregate, nullptr);
    EXPECT_EQ(p_aggregate->result, AxiomResult::UNKNOWN);
}

/* The owner room's wallRefs contains one malformed entry sharing this
 * wall's own raw mapId/planeId identity (wrong planeType) and no
 * well-formed entry at all: a known contradiction, not a plain
 * not-reciprocal absence. Manufactured via direct snapshot mutation. */
TEST(SemanticAxiomEvaluator, WallReciprocalMalformedOnlyIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    roomRecord.wallRefs.front().planeType =
        geometric::Plane::PlaneVariant::DOOR;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The owner room's wallRefs contains two independent, well-formed
 * reciprocal entries for the same wall: which is "the" reciprocal link is
 * ambiguous. Manufactured via direct snapshot mutation -- Room::setWalls()
 * only ever admits one geometric::Plane pointer per call. */
TEST(SemanticAxiomEvaluator, WallReciprocalDuplicateIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    roomRecord.wallRefs.push_back(roomRecord.wallRefs.front());

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_RECIPROCAL_DUPLICATE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* One well-formed reciprocal entry plus one malformed entry sharing the
 * same raw wall identity: the known contradiction must dominate rather
 * than being hidden behind the otherwise-valid entry. */
TEST(SemanticAxiomEvaluator, WallReciprocalValidPlusMalformedIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    RawPlaneRef malformedCopy = roomRecord.wallRefs.front();
    malformedCopy.isLive      = false;
    roomRecord.wallRefs.push_back(malformedCopy);

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* ------------------------------------------------------------------------
 * AX-WALL-03: twin wall-face plausibility
 * ---------------------------------------------------------------------- */

TEST(SemanticAxiomEvaluator, NullTwinIsPass)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_ABSENT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::PASS);
}

TEST(SemanticAxiomEvaluator, SelfTwinIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    wall.setTwinFace(&wall);
    p_map->addMapPlane(&wall);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_SELF);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, AsymmetricTwinIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_map,
                        Eigen::Vector4d(-1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    /* Only wallA points to wallB; wallB's own twin stays absent. */
    wallA.setTwinFace(&wallB);
    p_map->addMapPlane(&wallA);
    p_map->addMapPlane(&wallB);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_ASYMMETRIC);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, BadTwinIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_map,
                        Eigen::Vector4d(-1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    wallA.setTwinFace(&wallB);
    wallB.setTwinFace(&wallA);
    wallB.setBad();
    p_map->addMapPlane(&wallA);
    p_map->addMapPlane(&wallB);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(report,
                                                     AxiomCode::AX_WALL_03,
                                                     ReasonCode::WALL_TWIN_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, CrossMapTwinIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();

    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_mapB,
                        Eigen::Vector4d(-1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    wallA.setTwinFace(&wallB);
    wallB.setTwinFace(&wallA);
    p_mapA->addMapPlane(&wallA);
    p_mapB->addMapPlane(&wallB);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, WrongTypeTwinIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    geometric::Plane groundNotWall;
    ASSERT_TRUE(test::makeGroundPlane(groundNotWall, 2, p_map));
    wall.setTwinFace(&groundNotWall);
    p_map->addMapPlane(&wall);
    p_map->addMapPlane(&groundNotWall);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_WRONG_TYPE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, SharedOwnerTwinIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_map,
                        Eigen::Vector4d(-1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    wallA.setTwinFace(&wallB);
    wallB.setTwinFace(&wallA);
    p_map->addMapPlane(&wallA);
    p_map->addMapPlane(&wallB);

    Room sharedOwner;
    test::makeRoom(sharedOwner, 1, p_map, &wallA);
    ASSERT_EQ((sharedOwner.setWalls(&wallB)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&sharedOwner);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_SHARED_OWNER_FORBIDDEN);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, StructurallyValidTwinIsUnknown)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_map,
                        Eigen::Vector4d(-1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    wallA.setTwinFace(&wallB);
    wallB.setTwinFace(&wallA);
    p_map->addMapPlane(&wallA);
    p_map->addMapPlane(&wallB);

    Room ownerA;
    test::makeRoom(ownerA, 1, p_map, &wallA);
    p_map->addDetectedMapRoom(&ownerA);
    Room ownerB;
    test::makeRoom(ownerB, 2, p_map, &wallB);
    p_map->addDetectedMapRoom(&ownerB);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_03,
        ReasonCode::WALL_TWIN_STRUCTURALLY_VALID_GEOMETRY_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* ------------------------------------------------------------------------
 * AX-PASS-01/02/03/04: passage provenance, cardinality, slot state, floor
 * ---------------------------------------------------------------------- */

TEST(SemanticAxiomEvaluator, NonPassableLivePassageIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  knownSide;
    test::makeRoom(knownSide,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&knownSide);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &knownSide,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      nullptr,
                      /*passable_in=*/false);
    p_map->addMapPassage(&passage);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_01,
                              ReasonCode::PASSAGE_PROVENANCE_NOT_PASSABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, PassableLivePassageIsUnknownForFullProvenance)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  knownSide;
    test::makeRoom(knownSide,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&knownSide);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &knownSide,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_01,
        ReasonCode::PASSAGE_PROVENANCE_FULL_CHAIN_UNVERIFIABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

TEST(SemanticAxiomEvaluator, ZeroConfirmedEndpointsIsFail)
{
    Atlas   atlas(0);
    Map    *p_map = atlas.getCurrentMap();
    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      nullptr,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_NO_CONFIRMED_ENDPOINT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Renamed from TwoConfirmedReciprocalEndpointsIsPass:
 * PassageRecord::endpointSlotReason is always
 * NOT_TRACKED_BY_CURRENT_SCHEMA, so two apparently valid, reciprocal legacy
 * pointers with no observable contradiction remain aggregate UNKNOWN, never
 * PASS -- see evaluateOnePassageCardinality.cc's own Doxygen. */
TEST(SemanticAxiomEvaluator, TwoConfirmedReciprocalEndpointsIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&far);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);

    const AggregateAxiomResult *p_aggregate =
        findAggregate(report, AxiomCode::AX_PASS_02);
    ASSERT_NE(p_aggregate, nullptr);
    EXPECT_EQ(p_aggregate->result, AxiomResult::UNKNOWN);

    const Finding *p_slotFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_03,
                              ReasonCode::PASSAGE_SLOT_STATE_VALID);
    ASSERT_NE(p_slotFinding, nullptr);
    EXPECT_EQ(p_slotFinding->result, AxiomResult::PASS);

    /* The clause itself genuinely passes, but the AX-PASS-03 aggregate must
     * still be UNKNOWN: authoritative endpoint-slot proof remains
     * permanently unavailable in this schema. */
    const Finding *p_slotUnverifiedFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_03,
        ReasonCode::PASSAGE_SLOT_ENDPOINT_PROOF_UNVERIFIED);
    ASSERT_NE(p_slotUnverifiedFinding, nullptr);
    EXPECT_EQ(p_slotUnverifiedFinding->result, AxiomResult::UNKNOWN);
    const AggregateAxiomResult *p_pass03Aggregate =
        findAggregate(report, AxiomCode::AX_PASS_03);
    ASSERT_NE(p_pass03Aggregate, nullptr);
    EXPECT_EQ(p_pass03Aggregate->result, AxiomResult::UNKNOWN);
}

/* Renamed from OneConfirmedEndpointOtherEmptyIsPass: see
 * TwoConfirmedReciprocalEndpointsIsUnknown. */
TEST(SemanticAxiomEvaluator, OneConfirmedEndpointOtherEmptyIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* A third live, confirmed, same-map room reverse-lists the passage even
 * though neither of the passage's own forward references name it. */
TEST(SemanticAxiomEvaluator, ThirdReverseOnlyConfirmedEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&far);
    Room thirdRoom;
    test::makeRoom(thirdRoom,
                   3,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&thirdRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    /* thirdRoom independently lists the same passage, but the passage's own
     * forward fields never name thirdRoom -- a reverse-only third endpoint. */
    ASSERT_EQ((thirdRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_THIRD_ENDPOINT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A reverse-only room that is itself RETIRED must be ignored outright --
 * "current-state axioms use live committed records... retired reverse-room
 * history alone must not poison a live passage." The original
 * BadReverseOnlyEndpointIsFail encoded the wrong current-state contract by
 * asserting FAIL for exactly this case; renamed and re-targeted to prove
 * the corrected behavior. */
TEST(SemanticAxiomEvaluator, RetiredReverseOnlyRoomDoesNotPoisonLivePassage)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room retiredRoom;
    test::makeRoom(retiredRoom,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&retiredRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retiredRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retiredRoom.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    /* No FAIL anomaly names the retired room at all; the passage settles at
     * its ordinary one-real-endpoint UNKNOWN, exactly as if retiredRoom's
     * passageRefs entry did not exist. */
    for (const Finding *p_finding_iter :
         findingsFor(report, AxiomCode::AX_PASS_02))
    {
        EXPECT_NE(p_finding_iter->result, AxiomResult::FAIL)
            << "a retired reverse-only room must not poison a live passage";
    }
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* The "bad" reverse-reference classification is about the reference's own
 * captured liveness reporting the passage as retired
 * (ReversePassageEndpointScan:: badReverseRoomKeys' own Doxygen), not about the
 * reverse room itself being retired (see
 * RetiredReverseOnlyRoomDoesNotPoisonLivePassage above). Since a room's
 * passageRef::isLive is read from the very same Passage pointer
 * PassageRecord::isLive itself derives from, the two can only disagree
 * through direct snapshot mutation -- the plan's own sanctioned technique
 * for a corrupt/collision state no public setter can represent. */
TEST(SemanticAxiomEvaluator, ReverseReferenceOwnLivenessBadIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room reverseOnly;
    test::makeRoom(reverseOnly,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&reverseOnly);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((reverseOnly.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    RoomRecord *p_reverseOnlyRecord = nullptr;
    for (RoomRecord &roomRecord : snapshot.maps.front().rooms)
    {
        if (roomRecord.key.entityId == 2)
        {
            p_reverseOnlyRecord = &roomRecord;
        }
    }
    ASSERT_NE(p_reverseOnlyRecord, nullptr);
    ASSERT_EQ(p_reverseOnlyRecord->passageRefs.size(), 1U);
    p_reverseOnlyRecord->passageRefs.front().isLive = false;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A live prospective (UNDEFINED-variant) reverse-only room does not become
 * a confirmed endpoint, but its own bad/cross-map/duplicate-identity
 * anomalies are still examined rather than silently discarded. */
TEST(SemanticAxiomEvaluator, LiveProspectiveReverseOnlyRoomAnomalyIsExamined)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();
    ASSERT_NE(p_mapA->getId(), p_mapB->getId());

    Room known;
    test::makeRoom(known, 1, p_mapA, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_mapA->addDetectedMapRoom(&known);
    Room prospectiveReverseOnly;
    test::makeRoom(prospectiveReverseOnly,
                   2,
                   p_mapB,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0),
                   Room::RoomVariant::UNDEFINED);
    p_mapB->addDetectedMapRoom(&prospectiveReverseOnly);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_mapA,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_mapA->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((prospectiveReverseOnly.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A reverse-only room merely fills an otherwise-empty forward slot: the
 * union of forward + reverse real endpoints still contains only two
 * distinct rooms, so this must not be misreported as a third endpoint. */
TEST(SemanticAxiomEvaluator, ReverseOnlyRoomFillingEmptySlotIsNotThirdEndpoint)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room farReverseOnly;
    test::makeRoom(farReverseOnly,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&farReverseOnly);

    Passage passage;
    /* Deliberately no prospective/far room on the forward side. */
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    /* farReverseOnly independently lists the passage back even though the
     * passage's own forward prospectiveRoomRef never names it. */
    ASSERT_EQ((farReverseOnly.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_thirdEndpointFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_THIRD_ENDPOINT);
    EXPECT_EQ(p_thirdEndpointFinding, nullptr)
        << "a reverse-only room filling an empty forward slot is not a "
           "third endpoint";
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* A same-map, live room's passageRefs entry shares this passage's map/
 * entity id but a different EntityKind: a wrong-kind key masquerading as a
 * reference to this passage. Manufactured via direct snapshot mutation --
 * RoomRecord::passageRefs is always PASSAGE-kind by construction, so no
 * public setter sequence can create this. */
TEST(SemanticAxiomEvaluator, ReverseWrongKindKeyIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room otherRoom;
    test::makeRoom(otherRoom,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&otherRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    RoomRecord *p_otherRoomRecord = nullptr;
    for (RoomRecord &roomRecord : snapshot.maps.front().rooms)
    {
        if (roomRecord.key.entityId == 2)
        {
            p_otherRoomRecord = &roomRecord;
        }
    }
    ASSERT_NE(p_otherRoomRecord, nullptr);
    EntityRef wrongKindRef;
    wrongKindRef.key    = EntityKey{EntityKind::ROOM, p_map->getId(), 1};
    wrongKindRef.reason = UnavailableReason::NONE;
    wrongKindRef.isLive = true;
    wrongKindRef.livenessUnavailableReason = UnavailableReason::NONE;
    p_otherRoomRecord->passageRefs.push_back(wrongKindRef);

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_WRONG_KIND);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* One live, same-map, ROOM-variant room's own passageRefs names this
 * passage more than once: relationship multiplicity a key-deduplication
 * pass must not silently erase to one clean reference. Manufactured via
 * direct snapshot mutation -- Room::setDoorways() de-duplicates by pointer,
 * so no public setter sequence can create this. */
TEST(SemanticAxiomEvaluator, ReverseReferenceDuplicatedIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    RoomRecord *p_knownRecord = nullptr;
    for (RoomRecord &roomRecord : snapshot.maps.front().rooms)
    {
        if (roomRecord.key.entityId == 1)
        {
            p_knownRecord = &roomRecord;
        }
    }
    ASSERT_NE(p_knownRecord, nullptr);
    ASSERT_EQ(p_knownRecord->passageRefs.size(), 1U);
    p_knownRecord->passageRefs.push_back(p_knownRecord->passageRefs.front());

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_REFERENCE_DUPLICATED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A forward (known-side) endpoint reference resolves to a key that more
 * than one distinct RoomRecord in the same map shares: which room actually
 * forms the endpoint is ambiguous. */
TEST(SemanticAxiomEvaluator, ForwardEndpointDuplicateIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    /* A second, distinct Room object deliberately reuses local id 1. */
    Room duplicateIdRoom;
    test::makeRoom(duplicateIdRoom,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&duplicateIdRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    /* Deliberately never call setDoorways() on either room sharing local id
     * 1, so the reverse scan's own duplicate-identity check (already
     * covered by PassageCardinalityDuplicateRoomIdentityIsFail) does not
     * fire first -- this isolates the FORWARD-endpoint duplicate-identity
     * path in resolveRoomEndpoint()/evaluateOnePassageCardinality(). */

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_FORWARD_ENDPOINT_DUPLICATE_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A forward (known-side) endpoint reference's key has an EntityKind other
 * than ROOM. Also proves the shared isKnownInvalidPassageEndpointReference()
 * predicate dominates AX-PASS-03 and AX-FLOOR-01's passage-floor-identity
 * clause too, not only AX-PASS-02's own cardinality check. Manufactured via
 * direct snapshot mutation -- entityRefForRoom() always keys ROOM. */
TEST(SemanticAxiomEvaluator, ForwardEndpointWrongKindIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    snapshot.maps.front().passages.front().knownSideRoomRef.key->kind =
        EntityKind::WALL;

    const AxiomEvaluationReport report               = evaluateState(snapshot);
    const Finding              *p_cardinalityFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_WRONG_KIND);
    ASSERT_NE(p_cardinalityFinding, nullptr);
    EXPECT_EQ(p_cardinalityFinding->result, AxiomResult::FAIL);

    const Finding *p_slotFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_03,
        ReasonCode::PASSAGE_SLOT_FORWARD_ENDPOINT_INVALID);
    ASSERT_NE(p_slotFinding, nullptr);
    EXPECT_EQ(p_slotFinding->result, AxiomResult::FAIL);

    const Finding *p_mapFloorFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_04,
        ReasonCode::PASSAGE_FLOOR_FORWARD_ENDPOINT_INVALID);
    ASSERT_NE(p_mapFloorFinding, nullptr);
    EXPECT_EQ(p_mapFloorFinding->result, AxiomResult::FAIL);

    const Finding *p_floorAxiomFinding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::FLOOR_PASSAGE_FORWARD_ENDPOINT_INVALID);
    ASSERT_NE(p_floorAxiomFinding, nullptr);
    EXPECT_EQ(p_floorAxiomFinding->result, AxiomResult::FAIL);
}

/* A forward endpoint reference resolves to a located RoomRecord whose own
 * declaredMapId disagrees with the map it was found in. Manufactured via
 * direct snapshot mutation. */
TEST(SemanticAxiomEvaluator, ForwardEndpointDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    snapshot.maps.front().rooms.front().declaredMapId = p_map->getId() + 999U;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A forward endpoint reference is present with no other known
 * contradiction, but its own EntityRef::isLive carries no value at all:
 * genuinely unproven liveness must be UNKNOWN, not the same as a known-bad
 * (isLive == false) endpoint, and must not be silently treated as
 * "no confirmed endpoint." Retargeted at a local id no RoomRecord in this
 * snapshot actually has (the same unlocatable-target technique
 * WallOwnerRecordUnavailableIsUnknown uses): resolveRoomEndpoint()
 * corroborates isLive from a *found* RoomRecord's own (always-populated)
 * isLive field, so a found room's liveness can never be genuinely
 * unavailable -- only an unenumerable-but-keyed target's can. Manufactured
 * via direct snapshot mutation. */
TEST(SemanticAxiomEvaluator, ForwardEndpointLivenessUnavailableIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    EntityRef &knownSideRef =
        snapshot.maps.front().passages.front().knownSideRoomRef;
    knownSideRef.key->entityId = 999;
    knownSideRef.isLive.reset();
    knownSideRef.livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_LIVENESS_UNAVAILABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
    EXPECT_EQ(findFindingWithReason(
                  report,
                  AxiomCode::AX_PASS_02,
                  ReasonCode::PASSAGE_CARDINALITY_NO_CONFIRMED_ENDPOINT),
              nullptr);
    EXPECT_EQ(
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_BAD),
        nullptr);
}

/* A same-map room's passageRefs entry cleanly names this passage with no
 * independently-known contradiction, but that entry's own EntityRef::isLive
 * carries no value: "missing liveness is unavailable, not live," so it must
 * neither be silently counted as a confirmed reciprocal endpoint nor treated
 * as a proven third-endpoint contradiction. Manufactured via direct
 * snapshot mutation. */
TEST(SemanticAxiomEvaluator, ReverseEndpointLivenessUnavailableIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room otherRoom;
    test::makeRoom(otherRoom,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&otherRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((otherRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    RoomRecord           *p_otherRoomRecord = nullptr;
    for (RoomRecord &roomRecord : snapshot.maps.front().rooms)
    {
        if (roomRecord.key.entityId == 2)
        {
            p_otherRoomRecord = &roomRecord;
        }
    }
    ASSERT_NE(p_otherRoomRecord, nullptr);
    ASSERT_EQ(p_otherRoomRecord->passageRefs.size(), 1U);
    p_otherRoomRecord->passageRefs.front().isLive.reset();
    p_otherRoomRecord->passageRefs.front().livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_LIVENESS_UNAVAILABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
    EXPECT_EQ(
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_THIRD_ENDPOINT),
        nullptr);
}

/* A clean, live prospective (non-ROOM-variant) room independently and
 * trustworthily lists a passage back via its own passageRefs, even though
 * the passage's own forward prospectiveRoomRef is absent. This reverse
 * prospective relationship must be represented by scanReversePassageEndpoints()
 * rather than silently discarded, and must not itself make the passage's
 * ordinary cardinality UNKNOWN become a false third-endpoint FAIL. */
TEST(SemanticAxiomEvaluator, CleanProspectiveReverseRelationshipIsRepresented)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room prospectiveRoom;
    test::makeRoom(prospectiveRoom,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0),
                   Room::RoomVariant::UNDEFINED);
    p_map->addDetectedMapRoom(&prospectiveRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((prospectiveRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    const ReversePassageEndpointScan scan =
        scanReversePassageEndpoints(snapshot.maps.front().passages.front(),
                                    p_map->getId(),
                                    snapshot);
    /* The prospective room's own clean reciprocal reference is represented,
     * not discarded. known's own reciprocal reference (the passage's
     * forward known-side room, which also independently lists the passage
     * back) still correctly appears in confirmedReverseRoomKeys -- this
     * scan reports every reverse relationship it finds, forward-named rooms
     * included; the caller reconciles that (see
     * evaluateOnePassageCardinality.cc's union computation). */
    ASSERT_EQ(scan.prospectiveReverseRoomKeys.size(), 1U);
    EXPECT_EQ(scan.prospectiveReverseRoomKeys.front(),
              (EntityKey{EntityKind::ROOM, p_map->getId(), 2}));
    ASSERT_EQ(scan.confirmedReverseRoomKeys.size(), 1U);
    EXPECT_EQ(scan.confirmedReverseRoomKeys.front(),
              (EntityKey{EntityKind::ROOM, p_map->getId(), 1}));

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* A reverse-only room lists the passage back, but that room is declared in a
 * different map than the passage. */
TEST(SemanticAxiomEvaluator, CrossMapReverseOnlyEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_otherMap = atlas.getCurrentMap();
    ASSERT_NE(p_map->getId(), p_otherMap->getId());

    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room crossMapRoom;
    test::makeRoom(crossMapRoom,
                   2,
                   p_otherMap,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_otherMap->addDetectedMapRoom(&crossMapRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((crossMapRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A same-map room's passageRefs entry shares this passage's local id but
 * carries no key at all: its own underlying pointer had no map at capture
 * time, so it cannot be safely counted as a real reverse endpoint either
 * way. Built with a bare Passage object that is never registered with any
 * map (test::makePassage always calls setMap()), so its own getMap()
 * returns nullptr and entityRefForPassage() cannot form a key for it. */
/* A room's own passageRefs entry carries a local id with no key (its
 * underlying Passage object was never registered with any map), and that
 * bare local id happens to equal an unrelated, genuinely evaluated
 * passage's own local id. This malformed reference must never be
 * attributed to that specific passage -- local ids are unique only within
 * one map and are not themselves a map-qualified identity (see
 * EntityKey.h) -- so the real passage's own cardinality remains its
 * ordinary one-real-endpoint UNKNOWN, while a separate, room-scoped
 * malformed-reference Finding (naming only the room, never the passage)
 * reports the evidence instead. Retargeted from
 * UnresolvableReverseEndpointIsFail, which encoded exactly the
 * bare-local-id-attribution defect described above. */
TEST(SemanticAxiomEvaluator,
     UnkeyedReverseReferenceIsRoomScopedNotPassageAttributed)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room roomWithDanglingRef;
    test::makeRoom(roomWithDanglingRef,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&roomWithDanglingRef);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    /* Never registered via AddMapPassage, so its own getMap() is nullptr,
     * but it shares the real passage's local id (1). */
    Passage unregisteredPassageWithSameId;
    ASSERT_EQ((unregisteredPassageWithSameId.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((roomWithDanglingRef.setDoorways(&unregisteredPassageWithSameId)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));

    const Finding *p_malformedFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ROOM_HAS_MALFORMED_REFERENCE);
    ASSERT_NE(p_malformedFinding, nullptr);
    EXPECT_EQ(p_malformedFinding->result, AxiomResult::FAIL);
    ASSERT_EQ(p_malformedFinding->involvedKeys.size(), 1U);
    EXPECT_EQ(p_malformedFinding->involvedKeys.front(),
              (EntityKey{EntityKind::ROOM, p_map->getId(), 2}));

    const Finding *p_cardinalityFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_cardinalityFinding, nullptr);
    EXPECT_EQ(p_cardinalityFinding->result, AxiomResult::UNKNOWN);
    /* The real passage's own finding must not name roomWithDanglingRef. */
    EXPECT_EQ(std::find(p_cardinalityFinding->involvedKeys.begin(),
                        p_cardinalityFinding->involvedKeys.end(),
                        EntityKey{EntityKind::ROOM, p_map->getId(), 2}),
              p_cardinalityFinding->involvedKeys.end());
}

/* Two distinct RoomRecord objects share the exact key a reverse-listing
 * room resolves to: which room actually forms the endpoint is ambiguous. */
TEST(SemanticAxiomEvaluator, PassageCardinalityDuplicateRoomIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    /* A second, distinct Room object deliberately reuses local id 1. */
    Room duplicateIdRoom;
    test::makeRoom(duplicateIdRoom,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&duplicateIdRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((duplicateIdRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ROOM_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, DuplicateEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  onlyRoom;
    test::makeRoom(onlyRoom, 1, p_map, nullptr, Eigen::Vector3d(0.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&onlyRoom);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &onlyRoom,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &onlyRoom);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((onlyRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ENDPOINT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, NonReciprocalEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    /* Deliberately never call known.setDoorways(&passage): the passage
     * names the room, but the room does not name the passage back. */

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Regression coverage: resolveRoomEndpoint() must
 * treat a referenced room's liveness (EntityRef::isLive, populated
 * directly from the pointer's own isBad() independent of enumeration) as
 * trustworthy even when this snapshot cannot locate that room's own
 * RoomRecord in any captured map's collection -- otherwise a bad/orphaned
 * reference on one side, paired with a genuinely valid confirmed room on
 * the other side, would incorrectly satisfy "at least one confirmed room"
 * and PASS rather than FAIL. */
TEST(SemanticAxiomEvaluator, BadUnenumeratedOtherSideEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    /* Declares the same map and is bad, but is deliberately never added to
     * any of p_map's room collections, so this snapshot cannot enumerate
     * its RoomRecord (isFoundInSnapshot == false) even though its own
     * EntityRef::isLive is still populated correctly from isBad(). */
    Room badUnenumerated;
    ASSERT_EQ((badUnenumerated.setId(99)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((badUnenumerated.setMap(p_map)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((badUnenumerated.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &badUnenumerated);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, KnownSideNotConfirmedIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  unpromoted;
    test::makeRoom(unpromoted,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0),
                   Room::RoomVariant::UNDEFINED);
    p_map->addDetectedMapRoom(&unpromoted);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &unpromoted,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((unpromoted.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_03,
        ReasonCode::PASSAGE_SLOT_KNOWN_SIDE_NOT_CONFIRMED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* EntityRef documents key.has_value() <=>
 * reason == NONE as an invariant, but snapshot records are adversarial value
 * inputs (see
 * resolveRoomEndpoint.cc/isKnownInvalidPassageEndpointReference.cc). A keyed
 * forward reference whose own reason is not NONE must fail AX-PASS-02,
 * AX-PASS-03, AX-PASS-04, and AX-FLOOR-01's passage branch, not flow through as
 * an ordinary valid reference. Manufactured via direct snapshot mutation,
 * mirroring ReverseReferenceDuplicatedIsFail's own precedent, since no public
 * setter sequence can violate this invariant. */
TEST(SemanticAxiomEvaluator,
     ReasonInconsistentForwardReferenceFailsEveryPassageAxiom)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    PassageRecord &passageRecord = snapshot.maps.front().passages.front();
    ASSERT_TRUE(passageRecord.knownSideRoomRef.key.has_value());
    /* Invariant-violating: key retained, but reason no longer NONE. */
    passageRecord.knownSideRoomRef.reason =
        UnavailableReason::ENTITY_HAS_NO_MAP;

    const AxiomEvaluationReport report = evaluateState(snapshot);

    const Finding *p_cardinalityFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_REASON_INCONSISTENT);
    ASSERT_NE(p_cardinalityFinding, nullptr);
    EXPECT_EQ(p_cardinalityFinding->result, AxiomResult::FAIL);

    const Finding *p_slotFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_03,
        ReasonCode::PASSAGE_SLOT_FORWARD_ENDPOINT_INVALID);
    ASSERT_NE(p_slotFinding, nullptr);
    EXPECT_EQ(p_slotFinding->result, AxiomResult::FAIL);

    const Finding *p_mapFloorFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_04,
        ReasonCode::PASSAGE_FLOOR_FORWARD_ENDPOINT_INVALID);
    ASSERT_NE(p_mapFloorFinding, nullptr);
    EXPECT_EQ(p_mapFloorFinding->result, AxiomResult::FAIL);

    const Finding *p_floorIdentityFinding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::FLOOR_PASSAGE_FORWARD_ENDPOINT_INVALID);
    ASSERT_NE(p_floorIdentityFinding, nullptr);
    EXPECT_EQ(p_floorIdentityFinding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, CrossMapPassageEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();

    Room roomInMapB;
    test::makeRoom(roomInMapB,
                   1,
                   p_mapB,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0));
    p_mapB->addDetectedMapRoom(&roomInMapB);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_mapA,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &roomInMapB,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_mapA->addMapPassage(&passage);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);

    const Finding *p_floorFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_04,
                              ReasonCode::PASSAGE_FLOOR_ENDPOINT_CROSS_MAP);
    ASSERT_NE(p_floorFinding, nullptr);
    EXPECT_EQ(p_floorFinding->result, AxiomResult::FAIL);

    /* AX-PASS-03 (slot state) and AX-FLOOR-01's
     * passage branch (endpoint floor identity) must fail on the identical
     * real-cross-map endpoint too, not only AX-PASS-02/04. */
    const Finding *p_slotFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_03,
                              ReasonCode::PASSAGE_SLOT_ENDPOINT_CROSS_MAP);
    ASSERT_NE(p_slotFinding, nullptr);
    EXPECT_EQ(p_slotFinding->result, AxiomResult::FAIL);

    const Finding *p_floorIdentityFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::FLOOR_PASSAGE_ENDPOINT_CROSS_MAP);
    ASSERT_NE(p_floorIdentityFinding, nullptr);
    EXPECT_EQ(p_floorIdentityFinding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, CrossFloorPassageIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&far);

    Floor floorA;
    test::makeFloor(floorA, 1, p_map, {&known}, 0.0);
    p_map->addMapFloor(&floorA);
    Floor floorB;
    test::makeFloor(floorB, 2, p_map, {&far}, 3.0);
    p_map->addMapFloor(&floorB);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_04,
                              ReasonCode::PASSAGE_FLOOR_DISAGREEMENT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);

    const Finding *p_floorAxiomFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::FLOOR_PASSAGE_CROSS_FLOOR);
    ASSERT_NE(p_floorAxiomFinding, nullptr);
    EXPECT_EQ(p_floorAxiomFinding->result, AxiomResult::FAIL);
}

/* The AX-PASS-04 aggregate must
 * remain UNKNOWN even when the map/floor clause itself genuinely agrees,
 * since PassageRecord::endpointSlotReason is always
 * NOT_TRACKED_BY_CURRENT_SCHEMA -- renamed from the original
 * SameFloorPassageIsPass, which inspected only the positive same-floor
 * finding without checking the aggregate. */
TEST(SemanticAxiomEvaluator, SameFloorPassageAgreesButAggregateIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&far);

    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far}, 0.0);
    p_map->addMapFloor(&floor);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_04,
                              ReasonCode::PASSAGE_FLOOR_AGREEMENT_VALID);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::PASS);

    const Finding *p_unverifiedFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_04,
        ReasonCode::PASSAGE_FLOOR_ENDPOINT_PROOF_UNVERIFIED);
    ASSERT_NE(p_unverifiedFinding, nullptr);
    EXPECT_EQ(p_unverifiedFinding->result, AxiomResult::UNKNOWN);

    const AggregateAxiomResult *p_aggregate =
        findAggregate(report, AxiomCode::AX_PASS_04);
    ASSERT_NE(p_aggregate, nullptr);
    EXPECT_EQ(p_aggregate->result, AxiomResult::UNKNOWN);

    /* The passage branch of AX-FLOOR-01 shares the same capped ceiling. */
    const Finding *p_floorAxiomFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::FLOOR_PASSAGE_AGREEMENT_VALID);
    ASSERT_NE(p_floorAxiomFinding, nullptr);
    EXPECT_EQ(p_floorAxiomFinding->result, AxiomResult::PASS);
    const Finding *p_floorUnverifiedFinding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::FLOOR_PASSAGE_ENDPOINT_PROOF_UNVERIFIED);
    ASSERT_NE(p_floorUnverifiedFinding, nullptr);
    EXPECT_EQ(p_floorUnverifiedFinding->result, AxiomResult::UNKNOWN);
}

TEST(SemanticAxiomEvaluator, MissingFloorEvidenceOnPassageIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&far);
    /* Neither room is linked to any Floor. */

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    /* Both endpoint rooms'
     * own canonical evaluateOneRoomFloorReciprocity() result is itself
     * UNKNOWN (ROOM_FLOOR_UNLINKED, neither room has a floor at all), which
     * now dominates before the floorKey-missing EVIDENCE_UNAVAILABLE
     * fallback is even reached -- a more specific diagnosis of the same
     * UNKNOWN outcome. */
    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_04,
        ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* ------------------------------------------------------------------------
 * AX-BOUND-01: room boundary completeness meaning
 * ---------------------------------------------------------------------- */

TEST(SemanticAxiomEvaluator, UnobservedBoundaryIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    vs_graphs::core::semantic::Room::BoundaryStatus boundaryStatus{};
    ASSERT_EQ((room.getBoundaryStatus(boundaryStatus)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(boundaryStatus, Room::BoundaryStatus::UNOBSERVED);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_NOT_YET_COMPLETE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

TEST(SemanticAxiomEvaluator, ConflictingBoundaryIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::CONFLICTING)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_CONFLICTING_STATE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, CompleteWithTooFewCornersIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m(
            {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_TOO_FEW_CORNERS);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, CompleteWithNoWallEvidenceIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_NO_WALL_EVIDENCE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, CompleteSelfIntersectingIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    /* A bowtie quadrilateral: edges (0->1) and (2->3) cross. */
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_SELF_INTERSECTING);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, CompleteWithObservationGapsIsUnknown)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setObservationGaps({Room::ObservationGap{0.0, 0.2}})),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_BOUND_01,
        ReasonCode::ROOM_BOUNDARY_GAP_CORRESPONDENCE_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* Renamed from CompleteStructurallyValidIsPass: full edge-to-wall
 * geometric correspondence is not implemented here, so even a simple, valid
 * polygon with genuinely live/reciprocal/same-map wall evidence is UNKNOWN,
 * never PASS -- see evaluateOneRoomBoundary.cc's own Doxygen.
 * ROOM_BOUNDARY_STRUCTURALLY_ VALID/PASS is never emitted by production code
 * here; it is retained for a future extension implementing full geometric
 * correspondence. */
TEST(SemanticAxiomEvaluator, CompleteWithVerifiedWallEvidenceIsUnknown)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_BOUND_01,
        ReasonCode::ROOM_BOUNDARY_EDGE_SUPPORT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* NaN/Infinity boundary corners must never flow through comparison-based
 * geometry logic to VALID. */
TEST(SemanticAxiomEvaluator, NonFiniteCornerIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setBoundaryCorners_World_m(
                  {Eigen::Vector3d(0.0, 0.0, 0.0),
                   Eigen::Vector3d(1.0, 0.0, 0.0),
                   Eigen::Vector3d(1.0,
                                   std::numeric_limits<double>::quiet_NaN(),
                                   0.0),
                   Eigen::Vector3d(0.0, 1.0, 0.0)})),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_NON_FINITE_CORNER);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Infinity, not only NaN, must also be rejected. */
TEST(SemanticAxiomEvaluator, InfiniteCornerIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m(
            {Eigen::Vector3d(0.0, 0.0, 0.0),
             Eigen::Vector3d(1.0, 0.0, 0.0),
             Eigen::Vector3d(1.0, std::numeric_limits<double>::infinity(), 0.0),
             Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_NON_FINITE_CORNER);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A live room's only wall reference is itself retired: a retired wall alone
 * does not poison current-state wall/completeness output (AX-WALL-01 skips
 * it), but a live room still referencing it is a distinct, observable
 * contradiction AX-BOUND-01 must catch through its own wall-evidence
 * check. */
TEST(SemanticAxiomEvaluator, LiveRoomReferencingRetiredWallCannotProveBoundary)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);
    wall.setBad();

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));

    /* The retired wall itself is skipped by AX-WALL-01 -- no Finding at
     * all names it. */
    for (const Finding &finding : report.findings)
    {
        if (finding.axiomCode == AxiomCode::AX_WALL_01)
        {
            for (const EntityKey &key : finding.involvedKeys)
            {
                EXPECT_FALSE(key.kind == EntityKind::WALL && key.entityId == 1)
                    << "AX-WALL-01 must skip a retired WallRecord";
            }
        }
    }

    /* A retired wall's evidence is
     * now typed INVALID (a known contradiction), not merely absent, so this
     * asserts ROOM_BOUNDARY_INVALID_WALL_EVIDENCE rather than
     * ROOM_BOUNDARY_NO_WALL_EVIDENCE -- renamed from
     * LiveRoomReferencingRetiredWallCannotProveBoundary's original
     * NO_WALL_EVIDENCE expectation to reflect the more precise
     * classification. */
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* One valid wall reference plus one provably wrong-type reference: the
 * known contradiction must dominate rather than being hidden behind the
 * other valid reference or silently reaching the Phase-6 support UNKNOWN.
 * Manufactured via direct snapshot mutation -- Room::setWalls() only ever
 * admits WALL-typed planes in ordinary operation. */
TEST(SemanticAxiomEvaluator,
     BoundaryOneValidPlusOneWrongTypeWallReferenceIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    RoomRecord *p_roomRecord = nullptr;
    for (RoomRecord &roomRecord : snapshot.maps.front().rooms)
    {
        if (roomRecord.key.entityId == 1)
        {
            p_roomRecord = &roomRecord;
        }
    }
    ASSERT_NE(p_roomRecord, nullptr);
    ASSERT_EQ(p_roomRecord->wallRefs.size(), 1U);
    RawPlaneRef wrongTypeRef = p_roomRecord->wallRefs.front();
    wrongTypeRef.planeType   = geometric::Plane::PlaneVariant::DOOR;
    wrongTypeRef.planeId     = 999;
    wrongTypeRef.wallKey.reset();
    p_roomRecord->wallRefs.push_back(wrongTypeRef);

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A wallRef whose target has no map at all (mapId absent, reason == NONE)
 * is UNAVAILABLE, not INVALID: same-map/reciprocity cannot be verified
 * either way, but nothing is provably wrong either. */
TEST(SemanticAxiomEvaluator,
     BoundaryUnmappedWallReferenceContributesOnlyUnknown)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    RoomRecord *p_roomRecord = nullptr;
    for (RoomRecord &roomRecord : snapshot.maps.front().rooms)
    {
        if (roomRecord.key.entityId == 1)
        {
            p_roomRecord = &roomRecord;
        }
    }
    ASSERT_NE(p_roomRecord, nullptr);
    ASSERT_EQ(p_roomRecord->wallRefs.size(), 1U);
    RawPlaneRef unmappedRef = p_roomRecord->wallRefs.front();
    unmappedRef.mapId.reset();
    unmappedRef.wallKey.reset();
    unmappedRef.planeId = 998;
    p_roomRecord->wallRefs.push_back(unmappedRef);

    const AxiomEvaluationReport report = evaluateState(snapshot);
    /* No FAIL: the unmapped entry is unavailable, not a proven
     * contradiction, and the genuinely valid wall still leaves the room at
     * the Phase-6 support UNKNOWN. */
    const Finding              *p_invalidFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE);
    EXPECT_EQ(p_invalidFinding, nullptr);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_BOUND_01,
        ReasonCode::ROOM_BOUNDARY_EDGE_SUPPORT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* A COMPLETE room's wallRefs collection is nonempty, but every entry is
 * genuinely unavailable (no map at all): this must remain UNKNOWN, not
 * collapse to the ROOM_BOUNDARY_NO_WALL_EVIDENCE contradiction reserved for
 * a genuinely empty collection. Manufactured via direct snapshot mutation. */
TEST(SemanticAxiomEvaluator, NonemptyAllUnavailableWallEvidenceIsUnknownNotFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot     = captureSemanticGraphSnapshot(&atlas);
    RoomRecord           *p_roomRecord = nullptr;
    for (RoomRecord &roomRecord : snapshot.maps.front().rooms)
    {
        if (roomRecord.key.entityId == 1)
        {
            p_roomRecord = &roomRecord;
        }
    }
    ASSERT_NE(p_roomRecord, nullptr);
    ASSERT_EQ(p_roomRecord->wallRefs.size(), 1U);
    /* Make the sole reference genuinely unavailable (no map at all) rather
     * than removing it, so the collection stays nonempty. */
    p_roomRecord->wallRefs.front().mapId.reset();
    p_roomRecord->wallRefs.front().wallKey.reset();

    const AxiomEvaluationReport report = evaluateState(snapshot);
    EXPECT_EQ(findFindingWithReason(report,
                                    AxiomCode::AX_BOUND_01,
                                    ReasonCode::ROOM_BOUNDARY_NO_WALL_EVIDENCE),
              nullptr);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_BOUND_01,
        ReasonCode::ROOM_BOUNDARY_WALL_EVIDENCE_UNAVAILABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* ------------------------------------------------------------------------
 * AX-FLOOR-01: room-floor reciprocity
 * ---------------------------------------------------------------------- */

TEST(SemanticAxiomEvaluator, RoomWithNoFloorIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_UNLINKED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

TEST(SemanticAxiomEvaluator, RoomFloorReciprocalIsPass)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_RECIPROCAL_VALID);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::PASS);
}

TEST(SemanticAxiomEvaluator, RoomFloorNonReciprocalIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {});
    p_map->addMapFloor(&floor);
    /* Room believes it has a floor; the floor does not list it back. */
    ASSERT_EQ((room.setFloor(&floor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_NON_RECIPROCAL);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, RoomFloorCrossMapIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();

    Room room;
    test::makeRoom(room, 1, p_mapA, nullptr);
    p_mapA->addDetectedMapRoom(&room);

    Floor floorInMapB;
    test::makeFloor(floorInMapB, 1, p_mapB, {});
    p_mapB->addMapFloor(&floorInMapB);
    ASSERT_EQ((room.setFloor(&floorInMapB)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A floor's own roomRefs reverse-claims a room that has no forward
 * floorRef naming any floor at all: a known contradiction, not the
 * ordinary "no floor yet" case. Manufactured via direct snapshot mutation
 * -- Floor::addRoom()/setRooms() always keep Room::getFloor() reciprocal in
 * ordinary operation. */
TEST(SemanticAxiomEvaluator, RoomFloorReverseClaimWithoutForwardLinkIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    ASSERT_TRUE(snapshot.maps.front().floors.front().roomRefs.empty());
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    const EntityKey &roomKey = snapshot.maps.front().rooms.front().key;
    EntityRef        reverseClaimRef;
    reverseClaimRef.key    = roomKey;
    reverseClaimRef.reason = UnavailableReason::NONE;
    snapshot.maps.front().floors.front().roomRefs.push_back(reverseClaimRef);
    /* room.floorRef remains absent -- captureRoom() never touched by this
     * mutation. */
    ASSERT_FALSE(snapshot.maps.front().rooms.front().floorRef.key.has_value());

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_REVERSE_CLAIM_WITHOUT_FORWARD_LINK);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The room's own floorRef key is not FLOOR-typed. Manufactured via direct
 * snapshot mutation -- RoomRecord::floorRef is always FLOOR-kind by
 * construction via entityRefForFloor(). */
TEST(SemanticAxiomEvaluator, RoomFloorWrongKindIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    ASSERT_TRUE(snapshot.maps.front().rooms.front().floorRef.key.has_value());
    snapshot.maps.front().rooms.front().floorRef.key->kind = EntityKind::ROOM;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_WRONG_KIND);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The named floor's own declaredMapId disagrees with its containing map.
 * Manufactured via direct snapshot mutation. */
TEST(SemanticAxiomEvaluator, RoomFloorDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    snapshot.maps.front().floors.front().declaredMapId =
        snapshot.maps.front().floors.front().key.mapId + 1U;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The room's own declaredMapId (not the floor's) disagrees with the
 * containing map used to enumerate and key it. Manufactured via direct
 * snapshot mutation. */
TEST(SemanticAxiomEvaluator, RoomFloorRoomDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    snapshot.maps.front().rooms.front().declaredMapId = p_map->getId() + 999U;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_ROOM_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Every other AX-FLOOR-01 room-floor clause is affirmatively satisfied, but
 * the room's own declaredMapId is genuinely absent: positive reciprocity
 * proof is capped at UNKNOWN, not PASS. Manufactured via direct snapshot
 * mutation. */
TEST(SemanticAxiomEvaluator, RoomFloorRoomDeclaredMapUnavailableCapsAtUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    snapshot.maps.front().rooms.front().declaredMapId.reset();

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_passFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_RECIPROCAL_VALID);
    ASSERT_NE(p_passFinding, nullptr);
    EXPECT_EQ(p_passFinding->result, AxiomResult::PASS);
    const Finding *p_unknownFinding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_ROOM_DECLARED_MAP_UNAVAILABLE);
    ASSERT_NE(p_unknownFinding, nullptr);
    EXPECT_EQ(p_unknownFinding->result, AxiomResult::UNKNOWN);
}

/* The uniquely resolved floor's own
 * declaredMapId being unavailable must cap room-floor proof at UNKNOWN too,
 * mirroring the room's own missing-declared-map cap above. */
TEST(SemanticAxiomEvaluator, RoomFloorFloorDeclaredMapUnavailableCapsAtUnknown)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    snapshot.maps.front().floors.front().declaredMapId.reset();

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_passFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_RECIPROCAL_VALID);
    ASSERT_NE(p_passFinding, nullptr);
    EXPECT_EQ(p_passFinding->result, AxiomResult::PASS);
    const Finding *p_unknownFinding2 = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_FLOOR_DECLARED_MAP_UNAVAILABLE);
    ASSERT_NE(p_unknownFinding2, nullptr);
    EXPECT_EQ(p_unknownFinding2->result, AxiomResult::UNKNOWN);
}

/* Two real passage endpoints share an equal floor key, but that key names
 * more than one distinct FloorRecord in the same map: identity ambiguity,
 * not proof of agreement. Manufactured via direct snapshot mutation. */
TEST(SemanticAxiomEvaluator,
     PassageFloorDuplicateFloorRecordFailsViaCanonicalRoomFloorProof)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&far);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far}, 0.0);
    p_map->addMapFloor(&floor);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    FloorRecord duplicateFloor = snapshot.maps.front().floors.front();
    snapshot.maps.front().floors.push_back(duplicateFloor);

    const AxiomEvaluationReport report = evaluateState(snapshot);
    /* evaluatePassageFloorAgreement()
     * now consults each real endpoint room's own canonical
     * evaluateOneRoomFloorReciprocity() result first; a duplicate floor key
     * makes namedFloorMatchCount > 1 for both endpoint rooms there,
     * yielding a dominant ENDPOINT_ROOM_FLOOR_INVALID before the
     * floorKey-equality AMBIGUOUS path is ever reached -- the underlying
     * contradiction is now diagnosed one layer earlier and more precisely. */
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_04,
        ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    const Finding *p_floorAxiomFinding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::FLOOR_PASSAGE_ENDPOINT_ROOM_FLOOR_INVALID);
    ASSERT_NE(p_floorAxiomFinding, nullptr);
    EXPECT_EQ(p_floorAxiomFinding->result, AxiomResult::FAIL);
}

/* Two real passage endpoints share an equal, unique floor key, but the
 * resolved floor does not reciprocally list one of the endpoint rooms in
 * its own roomRefs: reciprocity ambiguity distinct from a plain missing
 * link. Manufactured via direct snapshot mutation. */
TEST(SemanticAxiomEvaluator,
     PassageFloorNonReciprocalMembershipFailsViaCanonicalRoomFloorProof)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&far);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far}, 0.0);
    p_map->addMapFloor(&floor);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.front().roomRefs.size(), 2U);
    /* Remove one endpoint room's reverse membership while both rooms'
     * forward floorRef still names this same floor. */
    snapshot.maps.front().floors.front().roomRefs.pop_back();

    const AxiomEvaluationReport report = evaluateState(snapshot);
    /* The endpoint room missing
     * reverse membership now fails its own canonical
     * evaluateOneRoomFloorReciprocity() check (ROOM_FLOOR_NON_RECIPROCAL)
     * first, which evaluatePassageFloorAgreement() propagates as a
     * dominant ENDPOINT_ROOM_FLOOR_INVALID before the floorKey-equality
     * AMBIGUOUS path is ever reached. */
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_04,
        ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Two distinct FloorRecord objects share the exact key the room's own
 * floorRef names: which floor actually owns the reciprocal link is
 * ambiguous. Map::AddMapFloor() itself detects and reassigns a colliding
 * local id ("Floor ID collision ... reassigned"), so no public setter
 * sequence can create two live FloorRecords sharing one key; manufactured
 * directly on the captured value snapshot, the sanctioned technique for a
 * corrupt/collision case no setter can represent. */
TEST(SemanticAxiomEvaluator, RoomFloorDuplicateIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);

    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    FloorRecord duplicateFloor = snapshot.maps.front().floors.front();
    duplicateFloor.roomRefs.clear();
    snapshot.maps.front().floors.push_back(duplicateFloor);

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_DUPLICATE_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* The named floor's own roomRefs lists the room twice: reverse membership is
 * not a clean single reciprocal link. Floor::setRooms()/addRoom() keep
 * Room<->Floor strictly reciprocal and duplicate-free by construction (no
 * public setter can create this collision), so this is manufactured
 * directly on the captured value snapshot -- the sanctioned technique for a
 * corrupt/collision case no setter can represent. */
TEST(SemanticAxiomEvaluator, RoomFloorDuplicateReverseMembershipIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);

    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.front().roomRefs.size(), 1U);
    snapshot.maps.front().floors.front().roomRefs.push_back(
        snapshot.maps.front().floors.front().roomRefs.front());

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_DUPLICATE_REVERSE_MEMBERSHIP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* A second, distinct floor also lists the room in its own roomRefs, even
 * though the room's own floorRef names only one canonical floor. Same
 * sanctioned mutation technique as RoomFloorDuplicateReverseMembershipIsFail
 * -- Floor::setRooms() actively detaches a room from any prior floor, so no
 * setter sequence can leave two floors both genuinely claiming it. */
TEST(SemanticAxiomEvaluator, RoomClaimedByMultipleFloorsIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);

    Floor firstFloor;
    test::makeFloor(firstFloor, 1, p_map, {&room});
    p_map->addMapFloor(&firstFloor);
    Floor secondFloor;
    test::makeFloor(secondFloor, 2, p_map, {});
    p_map->addMapFloor(&secondFloor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 2U);
    FloorRecord *p_firstFloorRecord  = nullptr;
    FloorRecord *p_secondFloorRecord = nullptr;
    for (FloorRecord &floorRecord : snapshot.maps.front().floors)
    {
        if (floorRecord.key.entityId == 1)
        {
            p_firstFloorRecord = &floorRecord;
        }
        else if (floorRecord.key.entityId == 2)
        {
            p_secondFloorRecord = &floorRecord;
        }
    }
    ASSERT_NE(p_firstFloorRecord, nullptr);
    ASSERT_NE(p_secondFloorRecord, nullptr);
    ASSERT_EQ(p_firstFloorRecord->roomRefs.size(), 1U);
    p_secondFloorRecord->roomRefs.push_back(
        p_firstFloorRecord->roomRefs.front());

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_CLAIMED_BY_MULTIPLE_FLOORS);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* ------------------------------------------------------------------------
 * Map-completeness truth table, legacy reproduction and divergence
 * ---------------------------------------------------------------------- */

TEST(SemanticAxiomEvaluator, ZeroConfirmedRoomsMakesMapIncomplete)
{
    Atlas                                    atlas(0);
    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_NE(std::find(results.front().reasons.begin(),
                        results.front().reasons.end(),
                        ReasonCode::COMPLETENESS_ZERO_CONFIRMED_ROOMS),
              results.front().reasons.end());

    const AxiomEvaluationReport report =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    const Finding *p_compFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_COMP_01,
                              ReasonCode::COMPLETENESS_ZERO_CONFIRMED_ROOMS);
    ASSERT_NE(p_compFinding, nullptr);
    EXPECT_EQ(p_compFinding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, LiveProspectiveRoomMakesMapIncomplete)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  prospective;
    test::makeRoom(prospective,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d::Zero(),
                   Room::RoomVariant::UNDEFINED);
    p_map->addDetectedMapRoom(&prospective);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_EQ(results.front().prospectiveRoomCount, 1U);
}

/* Required repair #7/#6: a map-local duplicate same-key room must never let
 * any first-match proof elsewhere in this map be trusted, so completeness
 * is an unconditional FAIL. */
TEST(SemanticAxiomEvaluator, CompletenessDuplicateIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    /* A second, distinct Room object deliberately reuses local id 1:
     * Map::AddDetectedMapRoom(), unlike AddMapPlane()/AddMapFloor()/
     * AddMapPassage(), has no id-collision handling, so this is reachable
     * through the real capture path rather than requiring direct snapshot
     * mutation. */
    Room duplicateIdRoom;
    test::makeRoom(duplicateIdRoom, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&duplicateIdRoom);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_NE(std::find(results.front().reasons.begin(),
                        results.front().reasons.end(),
                        ReasonCode::COMPLETENESS_DUPLICATE_IDENTITY),
              results.front().reasons.end());
}

/* Required repair #7: a confirmed room always contributes a room-creation/
 * bootstrap provenance UNKNOWN, since RoomRecord::creationProvenanceReason
 * is always NOT_TRACKED_BY_CURRENT_SCHEMA. */
TEST(SemanticAxiomEvaluator, CompletenessRoomCreationProvenanceUnavailable)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::UNKNOWN);
    EXPECT_NE(
        std::find(
            results.front().reasons.begin(),
            results.front().reasons.end(),
            ReasonCode::COMPLETENESS_ROOM_CREATION_PROVENANCE_UNAVAILABLE),
        results.front().reasons.end());
}

/* Required repair #7: AX-PASS-03 (slot state) must independently drive
 * completeness, not only AX-PASS-02 cardinality. */
TEST(SemanticAxiomEvaluator, CompletenessPassageSlotStateFailureIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  unpromoted;
    test::makeRoom(unpromoted,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0),
                   Room::RoomVariant::UNDEFINED);
    p_map->addDetectedMapRoom(&unpromoted);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &unpromoted,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((unpromoted.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_NE(std::find(results.front().reasons.begin(),
                        results.front().reasons.end(),
                        ReasonCode::COMPLETENESS_PASSAGE_ENDPOINTS_INVALID),
              results.front().reasons.end());
}

/* Required repair #7: AX-PASS-04 (map/floor agreement) and the passage
 * branch of AX-FLOOR-01 must independently drive completeness. */
TEST(SemanticAxiomEvaluator, CompletenessCrossFloorPassageIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    p_map->addDetectedMapRoom(&far);

    Floor floorA;
    test::makeFloor(floorA, 1, p_map, {&known}, 0.0);
    p_map->addMapFloor(&floorA);
    Floor floorB;
    test::makeFloor(floorB, 2, p_map, {&far}, 3.0);
    p_map->addMapFloor(&floorB);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_NE(std::find(results.front().reasons.begin(),
                        results.front().reasons.end(),
                        ReasonCode::COMPLETENESS_PASSAGE_ENDPOINTS_INVALID),
              results.front().reasons.end());
    EXPECT_NE(std::find(results.front().reasons.begin(),
                        results.front().reasons.end(),
                        ReasonCode::COMPLETENESS_HARD_CONTRADICTION),
              results.front().reasons.end());
}

/* Two MapSnapshot entries share the exact same mapId: which one is
 * authoritative for that map is itself ambiguous, so both must FAIL
 * regardless of which position the duplicate occupies in
 * SemanticGraphSnapshot::maps. Not reachable through production capture
 * (Atlas maps have unique ids by construction); manufactured via direct
 * snapshot mutation as a deliberately mutable adversarial input. */
TEST(SemanticAxiomEvaluator,
     DuplicateMapIdentityIsFailRegardlessOfInsertionOrder)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    const SemanticGraphSnapshot baseSnapshot =
        captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(baseSnapshot.maps.size(), 1U);
    const MapSnapshot duplicateMap = baseSnapshot.maps.front();

    SemanticGraphSnapshot appendedSnapshot = baseSnapshot;
    appendedSnapshot.maps.push_back(duplicateMap);
    const std::vector<MapCompletenessResult> appendedResults =
        evaluateMapCompleteness(appendedSnapshot);
    ASSERT_EQ(appendedResults.size(), 2U);
    for (const MapCompletenessResult &result : appendedResults)
    {
        EXPECT_EQ(result.conservativeResult, AxiomResult::FAIL);
        EXPECT_NE(std::find(result.reasons.begin(),
                            result.reasons.end(),
                            ReasonCode::COMPLETENESS_DUPLICATE_MAP_IDENTITY),
                  result.reasons.end());
    }

    SemanticGraphSnapshot prependedSnapshot = baseSnapshot;
    prependedSnapshot.maps.insert(prependedSnapshot.maps.begin(), duplicateMap);
    const std::vector<MapCompletenessResult> prependedResults =
        evaluateMapCompleteness(prependedSnapshot);
    ASSERT_EQ(prependedResults.size(), 2U);
    for (const MapCompletenessResult &result : prependedResults)
    {
        EXPECT_EQ(result.conservativeResult, AxiomResult::FAIL);
        EXPECT_NE(std::find(result.reasons.begin(),
                            result.reasons.end(),
                            ReasonCode::COMPLETENESS_DUPLICATE_MAP_IDENTITY),
                  result.reasons.end());
    }
}

/* A live room's own passageRefs entry cannot be safely resolved (a local id
 * with no key): required repair item "malformed passage refs" in the
 * completeness adversarial matrix. */
TEST(SemanticAxiomEvaluator, CompletenessRoomHasMalformedPassageReferenceIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    /* Never registered via AddMapPassage, so its own getMap() is nullptr. */
    Passage unregisteredPassage;
    ASSERT_EQ((unregisteredPassage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((room.setDoorways(&unregisteredPassage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_NE(
        std::find(
            results.front().reasons.begin(),
            results.front().reasons.end(),
            ReasonCode::COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE),
        results.front().reasons.end());
}

/* Renamed from FullyValidMapIsCompletePass: this is the strongest map this
 * schema can currently construct -- two reciprocal COMPLETE rooms, a
 * passage with two live confirmed reciprocal same-floor endpoints, no
 * contradiction anywhere -- and it is still only UNKNOWN, never PASS:
 * passage endpoint slot proof (PassageRecord::endpointSlotReason) and full
 * boundary edge-to-wall support are both permanently unavailable in this
 * schema. "No PASS
 * fixture until every required positive proof represented by the current
 * schema is actually present" -- this is the honest schema-limited ceiling,
 * not a defect. */
TEST(SemanticAxiomEvaluator,
     LegacyCompleteButConservativeUnknownSchemaLimitedDiverges)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wallA);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 5.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wallB);

    Room known;
    test::makeRoom(known, 1, p_map, &wallA, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((known.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (known.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 1.0, 0.0),
                                           Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&known);

    Room far;
    test::makeRoom(far, 2, p_map, &wallB, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((far.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (far.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 6.0, 0.0),
                                         Eigen::Vector3d(0.0, 6.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&far);

    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far});
    p_map->addMapFloor(&floor);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(snapshot);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::UNKNOWN);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_FALSE(results.front().reasons.empty());
    EXPECT_NE(
        std::find(results.front().reasons.begin(),
                  results.front().reasons.end(),
                  ReasonCode::COMPLETENESS_PASSAGE_SLOT_PROOF_UNAVAILABLE),
        results.front().reasons.end());
    EXPECT_EQ(results.front().confirmedRoomCount, 2U);
    EXPECT_EQ(results.front().completeRoomCount, 2U);
    EXPECT_EQ(results.front().fullyValidPassageCount, 0U);
    EXPECT_TRUE(results.front().legacy.isMapFullyModeled);
    EXPECT_TRUE(results.front().doLegacyAndConservativeDiverge);
}

TEST(SemanticAxiomEvaluator, LegacyReproducesDoubleRegisteredRoomMultiplicity)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  bothCollections;
    test::makeRoom(bothCollections, 1, p_map, nullptr);
    ASSERT_EQ(
        (bothCollections.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bothCollections.setBoundaryCorners_World_m(
                  {Eigen::Vector3d(0.0, 0.0, 0.0),
                   Eigen::Vector3d(1.0, 0.0, 0.0),
                   Eigen::Vector3d(1.0, 1.0, 0.0),
                   Eigen::Vector3d(0.0, 1.0, 0.0)})),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&bothCollections);
    p_map->addCandidateMapRoom(&bothCollections);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    /* Legacy counts the same live Room twice (once per GetAllRooms()
     * collection membership); the conservative calculation counts it
     * once. */
    EXPECT_EQ(results.front().legacy.confirmedRoomCount, 2U);
    EXPECT_EQ(results.front().legacy.completeRoomCount, 2U);
    EXPECT_EQ(results.front().confirmedRoomCount, 1U);
    EXPECT_EQ(results.front().completeRoomCount, 1U);
}

TEST(SemanticAxiomEvaluator,
     LegacyCompleteButConservativeFailsOnNonReciprocalPassageDiverges)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((known.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (known.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 1.0, 0.0),
                                           Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((far.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (far.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 6.0, 0.0),
                                         Eigen::Vector3d(0.0, 6.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&far);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far});
    p_map->addMapFloor(&floor);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    /* Legacy's own completeness check never inspects Room::getPassages()
     * reciprocity -- only the live known/far room pointers -- so leaving
     * both setDoorways() calls out reproduces a fixture that is legacy-
     * complete but conservatively invalid (non-reciprocal). */

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_TRUE(results.front().legacy.isMapFullyModeled);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_TRUE(results.front().doLegacyAndConservativeDiverge);
}

TEST(SemanticAxiomEvaluator,
     LegacyCompleteButConservativeUnknownDueToMissingFloorEvidenceDiverges)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wallA);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 5.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wallB);

    Room known;
    test::makeRoom(known, 1, p_map, &wallA, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((known.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (known.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 1.0, 0.0),
                                           Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&known);
    Room far;
    test::makeRoom(far, 2, p_map, &wallB, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((far.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (far.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 6.0, 0.0),
                                         Eigen::Vector3d(0.0, 6.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&far);
    /* Deliberately no Floor at all: legacy does not check floor linkage. */

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 1U);
    EXPECT_TRUE(results.front().legacy.isMapFullyModeled);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::UNKNOWN);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_TRUE(results.front().doLegacyAndConservativeDiverge);
}

/* A hard contradiction (multiple wall owners) in mapA must not affect
 * mapB's own, otherwise-empty (and therefore separately incomplete)
 * completeness result -- required repair #5, "no finding from map B may
 * affect map A" applied in the other direction. */
TEST(SemanticAxiomEvaluator, HardFailureInOneMapDoesNotContaminateAnotherMap)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();
    ASSERT_NE(p_mapA->getId(), p_mapB->getId());

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_mapA->addMapPlane(&wall);
    Room ownerOne;
    test::makeRoom(ownerOne, 1, p_mapA, &wall);
    p_mapA->addDetectedMapRoom(&ownerOne);
    Room ownerTwo;
    test::makeRoom(ownerTwo, 2, p_mapA, &wall);
    p_mapA->addDetectedMapRoom(&ownerTwo);

    /* mapB has no rooms at all: independently incomplete (zero confirmed
     * rooms), never FAIL for mapA's wall-ownership reason. */

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(results.size(), 2U);

    const MapCompletenessResult *p_mapAResult = nullptr;
    const MapCompletenessResult *p_mapBResult = nullptr;
    for (const MapCompletenessResult &result : results)
    {
        if (result.mapId == p_mapA->getId())
        {
            p_mapAResult = &result;
        }
        else if (result.mapId == p_mapB->getId())
        {
            p_mapBResult = &result;
        }
    }
    ASSERT_NE(p_mapAResult, nullptr);
    ASSERT_NE(p_mapBResult, nullptr);

    EXPECT_EQ(p_mapAResult->conservativeResult, AxiomResult::FAIL);
    EXPECT_NE(std::find(p_mapAResult->reasons.begin(),
                        p_mapAResult->reasons.end(),
                        ReasonCode::COMPLETENESS_HARD_CONTRADICTION),
              p_mapAResult->reasons.end());

    EXPECT_EQ(p_mapBResult->conservativeResult, AxiomResult::FAIL);
    EXPECT_EQ(p_mapBResult->reasons.size(), 1U);
    EXPECT_EQ(p_mapBResult->reasons.front(),
              ReasonCode::COMPLETENESS_ZERO_CONFIRMED_ROOMS);
}

/* ------------------------------------------------------------------------
 * Adversarial regression tests. Each mirrors a specific counterexample
 * against an earlier source revision.
 * ---------------------------------------------------------------------- */

/* A duplicate MapSnapshot::mapId splitting a room and its floor across two
 * floor across two MapSnapshot entries must never null-dereference through
 * evaluateState(), and must never grant positive (PASS) proof either. */
TEST(SemanticAxiomEvaluator,
     DuplicateContainingMapRoomFloorSplitDoesNotCrashAndIsNotPass)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_mapA, nullptr);
    p_mapA->addDetectedMapRoom(&room);

    atlas.createNewMap();
    Map  *p_mapB = atlas.getCurrentMap();
    Floor floor;
    test::makeFloor(floor, 1, p_mapB, {});
    p_mapB->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 2U);
    MapSnapshot *p_roomMap  = nullptr;
    MapSnapshot *p_floorMap = nullptr;
    for (MapSnapshot &mapSnapshot : snapshot.maps)
    {
        if (!mapSnapshot.rooms.empty())
        {
            p_roomMap = &mapSnapshot;
        }
        if (!mapSnapshot.floors.empty())
        {
            p_floorMap = &mapSnapshot;
        }
    }
    ASSERT_NE(p_roomMap, nullptr);
    ASSERT_NE(p_floorMap, nullptr);
    ASSERT_NE(p_roomMap, p_floorMap);

    /* Adversarial-only (unreachable through production capture): force both
     * MapSnapshots to share one mapId and link the room's forward floorRef
     * into the sibling duplicate map's floor. */
    const long unsigned int sharedMapId  = p_roomMap->mapId;
    p_floorMap->mapId                    = sharedMapId;
    p_floorMap->floors.front().key.mapId = sharedMapId;
    p_roomMap->rooms.front().floorRef.key =
        EntityKey{EntityKind::FLOOR, sharedMapId, 1};
    p_roomMap->rooms.front().floorRef.reason = UnavailableReason::NONE;
    ASSERT_EQ(countMapSnapshotsWithId(snapshot, sharedMapId), 2U);

    const AxiomEvaluationReport report = evaluateState(snapshot);
    for (const Finding *p_finding : findingsFor(report, AxiomCode::AX_FLOOR_01))
    {
        EXPECT_NE(p_finding->result, AxiomResult::PASS);
    }
}

/* Checkpoint 2: a forward endpoint whose own EntityRef::isLive is genuinely
 * unavailable must stay UNKNOWN even though the target RoomRecord it
 * resolves to is enumerated and live -- the found record must never
 * manufacture the reference-level liveness it does not itself carry. */
TEST(SemanticAxiomEvaluator,
     ForwardEndpointLivenessUnavailableStaysUnknownDespiteLiveEnumeratedRecord)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    PassageRecord &passageRecord = snapshot.maps.front().passages.front();
    ASSERT_TRUE(passageRecord.knownSideRoomRef.isLive.has_value());
    passageRecord.knownSideRoomRef.isLive = std::nullopt;
    passageRecord.knownSideRoomRef.livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_LIVENESS_UNAVAILABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
    EXPECT_EQ(
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL),
        nullptr);
}

/* Checkpoint 4: a room's own passageRefs entry naming this passage but
 * carrying reason != NONE (an EntityRef invariant violation) must not
 * silently supply reciprocity proof. */
TEST(SemanticAxiomEvaluator,
     ReasonInconsistentReciprocalPassageRefIsNotReciprocityProof)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    /* Deliberately never call known.setDoorways(): the adversarial
     * passageRefs entry below is hand-mutated instead. */

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    ASSERT_TRUE(snapshot.maps.front().rooms.front().passageRefs.empty());
    EntityRef malformedRef;
    malformedRef.key    = EntityKey{EntityKind::PASSAGE, p_map->getId(), 1};
    malformedRef.reason = UnavailableReason::ENTITY_HAS_NO_MAP;
    malformedRef.isLive = true;
    malformedRef.livenessUnavailableReason = UnavailableReason::NONE;
    snapshot.maps.front().rooms.front().passageRefs.push_back(malformedRef);

    /* scanReversePassageEndpoints() (also fixed this round to check
     * passageRef.reason) classifies this exact entry as a bad reverse
     * reference before roomListsPassageBack() is ever reached from
     * evaluateOnePassageCardinality() -- both independently reject it, but
     * the reverse-scan FAIL fires first. Either dominant path proves the
     * required property: a malformed reciprocal reference can never supply
     * reciprocity proof. */
    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    EXPECT_EQ(findFindingWithReason(
                  report,
                  AxiomCode::AX_PASS_02,
                  ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED),
              nullptr);
}

/* Checkpoint 4 (roomListsPassageBack-specific): the passage this reciprocity
 * claim is about must itself resolve to exactly one live passage record --
 * a duplicate passage identity is not something scanReversePassageEndpoints()
 * checks at all, so this isolates roomListsPassageBack()'s own dedicated
 * guard. */
TEST(SemanticAxiomEvaluator,
     ReciprocityAgainstDuplicatePassageIdentityIsNotProof)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    p_map->addDetectedMapRoom(&known);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    p_map->addMapPassage(&passage);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    /* Map::AddMapPassage() itself rejects a second Passage object reusing
     * local id 1 ("Passage ID collision"), so this ambiguity is
     * manufactured via direct snapshot mutation instead, mirroring
     * ReverseReferenceDuplicatedIsFail's own precedent: a second,
     * byte-identical PassageRecord sharing this exact key. */
    snapshot.maps.front().passages.push_back(
        snapshot.maps.front().passages.front());

    const AxiomEvaluationReport        report = evaluateState(snapshot);
    /* Both PassageRecords share one local id, so countPassageRecordsWithKey()
     * returns 2 for either's own key; roomListsPassageBack() must therefore
     * refuse reciprocity proof for both, yielding a genuine FAIL rather than
     * the ordinary schema-limited UNKNOWN
     * (PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED) an unambiguous,
     * reciprocal passage would otherwise receive. */
    const std::vector<const Finding *> findings =
        findingsFor(report, AxiomCode::AX_PASS_02);
    ASSERT_EQ(findings.size(), 2U);
    for (const Finding *p_finding : findings)
    {
        EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
        EXPECT_EQ(p_finding->reasonCode,
                  ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL);
    }
}

/* Checkpoint 5: missing owner-reference liveness must not mask an
 * independently known cross-map owner. */
TEST(SemanticAxiomEvaluator, OwnerLivenessUnavailableDoesNotMaskCrossMapOwner)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_mapA->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_mapB, &wall);
    p_mapB->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot     = captureSemanticGraphSnapshot(&atlas);
    WallRecord           *p_wallRecord = nullptr;
    for (MapSnapshot &mapSnapshot : snapshot.maps)
    {
        if (!mapSnapshot.walls.empty())
        {
            p_wallRecord = &mapSnapshot.walls.front();
        }
    }
    ASSERT_NE(p_wallRecord, nullptr);
    ASSERT_EQ(p_wallRecord->ownerRoomRefs.size(), 1U);
    p_wallRecord->ownerRoomRefs.front().isLive = std::nullopt;
    p_wallRecord->ownerRoomRefs.front().livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Checkpoint 6: the owner EntityRef's own reason must be validated, and a
 * reciprocal RawPlaneRef sharing this wall's identity but carrying a
 * non-NONE reason must fail rather than being silently skipped. */
TEST(SemanticAxiomEvaluator,
     OwnerReasonInconsistentIsFailDespiteValidReciprocal)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot   = captureSemanticGraphSnapshot(&atlas);
    WallRecord           &wallRecord = snapshot.maps.front().walls.front();
    ASSERT_EQ(wallRecord.ownerRoomRefs.size(), 1U);
    wallRecord.ownerRoomRefs.front().reason =
        UnavailableReason::ENTITY_HAS_NO_MAP;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_REASON_INCONSISTENT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

TEST(SemanticAxiomEvaluator, ContradictoryReciprocalWithNonNoneReasonIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot   = captureSemanticGraphSnapshot(&atlas);
    RoomRecord           &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    /* Shares this wall's own mapId/planeId identity but claims "absent". */
    RawPlaneRef contradictory = roomRecord.wallRefs.front();
    contradictory.reason      = UnavailableReason::NULL_REFERENCE;
    roomRecord.wallRefs.push_back(contradictory);

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* D1: a genuinely absent owner declaredMapId is missing evidence, not a
 * proven mismatch -- must cap at UNKNOWN, not fabricate a MISMATCH FAIL. */
TEST(SemanticAxiomEvaluator, OwnerDeclaredMapUnavailableCapsAtUnknownNotFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot   = captureSemanticGraphSnapshot(&atlas);
    RoomRecord           &roomRecord = snapshot.maps.front().rooms.front();
    roomRecord.declaredMapId.reset();

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_passFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER);
    ASSERT_NE(p_passFinding, nullptr);
    EXPECT_EQ(p_passFinding->result, AxiomResult::PASS);
    const Finding *p_unknownFinding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_UNAVAILABLE);
    ASSERT_NE(p_unknownFinding, nullptr);
    EXPECT_EQ(p_unknownFinding->result, AxiomResult::UNKNOWN);
    EXPECT_EQ(findFindingWithReason(
                  report,
                  AxiomCode::AX_WALL_01,
                  ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH),
              nullptr);
}

/* Checkpoint 7: a RawPlaneRef whose reason claims "absent" but whose data is
 * populated and contradictory must be INVALID (a known contradiction), not
 * UNAVAILABLE. */
TEST(SemanticAxiomEvaluator,
     BoundaryRawRefReasonInconsistentIsInvalidNotUnavailable)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot   = captureSemanticGraphSnapshot(&atlas);
    RoomRecord           &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_TRUE(roomRecord.wallRefs.empty());
    RawPlaneRef contradictory;
    contradictory.reason    = UnavailableReason::NULL_REFERENCE;
    contradictory.mapId     = p_map->getId();
    contradictory.planeId   = 5;
    contradictory.isLive    = false;
    contradictory.planeType = geometric::Plane::PlaneVariant::GROUND;
    roomRecord.wallRefs.push_back(contradictory);

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Checkpoint 7: an owning wall whose own AX-WALL-01 aggregate is PASS
 * accompanied by an UNKNOWN (declared map genuinely absent) must not be
 * reused as VALID boundary evidence. */
TEST(SemanticAxiomEvaluator,
     BoundaryEvidenceFromPassPlusUnknownOwnerIsUnavailable)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_World_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    snapshot.maps.front().rooms.front().declaredMapId.reset();

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_BOUND_01,
        ReasonCode::ROOM_BOUNDARY_WALL_EVIDENCE_UNAVAILABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/* D2: a wall-twin RawPlaneRef whose reason claims "absent" but whose data is
 * populated (here, a dead self-twin) must fail, not silently PASS as
 * WALL_TWIN_ABSENT. */
TEST(SemanticAxiomEvaluator, WallTwinReasonInconsistentIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);

    SemanticGraphSnapshot snapshot   = captureSemanticGraphSnapshot(&atlas);
    WallRecord           &wallRecord = snapshot.maps.front().walls.front();
    ASSERT_EQ(wallRecord.twinRef.reason, UnavailableReason::NULL_REFERENCE);
    wallRecord.twinRef.mapId     = wallRecord.key.mapId;
    wallRecord.twinRef.planeId   = wallRecord.key.entityId;
    wallRecord.twinRef.planeType = geometric::Plane::PlaneVariant::WALL;
    wallRecord.twinRef.isLive    = false;
    wallRecord.twinRef.wallKey   = wallRecord.key;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_REASON_INCONSISTENT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* Checkpoint 8: a floor's reverse roomRefs entry naming this room but
 * carrying reason != NONE and known-false liveness must not become PASS. */
TEST(SemanticAxiomEvaluator, FloorReverseMemberReasonInconsistentAndDeadIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    FloorRecord &floorRecord = snapshot.maps.front().floors.front();
    ASSERT_EQ(floorRecord.roomRefs.size(), 1U);
    floorRecord.roomRefs.front().reason = UnavailableReason::ENTITY_HAS_NO_MAP;
    floorRecord.roomRefs.front().isLive = false;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_REVERSE_MEMBER_INVALID);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    EXPECT_EQ(findFindingWithReason(report,
                                    AxiomCode::AX_FLOOR_01,
                                    ReasonCode::ROOM_FLOOR_RECIPROCAL_VALID),
              nullptr);
}

/* Checkpoint 11b: a wall in one MapSnapshot whose owner room lives in a
 * sibling MapSnapshot sharing the same (duplicated) mapId must never gain
 * positive ownership proof from a first-match lookup, regardless of which
 * duplicate happens to enumerate first. */
TEST(SemanticAxiomEvaluator, WallOwnerSplitAcrossDuplicateContainingMapIsFail)
{
    Atlas            atlas(0);
    Map             *p_mapA = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_mapA->addMapPlane(&wall);

    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();
    Room room;
    test::makeRoom(room, 1, p_mapB, nullptr);
    p_mapB->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 2U);
    MapSnapshot *p_wallMap = nullptr;
    MapSnapshot *p_roomMap = nullptr;
    for (MapSnapshot &mapSnapshot : snapshot.maps)
    {
        if (!mapSnapshot.walls.empty())
        {
            p_wallMap = &mapSnapshot;
        }
        if (!mapSnapshot.rooms.empty())
        {
            p_roomMap = &mapSnapshot;
        }
    }
    ASSERT_NE(p_wallMap, nullptr);
    ASSERT_NE(p_roomMap, nullptr);

    /* Adversarial-only: force both to share one mapId, then hand-wire the
     * wall's owner reference to the room now living in the "duplicate". */
    const long unsigned int sharedMapId = p_wallMap->mapId;
    p_roomMap->mapId                    = sharedMapId;
    p_roomMap->rooms.front().key.mapId  = sharedMapId;
    p_wallMap->walls.front().key.mapId  = sharedMapId;
    EntityRef ownerRef;
    ownerRef.key    = EntityKey{EntityKind::ROOM, sharedMapId, 1};
    ownerRef.reason = UnavailableReason::NONE;
    ownerRef.isLive = true;
    ownerRef.livenessUnavailableReason     = UnavailableReason::NONE;
    p_wallMap->walls.front().ownerRoomRefs = {ownerRef};

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_CONTAINING_MAP_AMBIGUOUS);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    EXPECT_EQ(
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER),
        nullptr);
}

/* ------------------------------------------------------------------------
 * Reference-liveness versus record-contradiction ordering.
 * ---------------------------------------------------------------------- */

/* Reference-level liveness genuinely unavailable must not mask a
 * record-level contradiction (here, the resolved owner RoomRecord is itself
 * retired). */
TEST(SemanticAxiomEvaluator,
     OwnerRecordNotLiveDominatesReferenceLivenessUnavailable)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);
    ASSERT_EQ((room.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot   = captureSemanticGraphSnapshot(&atlas);
    WallRecord           &wallRecord = snapshot.maps.front().walls.front();
    ASSERT_EQ(wallRecord.ownerRoomRefs.size(), 1U);
    wallRecord.ownerRoomRefs.front().isLive = std::nullopt;
    wallRecord.ownerRoomRefs.front().livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_NOT_LIVE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    EXPECT_EQ(findFindingWithReason(
                  report,
                  AxiomCode::AX_WALL_01,
                  ReasonCode::WALL_OWNERSHIP_OWNER_LIVENESS_UNAVAILABLE),
              nullptr);
}

/* A reciprocal RawPlaneRef whose own wallKey names
 * this wall but whose own raw mapId contradicts that same wallKey is an
 * internal inconsistency the relevance gate must still catch, not silently
 * treat as an unrelated reference. */
TEST(SemanticAxiomEvaluator, ReciprocalWallKeyRawIdentityMismatchIsFail)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot   = captureSemanticGraphSnapshot(&atlas);
    RoomRecord           &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    RawPlaneRef mismatched = roomRecord.wallRefs.front();
    ASSERT_TRUE(mismatched.wallKey.has_value());
    mismatched.mapId = 4242UL;
    roomRecord.wallRefs.push_back(mismatched);

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    EXPECT_EQ(
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER),
        nullptr);
}

/* Checkpoint 8 (regression fix): a floor reverse-claiming this room twice --
 * once cleanly live, once with liveness genuinely unavailable -- is still
 * ambiguous multiplicity, not a clean single reciprocal member. */
TEST(SemanticAxiomEvaluator,
     FloorReverseDuplicateWithOneLivenessUnavailableIsFail)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  room;
    test::makeRoom(room, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&room);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    p_map->addMapFloor(&floor);

    SemanticGraphSnapshot snapshot    = captureSemanticGraphSnapshot(&atlas);
    FloorRecord          &floorRecord = snapshot.maps.front().floors.front();
    ASSERT_EQ(floorRecord.roomRefs.size(), 1U);
    EntityRef unavailableDuplicate = floorRecord.roomRefs.front();
    unavailableDuplicate.isLive    = std::nullopt;
    unavailableDuplicate.livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;
    floorRecord.roomRefs.push_back(unavailableDuplicate);

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_DUPLICATE_REVERSE_MEMBERSHIP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    EXPECT_EQ(findFindingWithReason(report,
                                    AxiomCode::AX_FLOOR_01,
                                    ReasonCode::ROOM_FLOOR_RECIPROCAL_VALID),
              nullptr);
}

/* The owner-record declaredMapId mismatch FAIL check is itself
 * record-level and must be checked before the reference-liveness-
 * unavailable UNKNOWN, not after it -- otherwise a wall whose owner
 * reference liveness is merely unproven but whose resolved owner record
 * provably declares a different map is masked behind an UNKNOWN. */
TEST(SemanticAxiomEvaluator,
     OwnerDeclaredMapMismatchDominatesReferenceLivenessUnavailable)
{
    Atlas atlas(0);
    Map  *p_mapA = atlas.getCurrentMap();
    atlas.createNewMap();
    Map *p_mapB = atlas.getCurrentMap();

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_mapA->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_mapA, &wall);
    p_mapA->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.size(), 2U);
    WallRecord *p_wallRecord = nullptr;
    RoomRecord *p_roomRecord = nullptr;
    for (MapSnapshot &mapSnapshot : snapshot.maps)
    {
        if (!mapSnapshot.walls.empty())
        {
            p_wallRecord = &mapSnapshot.walls.front();
        }
        if (!mapSnapshot.rooms.empty())
        {
            p_roomRecord = &mapSnapshot.rooms.front();
        }
    }
    ASSERT_NE(p_wallRecord, nullptr);
    ASSERT_NE(p_roomRecord, nullptr);

    /* The owner record itself provably declares a different map than the
     * one it was actually enumerated from/owns the wall in. */
    p_roomRecord->declaredMapId = p_mapB->getId();
    ASSERT_EQ(p_wallRecord->ownerRoomRefs.size(), 1U);
    p_wallRecord->ownerRoomRefs.front().isLive = std::nullopt;
    p_wallRecord->ownerRoomRefs.front().livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    const AxiomEvaluationReport report    = evaluateState(snapshot);
    const Finding              *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    EXPECT_EQ(findFindingWithReason(
                  report,
                  AxiomCode::AX_WALL_01,
                  ReasonCode::WALL_OWNERSHIP_OWNER_LIVENESS_UNAVAILABLE),
              nullptr);
}

/* The reciprocal-scan FAILs depend only on p_owner->wallRefs and
 * wall_in.key (both already resolved), never on owner.isLive, so a
 * non-reciprocal owner must FAIL even when the owner reference's own
 * liveness is genuinely unproven -- not be masked behind that UNKNOWN. */
TEST(SemanticAxiomEvaluator,
     OwnerNotReciprocalDominatesReferenceLivenessUnavailable)
{
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    p_map->addMapPlane(&wall);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&room);

    SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    WallRecord &wallRecord = snapshot.maps.front().walls.front();
    ASSERT_EQ(wallRecord.ownerRoomRefs.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().rooms.front().wallRefs.size(), 1U);
    /* WallRecord::ownerRoomRefs is derived entirely from the ROOM side
     * (RoomRecord::wallRefs, inverted at capture -- see
     * captureSemanticGraphSnapshot.cc's wallOwnersByPointer), so a genuine,
     * provable non-reciprocity (owner ref present, but the owner record's
     * own wallRefs do not list this wall back) is manufactured via direct
     * snapshot mutation, mirroring this file's own established precedent
     * for adversarial-only states. */
    snapshot.maps.front().rooms.front().wallRefs.clear();
    wallRecord.ownerRoomRefs.front().isLive = std::nullopt;
    wallRecord.ownerRoomRefs.front().livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    const AxiomEvaluationReport report = evaluateState(snapshot);
    const Finding              *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_NOT_RECIPROCAL);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    EXPECT_EQ(findFindingWithReason(
                  report,
                  AxiomCode::AX_WALL_01,
                  ReasonCode::WALL_OWNERSHIP_OWNER_LIVENESS_UNAVAILABLE),
              nullptr);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
