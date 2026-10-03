/*!
 * @file            test_SemanticAxiomEvaluator.cpp
 *
 * @brief           Unit tests for the semantic axiom evaluator
 *                  (SemanticAxiomEvaluator).
 */

/*
 * Focused, ROS/Gazebo-free tests for the pure SemanticAxiomEvaluator module
 * (evaluateState(), evaluateTransition(), evaluateMapCompleteness(),
 * computeAxiomCapabilityTable()).
 *
 * Every test builds real Atlas/Map/Room/`geometric::Plane`/Passage/Floor
 * objects through SemanticFixtures and the model's own setters, captures a
 * genuine SemanticGraphSnapshot via the production
 * captureSemanticGraphSnapshot() entry point (never hand-constructing a
 * snapshot), then evaluates it through the production evaluator entry points --
 * matching test_SemanticGraphSnapshot.cpp's own methodology.
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

/*!
 * @brief           Checks that evaluating an empty Atlas gives sixteen
 *                  aggregate results, one per axiom code with no duplicate,
 *                  running from AX_FRAME_01 to AX_MERGE_01.
 */
TEST(SemanticAxiomEvaluator, AggregateReportContainsExactlyOneEntryPerAxiomCode)
{
    Atlas                 atlas(0);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that the capability table has sixteen entries sorted
 *                  by axiom code and that AX_FRAME_01, AX_PASS_02, AX_FLOOR_01,
 *                  AX_COMP_01 and AX_MERGE_01 carry the expected capability
 *                  level, owning phase and class.
 */
TEST(SemanticAxiomEvaluator,
     AxiomCapabilityTableHasSixteenEntriesSortedByCodeWithKnownAssignments)
{
    std::vector<AxiomCapabilityEntry> table{};
    ASSERT_EQ(
        (computeAxiomCapabilityTable(table)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that on an empty Atlas each deferred axiom reports
 *                  exactly one UNKNOWN finding and an UNKNOWN aggregate, while
 *                  AX_ROOM_01 reports no finding and aggregates to PASS.
 */
TEST(SemanticAxiomEvaluator, DeferredAxiomsAlwaysReportExactlyOneUnknownFinding)
{
    Atlas                 atlas(0);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);

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

/*!
 * @brief           Checks that AX_ROOM_01 reports one UNKNOWN finding per room,
 *                  each naming only its own room, so two rooms give two
 *                  findings and an UNKNOWN aggregate.
 */
TEST(SemanticAxiomEvaluator,
     RoomCreationProvenanceIsPerRoomAndVacuousWithNoRooms)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room roomA;
    test::makeRoom(roomA, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&roomA)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room roomB;
    test::makeRoom(roomB, 2, p_map, nullptr, Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&roomB)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that AX_WALL_01 aggregates to FAIL when one wall has
 *                  no owner, one has two owners and one has a single valid
 *                  owner.
 */
TEST(SemanticAxiomEvaluator, AggregationPrecedenceFailBeatsUnknownBeatsPass)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&wallUnknown)),
              MapStatus::MAP_STATUS_SUCCESS);

    geometric::Plane wallMultiOwner;
    test::makeWallPlane(wallMultiOwner,
                        2,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wallMultiOwner)),
              MapStatus::MAP_STATUS_SUCCESS);

    geometric::Plane wallValid;
    test::makeWallPlane(wallValid,
                        3,
                        p_map,
                        Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitY(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wallValid)), MapStatus::MAP_STATUS_SUCCESS);

    Room ownerA;
    test::makeRoom(ownerA, 1, p_map, &wallMultiOwner);
    ASSERT_EQ((ownerA.setWalls(&wallValid)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&ownerA)),
              MapStatus::MAP_STATUS_SUCCESS);

    Room ownerB;
    test::makeRoom(ownerB, 2, p_map, &wallMultiOwner);
    ASSERT_EQ((p_map->addDetectedMapRoom(&ownerB)),
              MapStatus::MAP_STATUS_SUCCESS);

    std::unique_lock<std::mutex> lock{};
    ASSERT_EQ((atlas.acquireSemanticUpdateLock(lock)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    lock.unlock();

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const AggregateAxiomResult *p_wall01 =
        findAggregate(report, AxiomCode::AX_WALL_01);
    ASSERT_NE(p_wall01, nullptr);
    EXPECT_EQ(p_wall01->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that aggregateFindings turns PASS, UNKNOWN and FAIL
 *                  findings of one entity and axiom into FAIL, and into UNKNOWN
 *                  once the FAIL finding is removed.
 */
TEST(SemanticAxiomEvaluator, SameEntityPrecedenceFailBeatsUnknownBeatsPass)
{
    const EntityKey        oneWallKey{EntityKind::WALL, 1UL, 1};
    const EntityKey        oneOwnerKey{EntityKind::ROOM, 1UL, 1};
    std::vector<EntityKey> involvedKeys{oneWallKey, oneOwnerKey};

    std::vector<Finding> findings;
    Finding              finding2{};
    ASSERT_EQ(
        (makeFinding(AxiomCode::AX_WALL_01,
                     AxiomResult::PASS,
                     ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER,
                     involvedKeys,
                     finding2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    findings.push_back(finding2);
    Finding finding3{};
    ASSERT_EQ(
        (makeFinding(
            AxiomCode::AX_WALL_01,
            AxiomResult::UNKNOWN,
            ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE,
            involvedKeys,
            finding3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    findings.push_back(finding3);
    Finding finding4{};
    ASSERT_EQ(
        (makeFinding(AxiomCode::AX_WALL_01,
                     AxiomResult::FAIL,
                     ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS,
                     involvedKeys,
                     finding4)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    findings.push_back(finding4);

    std::vector<AggregateAxiomResult> aggregates{};
    ASSERT_EQ(
        (aggregateFindings(findings, aggregates)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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
    std::vector<AggregateAxiomResult> aggregatesWithoutFail{};
    ASSERT_EQ(
        (aggregateFindings(findingsWithoutFail, aggregatesWithoutFail)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that evaluateState over a valid wall, an ownerless
 *                  wall and a doubly owned wall reports all three findings and
 *                  aggregates AX_WALL_01 to FAIL.
 */
TEST(SemanticAxiomEvaluator,
     AxWall01PrecedenceThroughEvaluateStateOverRealEntities)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&passWall)), MapStatus::MAP_STATUS_SUCCESS);
    Room passRoom;
    test::makeRoom(passRoom, 1, p_map, &passWall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&passRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&unknownWall)),
              MapStatus::MAP_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPlane(&failWall)), MapStatus::MAP_STATUS_SUCCESS);
    Room failOwnerA;
    test::makeRoom(failOwnerA,
                   2,
                   p_map,
                   &failWall,
                   Eigen::Vector3d(10.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&failOwnerA)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room failOwnerB;
    test::makeRoom(failOwnerB,
                   3,
                   p_map,
                   &failWall,
                   Eigen::Vector3d(10.0, 1.0, 0.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&failOwnerB)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);

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

/*!
 * @brief           Checks that evaluating a snapshot and a copy with its room
 *                  and wall records reversed gives the same finding ids and
 *                  results.
 */
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
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room1;
    test::makeRoom(room1, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room1)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room room2;
    test::makeRoom(room2, 2, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room2)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot original{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, original)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    SemanticGraphSnapshot reordered = original;
    ASSERT_EQ(reordered.maps.size(), 1U);
    std::reverse(reordered.maps.front().rooms.begin(),
                 reordered.maps.front().rooms.end());
    std::reverse(reordered.maps.front().walls.begin(),
                 reordered.maps.front().walls.end());

    AxiomEvaluationReport reportOriginal{};
    ASSERT_EQ(
        (evaluateState(original, reportOriginal)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    AxiomEvaluationReport reportReordered{};
    ASSERT_EQ(
        (evaluateState(reordered, reportReordered)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);

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

/*!
 * @brief           Checks that the findings of one evaluation come out in non-
 *                  decreasing id order.
 */
TEST(SemanticAxiomEvaluator, FindingsAreSortedById)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    for (std::size_t index = 1U; index < report.findings.size(); ++index)
    {
        EXPECT_LE(report.findings[index - 1U].id, report.findings[index].id);
    }
}

/*!
 * @brief           Checks that evaluating one snapshot twice gives identical
 *                  findings and aggregates and leaves the snapshot's contents
 *                  unchanged.
 */
TEST(SemanticAxiomEvaluator, EvaluateStateIsIdempotentAndDoesNotMutateInput)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport reportFirst{};
    ASSERT_EQ(
        (evaluateState(snapshot, reportFirst)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    AxiomEvaluationReport reportSecond{};
    ASSERT_EQ(
        (evaluateState(snapshot, reportSecond)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);

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

/*!
 * @brief           Checks that evaluateTransition reports one
 *                  not-yet-implemented finding each for AX_FRAME_01 and
 *                  AX_TXN_01 while its AX_WALL_01 findings equal those of
 *                  evaluateState on the after snapshot.
 */
TEST(SemanticAxiomEvaluator,
     EvaluateTransitionReplacesFrameAndTxnPlaceholdersButKeepsRestFromAfter)
{
    Atlas atlasBefore(0);
    Atlas atlasAfter(0);
    Map  *p_mapAfter = nullptr;
    ASSERT_EQ((atlasAfter.getCurrentMap(p_mapAfter)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapAfter,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_mapAfter->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot before{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlasBefore, before)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    SemanticGraphSnapshot after{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlasAfter, after)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    const TransitionEvaluationContext context;
    AxiomEvaluationReport             transitionReport{};
    ASSERT_EQ(
        (evaluateTransition(before, after, context, transitionReport)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    AxiomEvaluationReport afterStateReport{};
    ASSERT_EQ(
        (evaluateState(after, afterStateReport)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);

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

/*!
 * @brief           Checks that a wall no room owns gets an UNKNOWN AX_WALL_01
 *                  finding because its ownership commitment cannot be verified.
 */
TEST(SemanticAxiomEvaluator, OwnerlessWallIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_ZERO_OWNERS_COMMITMENT_UNVERIFIABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a wall owned by exactly one live room of the
 *                  same map gets a PASS AX_WALL_01 finding.
 */
TEST(SemanticAxiomEvaluator, SingleValidSameMapOwnerIsPass)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_SINGLE_VALID_OWNER);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::PASS);
}

/*!
 * @brief           Checks that a wall owned by two rooms gets a FAIL AX_WALL_01
 *                  finding with an observed owner count of two.
 */
TEST(SemanticAxiomEvaluator, MultipleOwnersIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room roomA;
    test::makeRoom(roomA, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&roomA)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room roomB;
    test::makeRoom(roomB, 2, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&roomB)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_MULTIPLE_OWNERS);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
    ASSERT_TRUE(p_finding->evidence.observedCount.has_value());
    EXPECT_EQ(*p_finding->evidence.observedCount, 2U);
}

/*!
 * @brief           Checks that a wall whose only owner room is retired gets a
 *                  FAIL AX_WALL_01 finding.
 */
TEST(SemanticAxiomEvaluator, BadOwnerIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((room.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall owned by a room registered in a different
 *                  map gets a FAIL AX_WALL_01 finding.
 */
TEST(SemanticAxiomEvaluator, CrossMapOwnerIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long id{};
    ASSERT_EQ((p_mapA->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    unsigned long id2{};
    ASSERT_EQ((p_mapB->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_NE(id, id2);

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_mapA->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    /* Deliberately register a mapB room that owns mapA's wall: an upstream
     * contract violation this evaluator must still observe, not assume
     * away. */
    Room room;
    test::makeRoom(room, 1, p_mapB, &wall);
    ASSERT_EQ((p_mapB->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall whose owner reference locates a
 *                  prospective (UNDEFINED variant) room gets a FAIL AX_WALL_01
 *                  finding.
 */
TEST(SemanticAxiomEvaluator, WallOwnerWrongVariantIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room,
                   1,
                   p_map,
                   &wall,
                   Eigen::Vector3d::Zero(),
                   Room::RoomVariant::UNDEFINED);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_VARIANT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall gets a FAIL AX_WALL_01 finding when its
 *                  owner room is registered in the wall's map but declares a
 *                  different map.
 */
TEST(SemanticAxiomEvaluator, WallOwnerDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long id{};
    ASSERT_EQ((p_mapA->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    unsigned long id2{};
    ASSERT_EQ((p_mapB->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_NE(id, id2);

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_mapA->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    /* room's own declared map is mapB (makeRoom's p_map_in), but it is
     * enumerated (registered) under mapA's collection, so its containing-map
     * key matches the wall's map while its declaredMapId does not. */
    Room room;
    test::makeRoom(room, 1, p_mapB, &wall);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall gets a FAIL AX_WALL_01 finding when two
 *                  distinct room records share the owner's key.
 */
TEST(SemanticAxiomEvaluator, WallOwnerDuplicateIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    /* A second, distinct Room object deliberately reuses local id 1 -- an
     * upstream collision this evaluator must observe, not assume away. */
    Room duplicateIdRoom;
    test::makeRoom(duplicateIdRoom, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&duplicateIdRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_DUPLICATE_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall gets an UNKNOWN AX_WALL_01 finding when
 *                  its owner reference names a room that has no record in the
 *                  snapshot (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallOwnerRecordUnavailableIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.front().ownerRoomRefs.size(), 1U);
    /* Retarget the owner reference at a local id no RoomRecord in this
     * snapshot actually has, keeping key.mapId/isLive unchanged. */
    snapshot.maps.front().walls.front().ownerRoomRefs.front().key->entityId =
        999;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_UNAVAILABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a wall gets a FAIL AX_WALL_01 finding when the
 *                  owner room's wall references no longer list the wall (edited
 *                  into the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallOwnerNotReciprocalIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_NOT_RECIPROCAL);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that two wall records sharing one key give a FAIL
 *                  AX_WALL_01 finding (duplicate added to the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallDuplicateIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    WallRecord duplicateWall = snapshot.maps.front().walls.front();
    duplicateWall.ownerRoomRefs.clear();
    snapshot.maps.front().walls.push_back(duplicateWall);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_WALL_DUPLICATE_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall whose declared map differs from the map
 *                  that contains it gets a FAIL AX_WALL_01 finding (edited into
 *                  the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    snapshot.maps.front().walls.front().declaredMapId =
        snapshot.maps.front().walls.front().key.mapId + 1U;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_WALL_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall gets a FAIL AX_WALL_01 finding when its
 *                  owner room record is marked not live although the owner
 *                  reference says live (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallOwnerRecordNotLiveIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    snapshot.maps.front().rooms.front().isLive = false;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_RECORD_NOT_LIVE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall record whose key kind is not WALL gets a
 *                  FAIL AX_WALL_01 finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallWrongKeyKindIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    snapshot.maps.front().walls.front().key.kind = EntityKind::ROOM;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_WALL_WRONG_KEY_KIND);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall record whose plane type is not WALL gets
 *                  a FAIL AX_WALL_01 finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallWrongPlaneTypeIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    snapshot.maps.front().walls.front().planeType =
        geometric::Plane::PlaneVariant::DOOR;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_WALL_WRONG_PLANE_TYPE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall whose owner reference key is not of kind
 *                  ROOM gets a FAIL AX_WALL_01 finding (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator, WallOwnerWrongKeyKindIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().walls.front().ownerRoomRefs.size(), 1U);
    snapshot.maps.front().walls.front().ownerRoomRefs.front().key->kind =
        EntityKind::WALL;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_WRONG_KEY_KIND);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a missing declared map on a wall keeps the PASS
 *                  owner finding but adds an UNKNOWN finding and makes the
 *                  AX_WALL_01 aggregate UNKNOWN.
 */
TEST(SemanticAxiomEvaluator, WallDeclaredMapUnavailableCapsAggregateAtUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().walls.size(), 1U);
    snapshot.maps.front().walls.front().declaredMapId.reset();

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_passFinding =
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

/*!
 * @brief           Checks that an owner room whose only wall reference shares
 *                  the wall's identity but has the wrong plane type gives a
 *                  FAIL contradictory-reciprocal finding (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator, WallReciprocalMalformedOnlyIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    roomRecord.wallRefs.front().planeType =
        geometric::Plane::PlaneVariant::DOOR;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that an owner room listing the same wall twice with
 *                  well-formed references gives a FAIL duplicate-reciprocal
 *                  finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallReciprocalDuplicateIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    roomRecord.wallRefs.push_back(roomRecord.wallRefs.front());

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_RECIPROCAL_DUPLICATE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a valid wall reference plus a malformed copy of
 *                  it in the owner room gives a FAIL contradictory-reciprocal
 *                  finding instead of hiding behind the valid entry (edited
 *                  into the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallReciprocalValidPlusMalformedIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    RawPlaneRef malformedCopy = roomRecord.wallRefs.front();
    malformedCopy.isLive      = false;
    roomRecord.wallRefs.push_back(malformedCopy);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* ------------------------------------------------------------------------
 * AX-WALL-03: twin wall-face plausibility
 * ---------------------------------------------------------------------- */

/*!
 * @brief           Checks that a wall with no twin face gets a PASS AX_WALL_03
 *                  finding with reason WALL_TWIN_ABSENT.
 */
TEST(SemanticAxiomEvaluator, NullTwinIsPass)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_ABSENT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::PASS);
}

/*!
 * @brief           Checks that a wall that is its own twin face gets a FAIL
 *                  AX_WALL_03 finding.
 */
TEST(SemanticAxiomEvaluator, SelfTwinIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((wall.setTwinFace(&wall)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_SELF);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall whose twin does not point back at it gets
 *                  a FAIL AX_WALL_03 finding.
 */
TEST(SemanticAxiomEvaluator, AsymmetricTwinIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
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
    ASSERT_EQ((wallA.setTwinFace(&wallB)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wallA)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wallB)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_ASYMMETRIC);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall whose reciprocal twin is retired gets a
 *                  FAIL AX_WALL_03 finding.
 */
TEST(SemanticAxiomEvaluator, BadTwinIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
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
    ASSERT_EQ((wallA.setTwinFace(&wallB)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wallB.setTwinFace(&wallA)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wallB.setBad()), geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wallA)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wallB)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(report,
                                                     AxiomCode::AX_WALL_03,
                                                     ReasonCode::WALL_TWIN_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that reciprocal twin walls registered in different
 *                  maps get a FAIL AX_WALL_03 finding.
 */
TEST(SemanticAxiomEvaluator, CrossMapTwinIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

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
    ASSERT_EQ((wallA.setTwinFace(&wallB)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wallB.setTwinFace(&wallA)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_mapA->addMapPlane(&wallA)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_mapB->addMapPlane(&wallB)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall whose twin face is a ground plane gets a
 *                  FAIL AX_WALL_03 finding.
 */
TEST(SemanticAxiomEvaluator, WrongTypeTwinIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
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
    ASSERT_EQ((wall.setTwinFace(&groundNotWall)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&groundNotWall)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_WRONG_TYPE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that reciprocal twin walls owned by the same room get
 *                  a FAIL AX_WALL_03 finding.
 */
TEST(SemanticAxiomEvaluator, SharedOwnerTwinIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
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
    ASSERT_EQ((wallA.setTwinFace(&wallB)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wallB.setTwinFace(&wallA)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wallA)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wallB)), MapStatus::MAP_STATUS_SUCCESS);

    Room sharedOwner;
    test::makeRoom(sharedOwner, 1, p_map, &wallA);
    ASSERT_EQ((sharedOwner.setWalls(&wallB)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&sharedOwner)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_SHARED_OWNER_FORBIDDEN);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that reciprocal twin walls with different owners get
 *                  an UNKNOWN AX_WALL_03 finding because their geometry is not
 *                  verified.
 */
TEST(SemanticAxiomEvaluator, StructurallyValidTwinIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
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
    ASSERT_EQ((wallA.setTwinFace(&wallB)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((wallB.setTwinFace(&wallA)),
              geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wallA)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wallB)), MapStatus::MAP_STATUS_SUCCESS);

    Room ownerA;
    test::makeRoom(ownerA, 1, p_map, &wallA);
    ASSERT_EQ((p_map->addDetectedMapRoom(&ownerA)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room ownerB;
    test::makeRoom(ownerB, 2, p_map, &wallB);
    ASSERT_EQ((p_map->addDetectedMapRoom(&ownerB)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a live passage that is not passable gets a FAIL
 *                  AX_PASS_01 finding.
 */
TEST(SemanticAxiomEvaluator, NonPassableLivePassageIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room knownSide;
    test::makeRoom(knownSide,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&knownSide)),
              MapStatus::MAP_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_01,
                              ReasonCode::PASSAGE_PROVENANCE_NOT_PASSABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a live, passable passage gets an UNKNOWN
 *                  AX_PASS_01 finding because its full provenance chain cannot
 *                  be verified.
 */
TEST(SemanticAxiomEvaluator, PassableLivePassageIsUnknownForFullProvenance)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room knownSide;
    test::makeRoom(knownSide,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&knownSide)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &knownSide,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_01,
        ReasonCode::PASSAGE_PROVENANCE_FULL_CHAIN_UNVERIFIABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a passage with no confirmed room endpoint gets a
 *                  FAIL AX_PASS_02 finding.
 */
TEST(SemanticAxiomEvaluator, ZeroConfirmedEndpointsIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      nullptr,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_NO_CONFIRMED_ENDPOINT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a passage with two reciprocal confirmed rooms
 *                  stays UNKNOWN for AX_PASS_02 and AX_PASS_03 because
 *                  endpoint-slot proof is unavailable, although the slot-state
 *                  clause itself passes.
 */
TEST(SemanticAxiomEvaluator, TwoConfirmedReciprocalEndpointsIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a passage with one reciprocal confirmed room and
 *                  an empty other side gets an UNKNOWN AX_PASS_02 finding.
 */
TEST(SemanticAxiomEvaluator, OneConfirmedEndpointOtherEmptyIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a third live confirmed room that lists the
 *                  passage, without being named by it, gives a FAIL
 *                  third-endpoint finding.
 */
TEST(SemanticAxiomEvaluator, ThirdReverseOnlyConfirmedEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);
    Room thirdRoom;
    test::makeRoom(thirdRoom,
                   3,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&thirdRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    /* thirdRoom independently lists the same passage, but the passage's own
     * forward fields never name thirdRoom -- a reverse-only third endpoint. */
    ASSERT_EQ((thirdRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_THIRD_ENDPOINT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a retired room listing the passage is ignored,
 *                  leaving the passage at its ordinary UNKNOWN AX_PASS_02
 *                  result with no FAIL.
 */
TEST(SemanticAxiomEvaluator, RetiredReverseOnlyRoomDoesNotPoisonLivePassage)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room retiredRoom;
    test::makeRoom(retiredRoom,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&retiredRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retiredRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retiredRoom.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a room's passage reference marked not live while
 *                  the passage is live gives a FAIL bad-reverse-endpoint
 *                  finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, ReverseReferenceOwnLivenessBadIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room reverseOnly;
    test::makeRoom(reverseOnly,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&reverseOnly)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((reverseOnly.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a live prospective room declared in another map
 *                  and listing the passage still gives a FAIL cross-map
 *                  reverse-endpoint finding.
 */
TEST(SemanticAxiomEvaluator, LiveProspectiveReverseOnlyRoomAnomalyIsExamined)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long id{};
    ASSERT_EQ((p_mapA->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    unsigned long id2{};
    ASSERT_EQ((p_mapB->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_NE(id, id2);

    Room known;
    test::makeRoom(known, 1, p_mapA, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room prospectiveReverseOnly;
    test::makeRoom(prospectiveReverseOnly,
                   2,
                   p_mapB,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0),
                   Room::RoomVariant::UNDEFINED);
    ASSERT_EQ((p_mapB->addDetectedMapRoom(&prospectiveReverseOnly)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_mapA,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_mapA->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((prospectiveReverseOnly.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room listing the passage while filling its
 *                  empty far slot is not reported as a third endpoint and the
 *                  passage stays UNKNOWN.
 */
TEST(SemanticAxiomEvaluator, ReverseOnlyRoomFillingEmptySlotIsNotThirdEndpoint)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room farReverseOnly;
    test::makeRoom(farReverseOnly,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&farReverseOnly)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    /* Deliberately no prospective/far room on the forward side. */
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    /* farReverseOnly independently lists the passage back even though the
     * passage's own forward prospectiveRoomRef never names it. */
    ASSERT_EQ((farReverseOnly.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a room's passage reference whose key has the
 *                  passage's ids but the wrong entity kind gives a FAIL
 *                  wrong-kind reverse-endpoint finding (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator, ReverseWrongKindKeyIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room otherRoom;
    test::makeRoom(otherRoom,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&otherRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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
    EntityRef     wrongKindRef;
    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    wrongKindRef.key    = EntityKey{EntityKind::ROOM, mapId, 1};
    wrongKindRef.reason = UnavailableReason::NONE;
    wrongKindRef.isLive = true;
    wrongKindRef.livenessUnavailableReason = UnavailableReason::NONE;
    p_otherRoomRecord->passageRefs.push_back(wrongKindRef);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_WRONG_KIND);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room naming the same passage twice in its
 *                  passage references gives a FAIL duplicated-reverse-reference
 *                  finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, ReverseReferenceDuplicatedIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_REFERENCE_DUPLICATED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a passage whose known-side room key is shared by
 *                  two room records gets a FAIL duplicate-identity AX_PASS_02
 *                  finding.
 */
TEST(SemanticAxiomEvaluator, ForwardEndpointDuplicateIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    /* A second, distinct Room object deliberately reuses local id 1. */
    Room duplicateIdRoom;
    test::makeRoom(duplicateIdRoom,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&duplicateIdRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    /* Deliberately never call setDoorways() on either room sharing local id
     * 1, so the reverse scan's own duplicate-identity check (already
     * covered by PassageCardinalityDuplicateRoomIdentityIsFail) does not
     * fire first -- this isolates the FORWARD-endpoint duplicate-identity
     * path in resolveRoomEndpoint()/evaluateOnePassageCardinality(). */

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_FORWARD_ENDPOINT_DUPLICATE_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a known-side room reference whose key kind is
 *                  not ROOM gives FAIL findings for AX_PASS_02, AX_PASS_03,
 *                  AX_PASS_04 and AX_FLOOR_01 (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, ForwardEndpointWrongKindIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    snapshot.maps.front().passages.front().knownSideRoomRef.key->kind =
        EntityKind::WALL;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_cardinalityFinding = findFindingWithReason(
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

/*!
 * @brief           Checks that a known-side room that declares a different map
 *                  than the one it was found in gives a FAIL AX_PASS_02 finding
 *                  (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, ForwardEndpointDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    snapshot.maps.front().rooms.front().declaredMapId = mapId + 999U;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a known-side room reference with no liveness
 *                  value, pointing at an unlocatable room, gives UNKNOWN rather
 *                  than a no-endpoint or bad-endpoint FAIL (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator, ForwardEndpointLivenessUnavailableIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    EntityRef &knownSideRef =
        snapshot.maps.front().passages.front().knownSideRoomRef;
    knownSideRef.key->entityId = 999;
    knownSideRef.isLive.reset();
    knownSideRef.livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that a room's passage reference with no liveness
 *                  value gives UNKNOWN and is not counted as a third endpoint
 *                  (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, ReverseEndpointLivenessUnavailableIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room otherRoom;
    test::makeRoom(otherRoom,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&otherRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((otherRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    RoomRecord *p_otherRoomRecord = nullptr;
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that scanReversePassageEndpoints records a clean
 *                  prospective room that lists the passage, and that the
 *                  passage stays UNKNOWN instead of failing as a third
 *                  endpoint.
 */
TEST(SemanticAxiomEvaluator, CleanProspectiveReverseRelationshipIsRepresented)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room prospectiveRoom;
    test::makeRoom(prospectiveRoom,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0),
                   Room::RoomVariant::UNDEFINED);
    ASSERT_EQ((p_map->addDetectedMapRoom(&prospectiveRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((prospectiveRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    ReversePassageEndpointScan scan{};
    unsigned long              id{};
    ASSERT_EQ((p_map->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ(
        (scanReversePassageEndpoints(snapshot.maps.front().passages.front(),
                                     id,
                                     snapshot,
                                     scan)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    /* The prospective room's own clean reciprocal reference is represented,
     * not discarded. known's own reciprocal reference (the passage's
     * forward known-side room, which also independently lists the passage
     * back) still correctly appears in confirmedReverseRoomKeys -- this
     * scan reports every reverse relationship it finds, forward-named rooms
     * included; the caller reconciles that (see
     * evaluateOnePassageCardinality.cc's union computation). */
    ASSERT_EQ(scan.prospectiveReverseRoomKeys.size(), 1U);
    unsigned long id2{};
    ASSERT_EQ((p_map->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(scan.prospectiveReverseRoomKeys.front(),
              (EntityKey{EntityKind::ROOM, id2, 2}));
    ASSERT_EQ(scan.confirmedReverseRoomKeys.size(), 1U);
    unsigned long id3{};
    ASSERT_EQ((p_map->getId(id3)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(scan.confirmedReverseRoomKeys.front(),
              (EntityKey{EntityKind::ROOM, id3, 1}));

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a room declared in another map that lists the
 *                  passage gives a FAIL cross-map reverse-endpoint finding.
 */
TEST(SemanticAxiomEvaluator, CrossMapReverseOnlyEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_otherMap = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_otherMap)),
              AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long id{};
    ASSERT_EQ((p_map->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    unsigned long id2{};
    ASSERT_EQ((p_otherMap->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_NE(id, id2);

    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room crossMapRoom;
    test::makeRoom(crossMapRoom,
                   2,
                   p_otherMap,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_otherMap->addDetectedMapRoom(&crossMapRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((crossMapRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_REVERSE_ENDPOINT_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room's unkeyed passage reference sharing a
 *                  local id with a real passage gives a room-scoped FAIL
 *                  finding, while the real passage stays UNKNOWN and does not
 *                  name that room.
 */
TEST(SemanticAxiomEvaluator,
     UnkeyedReverseReferenceIsRoomScopedNotPassageAttributed)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room roomWithDanglingRef;
    test::makeRoom(roomWithDanglingRef,
                   2,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&roomWithDanglingRef)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    /* Never registered via AddMapPassage, so its own getMap() is nullptr,
     * but it shares the real passage's local id (1). */
    Passage unregisteredPassageWithSameId;
    ASSERT_EQ((unregisteredPassageWithSameId.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((roomWithDanglingRef.setDoorways(&unregisteredPassageWithSameId)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);

    const Finding *p_malformedFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ROOM_HAS_MALFORMED_REFERENCE);
    ASSERT_NE(p_malformedFinding, nullptr);
    EXPECT_EQ(p_malformedFinding->result, AxiomResult::FAIL);
    ASSERT_EQ(p_malformedFinding->involvedKeys.size(), 1U);
    unsigned long id2{};
    ASSERT_EQ((p_map->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(p_malformedFinding->involvedKeys.front(),
              (EntityKey{EntityKind::ROOM, id2, 2}));

    const Finding *p_cardinalityFinding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_SLOT_UNVERIFIED);
    ASSERT_NE(p_cardinalityFinding, nullptr);
    EXPECT_EQ(p_cardinalityFinding->result, AxiomResult::UNKNOWN);
    /* The real passage's own finding must not name roomWithDanglingRef. */
    unsigned long id3{};
    ASSERT_EQ((p_map->getId(id3)), MapStatus::MAP_STATUS_SUCCESS);
    EXPECT_EQ(std::find(p_cardinalityFinding->involvedKeys.begin(),
                        p_cardinalityFinding->involvedKeys.end(),
                        EntityKey{EntityKind::ROOM, id3, 2}),
              p_cardinalityFinding->involvedKeys.end());
}

/*!
 * @brief           Checks that two room records sharing the key of a room that
 *                  lists the passage give a FAIL duplicate-room-identity
 *                  finding.
 */
TEST(SemanticAxiomEvaluator, PassageCardinalityDuplicateRoomIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    /* A second, distinct Room object deliberately reuses local id 1. */
    Room duplicateIdRoom;
    test::makeRoom(duplicateIdRoom,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&duplicateIdRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((duplicateIdRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ROOM_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a passage whose known and far sides are the same
 *                  room gets a FAIL duplicate-endpoint AX_PASS_02 finding.
 */
TEST(SemanticAxiomEvaluator, DuplicateEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room onlyRoom;
    test::makeRoom(onlyRoom, 1, p_map, nullptr, Eigen::Vector3d(0.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&onlyRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &onlyRoom,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &onlyRoom);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((onlyRoom.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_02,
        ReasonCode::PASSAGE_CARDINALITY_DUPLICATE_ENDPOINT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a passage naming a room that does not list the
 *                  passage back gets a FAIL non-reciprocal AX_PASS_02 finding.
 */
TEST(SemanticAxiomEvaluator, NonReciprocalEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    /* Deliberately never call known.setDoorways(&passage): the passage
     * names the room, but the room does not name the passage back. */

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_NON_RECIPROCAL);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a retired far-side room missing from every room
 *                  collection still gives a FAIL bad-endpoint finding, even
 *                  next to a valid known-side room.
 */
TEST(SemanticAxiomEvaluator, BadUnenumeratedOtherSideEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

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
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot2{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot2)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot2, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_PASS_02,
                              ReasonCode::PASSAGE_CARDINALITY_ENDPOINT_BAD);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a passage whose known-side room is still
 *                  prospective gets a FAIL AX_PASS_03 finding.
 */
TEST(SemanticAxiomEvaluator, KnownSideNotConfirmedIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room unpromoted;
    test::makeRoom(unpromoted,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0),
                   Room::RoomVariant::UNDEFINED);
    ASSERT_EQ((p_map->addDetectedMapRoom(&unpromoted)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &unpromoted,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((unpromoted.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_03,
        ReasonCode::PASSAGE_SLOT_KNOWN_SIDE_NOT_CONFIRMED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a keyed known-side reference whose reason is not
 *                  NONE gives FAIL findings for AX_PASS_02, AX_PASS_03,
 *                  AX_PASS_04 and AX_FLOOR_01 (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     ReasonInconsistentForwardReferenceFailsEveryPassageAxiom)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    PassageRecord &passageRecord = snapshot.maps.front().passages.front();
    ASSERT_TRUE(passageRecord.knownSideRoomRef.key.has_value());
    /* Invariant-violating: key retained, but reason no longer NONE. */
    passageRecord.knownSideRoomRef.reason =
        UnavailableReason::ENTITY_HAS_NO_MAP;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);

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

/*!
 * @brief           Checks that a passage whose known-side room is in a
 *                  different map gives FAIL findings for AX_PASS_02,
 *                  AX_PASS_04, AX_PASS_03 and AX_FLOOR_01.
 */
TEST(SemanticAxiomEvaluator, CrossMapPassageEndpointIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    Room roomInMapB;
    test::makeRoom(roomInMapB,
                   1,
                   p_mapB,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_mapB->addDetectedMapRoom(&roomInMapB)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_mapA,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &roomInMapB,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_mapA->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a passage joining rooms on different floors
 *                  gives a FAIL AX_PASS_04 disagreement finding and a FAIL
 *                  cross-floor AX_FLOOR_01 finding.
 */
TEST(SemanticAxiomEvaluator, CrossFloorPassageIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);

    Floor floorA;
    test::makeFloor(floorA, 1, p_map, {&known}, 0.0);
    ASSERT_EQ((p_map->addMapFloor(&floorA)), MapStatus::MAP_STATUS_SUCCESS);
    Floor floorB;
    test::makeFloor(floorB, 2, p_map, {&far}, 3.0);
    ASSERT_EQ((p_map->addMapFloor(&floorB)), MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that rooms on one floor give PASS agreement findings
 *                  for AX_PASS_04 and AX_FLOOR_01 while both aggregates stay
 *                  UNKNOWN because endpoint proof is unverified.
 */
TEST(SemanticAxiomEvaluator, SameFloorPassageAgreesButAggregateIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);

    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far}, 0.0);
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a passage whose endpoint rooms have no floor
 *                  gets an UNKNOWN AX_PASS_04 finding for unverified room-floor
 *                  proof.
 */
TEST(SemanticAxiomEvaluator, MissingFloorEvidenceOnPassageIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);
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
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
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
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a room whose boundary was never observed gets an
 *                  UNKNOWN AX_BOUND_01 finding.
 */
TEST(SemanticAxiomEvaluator, UnobservedBoundaryIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    vs_graphs::core::semantic::Room::BoundaryStatus boundaryStatus{};
    ASSERT_EQ((room.getBoundaryStatus(boundaryStatus)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(boundaryStatus, Room::BoundaryStatus::UNOBSERVED);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_NOT_YET_COMPLETE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a room whose boundary status is CONFLICTING gets
 *                  a FAIL AX_BOUND_01 finding.
 */
TEST(SemanticAxiomEvaluator, ConflictingBoundaryIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::CONFLICTING)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_CONFLICTING_STATE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room marked COMPLETE with only two corners
 *                  gets a FAIL AX_BOUND_01 finding.
 */
TEST(SemanticAxiomEvaluator, CompleteWithTooFewCornersIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m(
            {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_TOO_FEW_CORNERS);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room marked COMPLETE with four corners but no
 *                  wall gets a FAIL AX_BOUND_01 finding.
 */
TEST(SemanticAxiomEvaluator, CompleteWithNoWallEvidenceIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_NO_WALL_EVIDENCE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a COMPLETE room whose corners form a
 *                  self-intersecting (bowtie) polygon gets a FAIL AX_BOUND_01
 *                  finding.
 */
TEST(SemanticAxiomEvaluator, CompleteSelfIntersectingIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    /* A bowtie quadrilateral: edges (0->1) and (2->3) cross. */
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_SELF_INTERSECTING);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a COMPLETE room with observation gaps gets an
 *                  UNKNOWN AX_BOUND_01 finding because gap correspondence is
 *                  unverified.
 */
TEST(SemanticAxiomEvaluator, CompleteWithObservationGapsIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setObservationGaps({Room::ObservationGap{0.0, 0.2}})),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_BOUND_01,
        ReasonCode::ROOM_BOUNDARY_GAP_CORRESPONDENCE_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a COMPLETE room with a valid polygon and valid
 *                  wall evidence still gets an UNKNOWN AX_BOUND_01 finding,
 *                  since edge-to-wall support is not verified.
 */
TEST(SemanticAxiomEvaluator, CompleteWithVerifiedWallEvidenceIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_BOUND_01,
        ReasonCode::ROOM_BOUNDARY_EDGE_SUPPORT_UNVERIFIED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a COMPLETE room with a NaN corner coordinate
 *                  gets a FAIL AX_BOUND_01 finding.
 */
TEST(SemanticAxiomEvaluator, NonFiniteCornerIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((room.setBoundaryCorners_world_m(
                  {Eigen::Vector3d(0.0, 0.0, 0.0),
                   Eigen::Vector3d(1.0, 0.0, 0.0),
                   Eigen::Vector3d(1.0,
                                   std::numeric_limits<double>::quiet_NaN(),
                                   0.0),
                   Eigen::Vector3d(0.0, 1.0, 0.0)})),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_NON_FINITE_CORNER);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a COMPLETE room with an infinite corner
 *                  coordinate gets a FAIL AX_BOUND_01 finding.
 */
TEST(SemanticAxiomEvaluator, InfiniteCornerIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m(
            {Eigen::Vector3d(0.0, 0.0, 0.0),
             Eigen::Vector3d(1.0, 0.0, 0.0),
             Eigen::Vector3d(1.0, std::numeric_limits<double>::infinity(), 0.0),
             Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_NON_FINITE_CORNER);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a live room referencing a retired wall gets a
 *                  FAIL invalid-wall-evidence AX_BOUND_01 finding while
 *                  AX_WALL_01 skips the retired wall.
 */
TEST(SemanticAxiomEvaluator, LiveRoomReferencingRetiredWallCannotProveBoundary)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((wall.setBad()), geometric::PlaneStatus::PLANE_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);

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

/*!
 * @brief           Checks that a room with one valid wall reference plus one
 *                  wrong-type reference gets a FAIL invalid-wall-evidence
 *                  AX_BOUND_01 finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     BoundaryOneValidPlusOneWrongTypeWallReferenceIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a wall reference with no map adds no FAIL to
 *                  AX_BOUND_01 and the room stays at the UNKNOWN edge-support
 *                  finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     BoundaryUnmappedWallReferenceContributesOnlyUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    /* No FAIL: the unmapped entry is unavailable, not a proven
     * contradiction, and the genuinely valid wall still leaves the room at
     * the Phase-6 support UNKNOWN. */
    const Finding *p_invalidFinding =
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

/*!
 * @brief           Checks that a COMPLETE room whose only wall reference is
 *                  unavailable gets an UNKNOWN wall-evidence-unavailable
 *                  finding rather than the no-wall-evidence FAIL (edited into
 *                  the snapshot).
 */
TEST(SemanticAxiomEvaluator, NonemptyAllUnavailableWallEvidenceIsUnknownNotFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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
    /* Make the sole reference genuinely unavailable (no map at all) rather
     * than removing it, so the collection stays nonempty. */
    p_roomRecord->wallRefs.front().mapId.reset();
    p_roomRecord->wallRefs.front().wallKey.reset();

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a room with no floor gets an UNKNOWN AX_FLOOR_01
 *                  finding with reason ROOM_FLOOR_UNLINKED.
 */
TEST(SemanticAxiomEvaluator, RoomWithNoFloorIsUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_UNLINKED);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a room and a floor that list each other get a
 *                  PASS AX_FLOOR_01 finding.
 */
TEST(SemanticAxiomEvaluator, RoomFloorReciprocalIsPass)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_RECIPROCAL_VALID);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::PASS);
}

/*!
 * @brief           Checks that a room whose floor does not list it back gets a
 *                  FAIL AX_FLOOR_01 finding.
 */
TEST(SemanticAxiomEvaluator, RoomFloorNonReciprocalIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);
    /* Room believes it has a floor; the floor does not list it back. */
    ASSERT_EQ((room.setFloor(&floor)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_NON_RECIPROCAL);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room whose floor belongs to a different map
 *                  gets a FAIL AX_FLOOR_01 finding.
 */
TEST(SemanticAxiomEvaluator, RoomFloorCrossMapIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    Room room;
    test::makeRoom(room, 1, p_mapA, nullptr);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    Floor floorInMapB;
    test::makeFloor(floorInMapB, 1, p_mapB, {});
    ASSERT_EQ((p_mapB->addMapFloor(&floorInMapB)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((room.setFloor(&floorInMapB)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a floor listing a room that names no floor gives
 *                  a FAIL reverse-claim finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomFloorReverseClaimWithoutForwardLinkIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_REVERSE_CLAIM_WITHOUT_FORWARD_LINK);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room whose floor reference key is not of kind
 *                  FLOOR gets a FAIL AX_FLOOR_01 finding (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomFloorWrongKindIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    ASSERT_TRUE(snapshot.maps.front().rooms.front().floorRef.key.has_value());
    snapshot.maps.front().rooms.front().floorRef.key->kind = EntityKind::ROOM;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_WRONG_KIND);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a floor whose declared map differs from its
 *                  containing map gets a FAIL AX_FLOOR_01 finding (edited into
 *                  the snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomFloorDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    snapshot.maps.front().floors.front().declaredMapId =
        snapshot.maps.front().floors.front().key.mapId + 1U;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room whose declared map differs from its
 *                  containing map gets a FAIL AX_FLOOR_01 finding (edited into
 *                  the snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomFloorRoomDeclaredMapMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    snapshot.maps.front().rooms.front().declaredMapId = mapId + 999U;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_ROOM_DECLARED_MAP_MISMATCH);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a missing declared map on the room keeps the
 *                  PASS reciprocity finding but adds an UNKNOWN finding (edited
 *                  into the snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomFloorRoomDeclaredMapUnavailableCapsAtUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    snapshot.maps.front().rooms.front().declaredMapId.reset();

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_passFinding =
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

/*!
 * @brief           Checks that a missing declared map on the floor keeps the
 *                  PASS reciprocity finding but adds an UNKNOWN finding (edited
 *                  into the snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomFloorFloorDeclaredMapUnavailableCapsAtUnknown)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    snapshot.maps.front().floors.front().declaredMapId.reset();

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_passFinding =
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

/*!
 * @brief           Checks that a duplicated floor record makes both endpoint
 *                  rooms fail their own floor proof, giving FAIL
 *                  endpoint-room-floor-invalid findings for AX_PASS_04 and
 *                  AX_FLOOR_01 (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     PassageFloorDuplicateFloorRecordFailsViaCanonicalRoomFloorProof)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far}, 0.0);
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    FloorRecord duplicateFloor = snapshot.maps.front().floors.front();
    snapshot.maps.front().floors.push_back(duplicateFloor);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    /* evaluatePassageFloorAgreement()
     * now consults each real endpoint room's own canonical
     * evaluateOneRoomFloorReciprocity() result first; a duplicate floor key
     * makes namedFloorMatchCount > 1 for both endpoint rooms there,
     * yielding a dominant ENDPOINT_ROOM_FLOOR_INVALID before the
     * floorKey-equality AMBIGUOUS path is ever reached -- the underlying
     * contradiction is now diagnosed one layer earlier and more precisely. */
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that a floor that drops one endpoint room's reverse
 *                  membership gives FAIL endpoint-room-floor-invalid findings
 *                  for AX_PASS_04 (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     PassageFloorNonReciprocalMembershipFailsViaCanonicalRoomFloorProof)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far}, 0.0);
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.front().roomRefs.size(), 2U);
    /* Remove one endpoint room's reverse membership while both rooms'
     * forward floorRef still names this same floor. */
    snapshot.maps.front().floors.front().roomRefs.pop_back();

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    /* The endpoint room missing
     * reverse membership now fails its own canonical
     * evaluateOneRoomFloorReciprocity() check (ROOM_FLOOR_NON_RECIPROCAL)
     * first, which evaluatePassageFloorAgreement() propagates as a
     * dominant ENDPOINT_ROOM_FLOOR_INVALID before the floorKey-equality
     * AMBIGUOUS path is ever reached. */
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_PASS_04,
        ReasonCode::PASSAGE_FLOOR_ENDPOINT_ROOM_FLOOR_INVALID);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that two floor records sharing the key a room's floor
 *                  reference names give a FAIL AX_FLOOR_01 finding (duplicate
 *                  added to the snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomFloorDuplicateIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    FloorRecord duplicateFloor = snapshot.maps.front().floors.front();
    duplicateFloor.roomRefs.clear();
    snapshot.maps.front().floors.push_back(duplicateFloor);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_FLOOR_01,
                              ReasonCode::ROOM_FLOOR_DUPLICATE_IDENTITY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a floor listing the same room twice gives a FAIL
 *                  duplicate-reverse-membership finding (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomFloorDuplicateReverseMembershipIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    ASSERT_EQ(snapshot.maps.front().floors.front().roomRefs.size(), 1U);
    snapshot.maps.front().floors.front().roomRefs.push_back(
        snapshot.maps.front().floors.front().roomRefs.front());

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_DUPLICATE_REVERSE_MEMBERSHIP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a room listed by a second floor besides its own
 *                  gives a FAIL claimed-by-multiple-floors finding (edited into
 *                  the snapshot).
 */
TEST(SemanticAxiomEvaluator, RoomClaimedByMultipleFloorsIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    Floor firstFloor;
    test::makeFloor(firstFloor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&firstFloor)), MapStatus::MAP_STATUS_SUCCESS);
    Floor secondFloor;
    test::makeFloor(secondFloor, 2, p_map, {});
    ASSERT_EQ((p_map->addMapFloor(&secondFloor)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_FLOOR_01,
        ReasonCode::ROOM_FLOOR_CLAIMED_BY_MULTIPLE_FLOORS);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/* ------------------------------------------------------------------------
 * Map-completeness truth table, legacy reproduction and divergence
 * ---------------------------------------------------------------------- */

/*!
 * @brief           Checks that a map with no confirmed room is conservatively
 *                  FAIL and not complete, with reason ZERO_CONFIRMED_ROOMS and
 *                  a FAIL AX_COMP_01 finding.
 */
TEST(SemanticAxiomEvaluator, ZeroConfirmedRoomsMakesMapIncomplete)
{
    Atlas                 atlas(0);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_NE(std::find(results.front().reasons.begin(),
                        results.front().reasons.end(),
                        ReasonCode::COMPLETENESS_ZERO_CONFIRMED_ROOMS),
              results.front().reasons.end());

    SemanticGraphSnapshot snapshot2{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot2)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot2, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_compFinding =
        findFindingWithReason(report,
                              AxiomCode::AX_COMP_01,
                              ReasonCode::COMPLETENESS_ZERO_CONFIRMED_ROOMS);
    ASSERT_NE(p_compFinding, nullptr);
    EXPECT_EQ(p_compFinding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a map holding a live prospective room is
 *                  conservatively FAIL and counts one prospective room.
 */
TEST(SemanticAxiomEvaluator, LiveProspectiveRoomMakesMapIncomplete)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room prospective;
    test::makeRoom(prospective,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d::Zero(),
                   Room::RoomVariant::UNDEFINED);
    ASSERT_EQ((p_map->addDetectedMapRoom(&prospective)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_EQ(results.front().prospectiveRoomCount, 1U);
}

/*!
 * @brief           Checks that two rooms sharing one local id make the map's
 *                  completeness FAIL with reason
 *                  COMPLETENESS_DUPLICATE_IDENTITY.
 */
TEST(SemanticAxiomEvaluator, CompletenessDuplicateIdentityIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    /* A second, distinct Room object deliberately reuses local id 1:
     * Map::AddDetectedMapRoom(), unlike AddMapPlane()/AddMapFloor()/
     * AddMapPassage(), has no id-collision handling, so this is reachable
     * through the real capture path rather than requiring direct snapshot
     * mutation. */
    Room duplicateIdRoom;
    test::makeRoom(duplicateIdRoom, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&duplicateIdRoom)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot2{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot2)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot2, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_NE(std::find(results.front().reasons.begin(),
                        results.front().reasons.end(),
                        ReasonCode::COMPLETENESS_DUPLICATE_IDENTITY),
              results.front().reasons.end());
}

/*!
 * @brief           Checks that a confirmed room makes the map's conservative
 *                  completeness UNKNOWN, with reason room-creation-provenance-
 *                  unavailable.
 */
TEST(SemanticAxiomEvaluator, CompletenessRoomCreationProvenanceUnavailable)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::UNKNOWN);
    EXPECT_NE(
        std::find(
            results.front().reasons.begin(),
            results.front().reasons.end(),
            ReasonCode::COMPLETENESS_ROOM_CREATION_PROVENANCE_UNAVAILABLE),
        results.front().reasons.end());
}

/*!
 * @brief           Checks that a failing AX_PASS_03 slot state (known-side room
 *                  still prospective) makes completeness FAIL with reason
 *                  PASSAGE_ENDPOINTS_INVALID.
 */
TEST(SemanticAxiomEvaluator, CompletenessPassageSlotStateFailureIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room unpromoted;
    test::makeRoom(unpromoted,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(0.0, -1.0, 1.0),
                   Room::RoomVariant::UNDEFINED);
    ASSERT_EQ((p_map->addDetectedMapRoom(&unpromoted)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &unpromoted,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((unpromoted.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_NE(std::find(results.front().reasons.begin(),
                        results.front().reasons.end(),
                        ReasonCode::COMPLETENESS_PASSAGE_ENDPOINTS_INVALID),
              results.front().reasons.end());
}

/*!
 * @brief           Checks that a cross-floor passage makes completeness FAIL
 *                  with reasons PASSAGE_ENDPOINTS_INVALID and
 *                  HARD_CONTRADICTION.
 */
TEST(SemanticAxiomEvaluator, CompletenessCrossFloorPassageIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);

    Floor floorA;
    test::makeFloor(floorA, 1, p_map, {&known}, 0.0);
    ASSERT_EQ((p_map->addMapFloor(&floorA)), MapStatus::MAP_STATUS_SUCCESS);
    Floor floorB;
    test::makeFloor(floorB, 2, p_map, {&far}, 3.0);
    ASSERT_EQ((p_map->addMapFloor(&floorB)), MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that two map snapshots sharing one map id both get
 *                  FAIL completeness with reason DUPLICATE_MAP_IDENTITY,
 *                  whether the duplicate is appended or prepended.
 */
TEST(SemanticAxiomEvaluator,
     DuplicateMapIdentityIsFailRegardlessOfInsertionOrder)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot baseSnapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, baseSnapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(baseSnapshot.maps.size(), 1U);
    const MapSnapshot duplicateMap = baseSnapshot.maps.front();

    SemanticGraphSnapshot appendedSnapshot = baseSnapshot;
    appendedSnapshot.maps.push_back(duplicateMap);
    std::vector<MapCompletenessResult> appendedResults{};
    ASSERT_EQ(
        (evaluateMapCompleteness(appendedSnapshot, appendedResults)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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
    std::vector<MapCompletenessResult> prependedResults{};
    ASSERT_EQ(
        (evaluateMapCompleteness(prependedSnapshot, prependedResults)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a room whose passage reference has no key makes
 *                  completeness FAIL with reason
 *                  ROOM_HAS_MALFORMED_PASSAGE_REFERENCE.
 */
TEST(SemanticAxiomEvaluator, CompletenessRoomHasMalformedPassageReferenceIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* Never registered via AddMapPassage, so its own getMap() is nullptr. */
    Passage unregisteredPassage;
    ASSERT_EQ((unregisteredPassage.setId(1)),
              vs_graphs::core::semantic::PassageStatus::PASSAGE_STATUS_SUCCESS);
    ASSERT_EQ((room.setDoorways(&unregisteredPassage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_NE(
        std::find(
            results.front().reasons.begin(),
            results.front().reasons.end(),
            ReasonCode::COMPLETENESS_ROOM_HAS_MALFORMED_PASSAGE_REFERENCE),
        results.front().reasons.end());
}

/*!
 * @brief           Checks that the strongest map the schema can build (two
 *                  reciprocal COMPLETE rooms and a same-floor passage) is
 *                  legacy fully modelled but conservatively UNKNOWN, so the two
 *                  calculations diverge.
 */
TEST(SemanticAxiomEvaluator,
     LegacyCompleteButConservativeUnknownSchemaLimitedDiverges)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wallA)), MapStatus::MAP_STATUS_SUCCESS);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 5.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wallB)), MapStatus::MAP_STATUS_SUCCESS);

    Room known;
    test::makeRoom(known, 1, p_map, &wallA, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((known.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (known.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 1.0, 0.0),
                                           Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Room far;
    test::makeRoom(far, 2, p_map, &wallB, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((far.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (far.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 6.0, 0.0),
                                         Eigen::Vector3d(0.0, 6.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);

    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that a room registered in two collections is counted
 *                  twice by the legacy calculation but once by the conservative
 *                  one.
 */
TEST(SemanticAxiomEvaluator, LegacyReproducesDoubleRegisteredRoomMultiplicity)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room bothCollections;
    test::makeRoom(bothCollections, 1, p_map, nullptr);
    ASSERT_EQ(
        (bothCollections.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((bothCollections.setBoundaryCorners_world_m(
                  {Eigen::Vector3d(0.0, 0.0, 0.0),
                   Eigen::Vector3d(1.0, 0.0, 0.0),
                   Eigen::Vector3d(1.0, 1.0, 0.0),
                   Eigen::Vector3d(0.0, 1.0, 0.0)})),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&bothCollections)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addCandidateMapRoom(&bothCollections)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    /* Legacy counts the same live Room twice (once per GetAllRooms()
     * collection membership); the conservative calculation counts it
     * once. */
    EXPECT_EQ(results.front().legacy.confirmedRoomCount, 2U);
    EXPECT_EQ(results.front().legacy.completeRoomCount, 2U);
    EXPECT_EQ(results.front().confirmedRoomCount, 1U);
    EXPECT_EQ(results.front().completeRoomCount, 1U);
}

/*!
 * @brief           Checks that a passage the rooms do not list back leaves the
 *                  map legacy fully modelled but conservatively FAIL, so the
 *                  calculations diverge.
 */
TEST(SemanticAxiomEvaluator,
     LegacyCompleteButConservativeFailsOnNonReciprocalPassageDiverges)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((known.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (known.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 1.0, 0.0),
                                           Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, nullptr, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((far.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (far.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 6.0, 0.0),
                                         Eigen::Vector3d(0.0, 6.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&known, &far});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0),
                      &far);
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    /* Legacy's own completeness check never inspects Room::getPassages()
     * reciprocity -- only the live known/far room pointers -- so leaving
     * both setDoorways() calls out reproduces a fixture that is legacy-
     * complete but conservatively invalid (non-reciprocal). */

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_TRUE(results.front().legacy.isMapFullyModeled);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::FAIL);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_TRUE(results.front().doLegacyAndConservativeDiverge);
}

/*!
 * @brief           Checks that a map with no floor is legacy fully modelled but
 *                  conservatively UNKNOWN, so the calculations diverge.
 */
TEST(SemanticAxiomEvaluator,
     LegacyCompleteButConservativeUnknownDueToMissingFloorEvidenceDiverges)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wallA;
    test::makeWallPlane(wallA,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wallA)), MapStatus::MAP_STATUS_SUCCESS);
    geometric::Plane wallB;
    test::makeWallPlane(wallB,
                        2,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 5.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wallB)), MapStatus::MAP_STATUS_SUCCESS);

    Room known;
    test::makeRoom(known, 1, p_map, &wallA, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((known.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (known.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 0.0, 0.0),
                                           Eigen::Vector3d(1.0, 1.0, 0.0),
                                           Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room far;
    test::makeRoom(far, 2, p_map, &wallB, Eigen::Vector3d(0.0, 1.0, 1.0));
    ASSERT_EQ((far.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (far.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 5.0, 0.0),
                                         Eigen::Vector3d(1.0, 6.0, 0.0),
                                         Eigen::Vector3d(0.0, 6.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&far)), MapStatus::MAP_STATUS_SUCCESS);
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
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((far.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 1U);
    EXPECT_TRUE(results.front().legacy.isMapFullyModeled);
    EXPECT_EQ(results.front().conservativeResult, AxiomResult::UNKNOWN);
    EXPECT_FALSE(results.front().isComplete);
    EXPECT_TRUE(results.front().doLegacyAndConservativeDiverge);
}

/*!
 * @brief           Checks that a doubly owned wall in map A makes map A FAIL
 *                  with HARD_CONTRADICTION while empty map B fails only with
 *                  ZERO_CONFIRMED_ROOMS.
 */
TEST(SemanticAxiomEvaluator, HardFailureInOneMapDoesNotContaminateAnotherMap)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    unsigned long id{};
    ASSERT_EQ((p_mapA->getId(id)), MapStatus::MAP_STATUS_SUCCESS);
    unsigned long id2{};
    ASSERT_EQ((p_mapB->getId(id2)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_NE(id, id2);

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_mapA->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room ownerOne;
    test::makeRoom(ownerOne, 1, p_mapA, &wall);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&ownerOne)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room ownerTwo;
    test::makeRoom(ownerTwo, 2, p_mapA, &wall);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&ownerTwo)),
              MapStatus::MAP_STATUS_SUCCESS);

    /* mapB has no rooms at all: independently incomplete (zero confirmed
     * rooms), never FAIL for mapA's wall-ownership reason. */

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> results{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, results)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(results.size(), 2U);

    const MapCompletenessResult *p_mapAResult = nullptr;
    const MapCompletenessResult *p_mapBResult = nullptr;
    for (const MapCompletenessResult &result : results)
    {
        unsigned long mapAId{};
        ASSERT_EQ((p_mapA->getId(mapAId)), MapStatus::MAP_STATUS_SUCCESS);
        if (result.mapId == mapAId)
        {
            p_mapAResult = &result;
        }
        else
        {
            unsigned long mapBId{};
            ASSERT_EQ((p_mapB->getId(mapBId)), MapStatus::MAP_STATUS_SUCCESS);
            if (result.mapId == mapBId)
            {
                p_mapBResult = &result;
            }
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

/*!
 * @brief           Checks that a room and its floor split across two map
 *                  snapshots sharing one map id neither crash evaluateState nor
 *                  yield a PASS AX_FLOOR_01 finding (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     DuplicateContainingMapRoomFloorSplitDoesNotCrashAndIsNotPass)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_mapA, nullptr);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_mapB, {});
    ASSERT_EQ((p_mapB->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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
    std::size_t mapSnapshots{};
    ASSERT_EQ(
        (countMapSnapshotsWithId(snapshot, sharedMapId, mapSnapshots)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(mapSnapshots, 2U);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    for (const Finding *p_finding : findingsFor(report, AxiomCode::AX_FLOOR_01))
    {
        EXPECT_NE(p_finding->result, AxiomResult::PASS);
    }
}

/*!
 * @brief           Checks that a known-side reference with no liveness value
 *                  stays UNKNOWN even though the room record it resolves to is
 *                  live, and gives no non-reciprocal FAIL (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator,
     ForwardEndpointLivenessUnavailableStaysUnknownDespiteLiveEnumeratedRecord)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    PassageRecord &passageRecord = snapshot.maps.front().passages.front();
    ASSERT_TRUE(passageRecord.knownSideRoomRef.isLive.has_value());
    passageRecord.knownSideRoomRef.isLive = std::nullopt;
    passageRecord.knownSideRoomRef.livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that a room's passage reference whose reason is not
 *                  NONE gives a FAIL bad-reverse-endpoint finding instead of
 *                  counting as reciprocity proof (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     ReasonInconsistentReciprocalPassageRefIsNotReciprocityProof)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    /* Deliberately never call known.setDoorways(): the adversarial
     * passageRefs entry below is hand-mutated instead. */

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().rooms.size(), 1U);
    ASSERT_TRUE(snapshot.maps.front().rooms.front().passageRefs.empty());
    EntityRef     malformedRef;
    unsigned long mapId{};
    ASSERT_EQ((p_map->getId(mapId)), MapStatus::MAP_STATUS_SUCCESS);
    malformedRef.key    = EntityKey{EntityKind::PASSAGE, mapId, 1};
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
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that two passage records sharing one key both get
 *                  FAIL non-reciprocal AX_PASS_02 findings instead of
 *                  reciprocity proof (duplicate added to the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     ReciprocityAgainstDuplicatePassageIdentityIsNotProof)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room known;
    test::makeRoom(known, 1, p_map, nullptr, Eigen::Vector3d(0.0, -1.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&known)),
              MapStatus::MAP_STATUS_SUCCESS);

    Passage passage;
    test::makePassage(passage,
                      1,
                      p_map,
                      Eigen::Vector4d(0.0, 1.0, 0.0, 0.0),
                      Eigen::Vector3d(0.0, 0.0, 1.0),
                      &known,
                      Eigen::Vector3d(0.0, -1.0, 0.0));
    ASSERT_EQ((p_map->addMapPassage(&passage)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((known.setDoorways(&passage)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().passages.size(), 1U);
    /* Map::AddMapPassage() itself rejects a second Passage object reusing
     * local id 1 ("Passage ID collision"), so this ambiguity is
     * manufactured via direct snapshot mutation instead, mirroring
     * ReverseReferenceDuplicatedIsFail's own precedent: a second,
     * byte-identical PassageRecord sharing this exact key. */
    snapshot.maps.front().passages.push_back(
        snapshot.maps.front().passages.front());

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
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

/*!
 * @brief           Checks that an owner reference with no liveness value does
 *                  not hide a FAIL cross-map owner finding (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator, OwnerLivenessUnavailableDoesNotMaskCrossMapOwner)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_mapA->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_mapB, &wall);
    ASSERT_EQ((p_mapB->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    WallRecord *p_wallRecord = nullptr;
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_01,
                              ReasonCode::WALL_OWNERSHIP_OWNER_CROSS_MAP);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that an owner reference whose reason is not NONE
 *                  gives a FAIL owner-reason-inconsistent finding even though
 *                  the reciprocal link is valid (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     OwnerReasonInconsistentIsFailDespiteValidReciprocal)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    WallRecord &wallRecord = snapshot.maps.front().walls.front();
    ASSERT_EQ(wallRecord.ownerRoomRefs.size(), 1U);
    wallRecord.ownerRoomRefs.front().reason =
        UnavailableReason::ENTITY_HAS_NO_MAP;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_OWNER_REASON_INCONSISTENT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a copy of the wall reference carrying a non-NONE
 *                  reason gives a FAIL contradictory-reciprocal finding instead
 *                  of being skipped (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, ContradictoryReciprocalWithNonNoneReasonIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    /* Shares this wall's own mapId/planeId identity but claims "absent". */
    RawPlaneRef contradictory = roomRecord.wallRefs.front();
    contradictory.reason      = UnavailableReason::NULL_REFERENCE;
    roomRecord.wallRefs.push_back(contradictory);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_WALL_01,
        ReasonCode::WALL_OWNERSHIP_RECIPROCAL_CONTRADICTORY);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a missing declared map on the owner room keeps
 *                  the PASS owner finding and adds an UNKNOWN finding, with no
 *                  mismatch FAIL (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, OwnerDeclaredMapUnavailableCapsAtUnknownNotFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    roomRecord.declaredMapId.reset();

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_passFinding =
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

/*!
 * @brief           Checks that a wall reference claiming to be absent yet
 *                  carrying populated, contradictory data gives a FAIL
 *                  invalid-wall-evidence AX_BOUND_01 finding (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator,
     BoundaryRawRefReasonInconsistentIsInvalidNotUnavailable)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_TRUE(roomRecord.wallRefs.empty());
    RawPlaneRef contradictory;
    contradictory.reason = UnavailableReason::NULL_REFERENCE;
    unsigned long mapId2{};
    ASSERT_EQ((p_map->getId(mapId2)), MapStatus::MAP_STATUS_SUCCESS);
    contradictory.mapId     = mapId2;
    contradictory.planeId   = 5;
    contradictory.isLive    = false;
    contradictory.planeType = geometric::Plane::PlaneVariant::GROUND;
    roomRecord.wallRefs.push_back(contradictory);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_BOUND_01,
                              ReasonCode::ROOM_BOUNDARY_INVALID_WALL_EVIDENCE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that wall evidence whose owner finding is PASS plus
 *                  UNKNOWN (declared map missing) gives an UNKNOWN
 *                  wall-evidence-unavailable finding, not valid evidence
 *                  (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     BoundaryEvidenceFromPassPlusUnknownOwnerIsUnavailable)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((room.setBoundaryStatus(Room::BoundaryStatus::COMPLETE)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (room.setBoundaryCorners_world_m({Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 0.0, 0.0),
                                          Eigen::Vector3d(1.0, 1.0, 0.0),
                                          Eigen::Vector3d(0.0, 1.0, 0.0)})),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    snapshot.maps.front().rooms.front().declaredMapId.reset();

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
        report,
        AxiomCode::AX_BOUND_01,
        ReasonCode::ROOM_BOUNDARY_WALL_EVIDENCE_UNAVAILABLE);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::UNKNOWN);
}

/*!
 * @brief           Checks that a twin reference claiming to be absent yet
 *                  carrying a dead self-twin gives a FAIL
 *                  twin-reason-inconsistent AX_WALL_03 finding (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator, WallTwinReasonInconsistentIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    WallRecord &wallRecord = snapshot.maps.front().walls.front();
    ASSERT_EQ(wallRecord.twinRef.reason, UnavailableReason::NULL_REFERENCE);
    wallRecord.twinRef.mapId     = wallRecord.key.mapId;
    wallRecord.twinRef.planeId   = wallRecord.key.entityId;
    wallRecord.twinRef.planeType = geometric::Plane::PlaneVariant::WALL;
    wallRecord.twinRef.isLive    = false;
    wallRecord.twinRef.wallKey   = wallRecord.key;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
        findFindingWithReason(report,
                              AxiomCode::AX_WALL_03,
                              ReasonCode::WALL_TWIN_REASON_INCONSISTENT);
    ASSERT_NE(p_finding, nullptr);
    EXPECT_EQ(p_finding->result, AxiomResult::FAIL);
}

/*!
 * @brief           Checks that a floor's room reference with a non-NONE reason
 *                  and a not-live flag gives a FAIL reverse-member-invalid
 *                  finding and no PASS (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, FloorReverseMemberReasonInconsistentAndDeadIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    ASSERT_EQ(snapshot.maps.front().floors.size(), 1U);
    FloorRecord &floorRecord = snapshot.maps.front().floors.front();
    ASSERT_EQ(floorRecord.roomRefs.size(), 1U);
    floorRecord.roomRefs.front().reason = UnavailableReason::ENTITY_HAS_NO_MAP;
    floorRecord.roomRefs.front().isLive = false;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
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

/*!
 * @brief           Checks that a wall and its owner room placed in two map
 *                  snapshots sharing one map id give a FAIL containing-map-
 *                  ambiguous finding and no PASS (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator, WallOwnerSplitAcrossDuplicateContainingMapIsFail)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_mapA->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_mapB, nullptr);
    ASSERT_EQ((p_mapB->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that a retired owner room gives a FAIL
 *                  owner-record-not-live finding even when the owner
 *                  reference's liveness is unavailable (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator,
     OwnerRecordNotLiveDominatesReferenceLivenessUnavailable)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((room.setBad()),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    WallRecord &wallRecord = snapshot.maps.front().walls.front();
    ASSERT_EQ(wallRecord.ownerRoomRefs.size(), 1U);
    wallRecord.ownerRoomRefs.front().isLive = std::nullopt;
    wallRecord.ownerRoomRefs.front().livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
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

/*!
 * @brief           Checks that a wall reference whose wall key names this wall
 *                  but whose raw map id disagrees gives a FAIL
 *                  contradictory-reciprocal finding and no PASS (edited into
 *                  the snapshot).
 */
TEST(SemanticAxiomEvaluator, ReciprocalWallKeyRawIdentityMismatchIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    RoomRecord &roomRecord = snapshot.maps.front().rooms.front();
    ASSERT_EQ(roomRecord.wallRefs.size(), 1U);
    RawPlaneRef mismatched = roomRecord.wallRefs.front();
    ASSERT_TRUE(mismatched.wallKey.has_value());
    mismatched.mapId = 4242UL;
    roomRecord.wallRefs.push_back(mismatched);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that a floor listing a room twice, once with
 *                  unavailable liveness, still gives a FAIL
 *                  duplicate-reverse-membership finding and no PASS (edited
 *                  into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     FloorReverseDuplicateWithOneLivenessUnavailableIsFail)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, nullptr);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);
    Floor floor;
    test::makeFloor(floor, 1, p_map, {&room});
    ASSERT_EQ((p_map->addMapFloor(&floor)), MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    FloorRecord &floorRecord = snapshot.maps.front().floors.front();
    ASSERT_EQ(floorRecord.roomRefs.size(), 1U);
    EntityRef unavailableDuplicate = floorRecord.roomRefs.front();
    unavailableDuplicate.isLive    = std::nullopt;
    unavailableDuplicate.livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;
    floorRecord.roomRefs.push_back(unavailableDuplicate);

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that an owner room declaring a different map gives a
 *                  FAIL mismatch finding even when the owner reference's
 *                  liveness is unavailable (edited into the snapshot).
 */
TEST(SemanticAxiomEvaluator,
     OwnerDeclaredMapMismatchDominatesReferenceLivenessUnavailable)
{
    Atlas atlas(0);
    Map  *p_mapA = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapA)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Map *p_mapB = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_mapB)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_mapA,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_mapA->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_mapA, &wall);
    ASSERT_EQ((p_mapA->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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
    unsigned long mapBId{};
    ASSERT_EQ((p_mapB->getId(mapBId)), MapStatus::MAP_STATUS_SUCCESS);
    p_roomRecord->declaredMapId = mapBId;
    ASSERT_EQ(p_wallRecord->ownerRoomRefs.size(), 1U);
    p_wallRecord->ownerRoomRefs.front().isLive = std::nullopt;
    p_wallRecord->ownerRoomRefs.front().livenessUnavailableReason =
        UnavailableReason::NOT_TRACKED_BY_CURRENT_SCHEMA;

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding = findFindingWithReason(
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

/*!
 * @brief           Checks that an owner room that does not list the wall back
 *                  gives a FAIL not-reciprocal finding even when the owner
 *                  reference's liveness is unavailable (edited into the
 *                  snapshot).
 */
TEST(SemanticAxiomEvaluator,
     OwnerNotReciprocalDominatesReferenceLivenessUnavailable)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall;
    test::makeWallPlane(wall,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);
    Room room;
    test::makeRoom(room, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
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

    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    const Finding *p_finding =
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
