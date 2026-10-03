/*!
 * @file            test_SemanticCanonicalSerialization.cpp
 *
 * @brief           Unit tests for the canonical serialization of semantic
 *                  snapshots (SemanticCanonicalSerialization).
 */

/*
 * Focused, ROS/Gazebo-free tests for versioned canonical JSON
 * serialization of SemanticGraphSnapshot, AxiomEvaluationReport, and
 * MapCompletenessResult.
 */

#include "Semantic/SemanticCanonicalSerialization.h"

#include <algorithm>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "Atlas.h"
#include "Geometric/Plane.h"
#include "Map.h"
#include "Semantic/SemanticAxiomEvaluator.h"
#include "Semantic/SemanticAxiomEvaluator/EnumNames.h"
#include "Semantic/SemanticCanonicalSerialization/private_functions.h"
#include "Semantic/SemanticGraphSnapshot.h"
#include "SemanticFixtures.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief           Checks that serializeDouble writes finite values as JSON
 *                  numbers, keeps the sign of zero, writes NaN and the
 *                  infinities as quoted strings, and always produces text that
 *                  parses as JSON.
 *
 *                  Calls serializeDouble() directly, because this encoding of
 *                  finite values, NaN, the infinities and signed zero is the
 *                  whole contract of the function.
 */
TEST(SemanticCanonicalSerialization,
     SerializeDoubleHandlesFiniteNaNInfinityAndSignedZero)
{
    EXPECT_EQ(serializeDouble(1.5).dump(), "1.5");
    EXPECT_EQ(serializeDouble(0.0).dump(), "0.0");
    EXPECT_EQ(serializeDouble(-0.0).dump(), "-0.0");
    EXPECT_EQ(serializeDouble(std::numeric_limits<double>::quiet_NaN()).dump(),
              "\"NaN\"");
    EXPECT_EQ(serializeDouble(std::numeric_limits<double>::infinity()).dump(),
              "\"Infinity\"");
    EXPECT_EQ(serializeDouble(-std::numeric_limits<double>::infinity()).dump(),
              "\"-Infinity\"");

    /* Every emitted token must itself parse back as valid JSON. */
    EXPECT_NO_THROW(nlohmann::json::parse(serializeDouble(1.5).dump()));
    EXPECT_NO_THROW(nlohmann::json::parse(
        serializeDouble(std::numeric_limits<double>::quiet_NaN()).dump()));
}

/*!
 * @brief           Checks that the snapshot, evaluation report and completeness
 *                  JSON each carry their schema version constant.
 */
TEST(SemanticCanonicalSerialization, SchemaVersionFieldsArePresentAndStable)
{
    Atlas                 atlas(0);
    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);

    EXPECT_EQ(serializeSnapshotTopologyOnly(snapshot)["schema"],
              SEMANTIC_SNAPSHOT_SCHEMA_VERSION);
    EXPECT_EQ(serializeSnapshotFullGeometry(snapshot)["schema"],
              SEMANTIC_SNAPSHOT_SCHEMA_VERSION);
    AxiomEvaluationReport report{};
    ASSERT_EQ(
        (evaluateState(snapshot, report)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(serializeEvaluationReport(report)["schema"],
              SEMANTIC_EVALUATION_REPORT_SCHEMA_VERSION);
    std::vector<MapCompletenessResult> completenessResults{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, completenessResults)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(serializeMapCompletenessResults(completenessResults)["schema"],
              SEMANTIC_COMPLETENESS_SCHEMA_VERSION);
}

/*!
 * @brief           Checks that the topology-only snapshot JSON leaves out a
 *                  wall's plane equation and centroid, which the full-geometry
 *                  JSON includes.
 */
TEST(SemanticCanonicalSerialization, SnapshotTopologyOnlyOmitsGeometryFields)
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
    const nlohmann::json topologyJson = serializeSnapshotTopologyOnly(snapshot);
    const nlohmann::json &wallJson    = topologyJson["maps"][0]["walls"][0];
    EXPECT_FALSE(wallJson.contains("equation_World"));
    EXPECT_FALSE(wallJson.contains("centroid_World_m"));
    EXPECT_TRUE(wallJson.contains("key"));
    EXPECT_TRUE(wallJson.contains("ownerRoomRefs"));

    const nlohmann::json geometryJson = serializeSnapshotFullGeometry(snapshot);
    const nlohmann::json &wallGeometryJson =
        geometryJson["maps"][0]["walls"][0];
    EXPECT_TRUE(wallGeometryJson.contains("equation_World"));
    EXPECT_TRUE(wallGeometryJson.contains("centroid_World_m"));
}

/*!
 * @brief           Checks that reversing the order of the room and wall records
 *                  inside one snapshot does not change the full-geometry JSON.
 */
TEST(SemanticCanonicalSerialization,
     SnapshotSerializationIsInvariantUnderPermutation)
{
    /* A separate Atlas assigns its Map a different id (Map::nextId is a
     * process-wide monotonic counter), so two independently constructed
     * "equivalent" fixtures would genuinely differ in every mapId-bearing
     * field. Proving permutation-independence instead re-orders a copy of
     * one snapshot's own record vectors in memory, so mapId and every
     * other field stays identical between the two serialized inputs and
     * only container order differs. */
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    geometric::Plane wall1;
    test::makeWallPlane(wall1,
                        1,
                        p_map,
                        Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                        Eigen::Vector3d::UnitY(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    geometric::Plane wall2;
    test::makeWallPlane(wall2,
                        2,
                        p_map,
                        Eigen::Vector4d(0.0, 1.0, 0.0, 5.0),
                        Eigen::Vector3d::UnitX(),
                        Eigen::Vector3d::UnitZ(),
                        1.0,
                        1.0);
    ASSERT_EQ((p_map->addMapPlane(&wall1)), MapStatus::MAP_STATUS_SUCCESS);
    ASSERT_EQ((p_map->addMapPlane(&wall2)), MapStatus::MAP_STATUS_SUCCESS);
    Room room1;
    test::makeRoom(room1, 1, p_map, &wall1);
    Room room2;
    test::makeRoom(room2, 2, p_map, &wall2);
    ASSERT_EQ((p_map->addDetectedMapRoom(&room1)),
              MapStatus::MAP_STATUS_SUCCESS);
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

    EXPECT_EQ(serializeSnapshotFullGeometry(original).dump(),
              serializeSnapshotFullGeometry(reordered).dump());
}

/*!
 * @brief           Checks that two rooms with the same entity key are both
 *                  written to the topology-only JSON rather than merged into
 *                  one.
 */
TEST(SemanticCanonicalSerialization,
     SnapshotSerializationPreservesEntityKeyCollisionDuplicates)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);

    Room roomFirst;
    test::makeRoom(roomFirst,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(1.0, 0.0, 0.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&roomFirst)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room roomSecondSameId;
    test::makeRoom(roomSecondSameId,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(2.0, 0.0, 0.0));
    /* Room permits a genuine same-map id collision through
     * AddCandidateMapRoom() (a bare std::set<Room *>::insert() with no id
     * bookkeeping), confirmed by direct source read during P1.1's residual
     * repair. */
    ASSERT_EQ((p_map->addCandidateMapRoom(&roomSecondSameId)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    const nlohmann::json json = serializeSnapshotTopologyOnly(snapshot);
    EXPECT_EQ(json["maps"][0]["rooms"].size(), 2U);
}

/*!
 * @brief           Checks that reversing the findings of an evaluation report
 *                  does not change the serialized report.
 */
TEST(SemanticCanonicalSerialization,
     EvaluationReportSerializationIsInvariantUnderFindingPermutation)
{
    /* Same rationale as SnapshotSerializationIsInvariantUnderPermutation:
     * re-order a copy of one report's own findings vector in memory rather
     * than comparing across two independently constructed Atlas instances,
     * whose Map ids would genuinely differ. */
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

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    AxiomEvaluationReport original{};
    ASSERT_EQ(
        (evaluateState(snapshot, original)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    AxiomEvaluationReport reordered = original;
    std::reverse(reordered.findings.begin(), reordered.findings.end());

    EXPECT_EQ(serializeEvaluationReport(original).dump(),
              serializeEvaluationReport(reordered).dump());
}

/*!
 * @brief           Checks that serializing the map completeness results gives
 *                  the same text whatever order the per-map results arrive in.
 */
TEST(SemanticCanonicalSerialization,
     CompletenessResultsSerializationIsInvariantUnderMapPermutation)
{
    Atlas atlas(0);
    ASSERT_EQ((atlas.createNewMap()), AtlasStatus::ATLAS_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    std::vector<MapCompletenessResult> original{};
    ASSERT_EQ(
        (evaluateMapCompleteness(snapshot, original)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    ASSERT_EQ(original.size(), 2U);

    /* Serialize the same two results reversed, and confirm the
     * serializer's own defensive re-sort makes both dumps identical
     * regardless of the input vector's order. */
    std::vector<MapCompletenessResult> reordered = original;
    std::reverse(reordered.begin(), reordered.end());
    EXPECT_EQ(serializeMapCompletenessResults(original).dump(),
              serializeMapCompletenessResults(reordered).dump());
}

/*!
 * @brief           Checks that a room held in both the detected and candidate
 *                  sets appears twice, with the same id, in the legacy
 *                  incomplete-room list.
 */
TEST(SemanticCanonicalSerialization,
     CompletenessSerializationPreservesLegacyMultiplicityDuplicateIds)
{
    Atlas atlas(0);
    Map  *p_map = nullptr;
    ASSERT_EQ((atlas.getCurrentMap(p_map)), AtlasStatus::ATLAS_STATUS_SUCCESS);
    Room bothCollections;
    test::makeRoom(bothCollections, 1, p_map, nullptr);
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
    const nlohmann::json  json = serializeMapCompletenessResults(results);
    const nlohmann::json &incompleteRoomIdsJson =
        json["results"][0]["legacy"]["incompleteRoomIds"];
    ASSERT_EQ(incompleteRoomIdsJson.size(), 2U);
    EXPECT_EQ(incompleteRoomIdsJson[0], 1);
    EXPECT_EQ(incompleteRoomIdsJson[1], 1);
}

/*!
 * @brief           Checks that serializing a snapshot twice gives identical
 *                  text and leaves the snapshot's own record order unchanged.
 */
TEST(SemanticCanonicalSerialization,
     SerializationNeverMutatesInputAndIsRepeatable)
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
    Room roomB;
    test::makeRoom(roomB, 2, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&roomB)),
              MapStatus::MAP_STATUS_SUCCESS);
    Room roomA;
    test::makeRoom(roomA, 1, p_map, &wall);
    ASSERT_EQ((p_map->addDetectedMapRoom(&roomA)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    /* snapshot.maps[0].rooms is already sorted ascending by key (roomA
     * before roomB) regardless of the reversed registration order above. */
    ASSERT_EQ(snapshot.maps[0].rooms.size(), 2U);
    EXPECT_EQ(snapshot.maps[0].rooms[0].key.entityId, 1);
    EXPECT_EQ(snapshot.maps[0].rooms[1].key.entityId, 2);

    const std::string firstDump =
        serializeSnapshotFullGeometry(snapshot).dump();
    const std::string secondDump =
        serializeSnapshotFullGeometry(snapshot).dump();
    EXPECT_EQ(firstDump, secondDump);

    /* snapshot's own record order is untouched by serialization. */
    EXPECT_EQ(snapshot.maps[0].rooms[0].key.entityId, 1);
    EXPECT_EQ(snapshot.maps[0].rooms[1].key.entityId, 2);
}

/*!
 * @brief           Checks that two findings with the same id but different
 *                  evidence serialize identically in either input order.
 *
 *                  Regression test: the serializer once sorted findings by id
 *                  only, so two findings with the same id (the same axiom,
 *                  reason and involved keys) could come out in either order.
 */
TEST(SemanticCanonicalSerialization,
     FindingsWithSameIdAndUnequalEvidenceAreOrderedDeterministically)
{
    Finding findingLowEvidence;
    findingLowEvidence.id             = "same-id";
    findingLowEvidence.axiomCode      = AxiomCode::AX_WALL_01;
    findingLowEvidence.result         = AxiomResult::FAIL;
    findingLowEvidence.classification = AxiomClass::HARD;
    findingLowEvidence.reasonCode     = ReasonCode::WALL_OWNERSHIP_OWNER_BAD;
    findingLowEvidence.evidence.observedCount = 1U;

    Finding findingHighEvidence                = findingLowEvidence;
    findingHighEvidence.evidence.observedCount = 2U;

    AxiomEvaluationReport reportInOrder;
    reportInOrder.findings = {findingLowEvidence, findingHighEvidence};
    AxiomEvaluationReport reportReversed;
    reportReversed.findings = {findingHighEvidence, findingLowEvidence};

    EXPECT_EQ(serializeEvaluationReport(reportInOrder).dump(),
              serializeEvaluationReport(reportReversed).dump());
}

/*!
 * @brief           Checks that two aggregate results with the same axiom code
 *                  but different payloads serialize identically in either input
 *                  order.
 *
 *                  A report should hold one aggregate per axiom code, but the
 *                  serializer must not rely on that: a collision must still
 *                  give one fixed order.
 */
TEST(SemanticCanonicalSerialization,
     AggregatesWithSameAxiomCodeAndUnequalPayloadAreOrderedDeterministically)
{
    AggregateAxiomResult aggregatePass;
    aggregatePass.axiomCode                = AxiomCode::AX_WALL_01;
    aggregatePass.result                   = AxiomResult::PASS;
    aggregatePass.classification           = AxiomClass::HARD;
    aggregatePass.contributingFindingCount = 1U;

    AggregateAxiomResult aggregateFail     = aggregatePass;
    aggregateFail.result                   = AxiomResult::FAIL;
    aggregateFail.contributingFindingCount = 3U;

    AxiomEvaluationReport reportInOrder;
    reportInOrder.aggregates = {aggregatePass, aggregateFail};
    AxiomEvaluationReport reportReversed;
    reportReversed.aggregates = {aggregateFail, aggregatePass};

    EXPECT_EQ(serializeEvaluationReport(reportInOrder).dump(),
              serializeEvaluationReport(reportReversed).dump());
}

/*!
 * @brief           Checks that two completeness results with the same map id
 *                  but different payloads serialize identically in either input
 *                  order.
 */
TEST(
    SemanticCanonicalSerialization,
    CompletenessResultsWithSameMapIdAndUnequalPayloadAreOrderedDeterministically)
{
    MapCompletenessResult resultIncomplete;
    resultIncomplete.mapId              = 7U;
    resultIncomplete.conservativeResult = AxiomResult::FAIL;
    resultIncomplete.isComplete         = false;
    resultIncomplete.confirmedRoomCount = 1U;

    MapCompletenessResult resultComplete = resultIncomplete;
    resultComplete.conservativeResult    = AxiomResult::PASS;
    resultComplete.isComplete            = true;
    resultComplete.confirmedRoomCount    = 2U;

    EXPECT_EQ(
        serializeMapCompletenessResults({resultIncomplete, resultComplete})
            .dump(),
        serializeMapCompletenessResults({resultComplete, resultIncomplete})
            .dump());
}

/*!
 * @brief           Checks that two map snapshots with the same map id but
 *                  different rooms serialize identically in either input order,
 *                  for both the full-geometry and topology-only forms.
 */
TEST(SemanticCanonicalSerialization,
     MapSnapshotsWithSameMapIdAndUnequalPayloadAreOrderedDeterministically)
{
    MapSnapshot mapVariantA;
    mapVariantA.mapId        = 9U;
    mapVariantA.isCurrentMap = true;
    RoomRecord roomA;
    roomA.key = EntityKey{EntityKind::ROOM, 9U, 1};
    mapVariantA.rooms.push_back(roomA);

    MapSnapshot mapVariantB;
    mapVariantB.mapId        = 9U;
    mapVariantB.isCurrentMap = true;
    RoomRecord roomB;
    roomB.key    = EntityKey{EntityKind::ROOM, 9U, 2};
    roomB.isLive = false;
    mapVariantB.rooms.push_back(roomB);

    SemanticGraphSnapshot snapshotInOrder;
    snapshotInOrder.maps = {mapVariantA, mapVariantB};
    SemanticGraphSnapshot snapshotReversed;
    snapshotReversed.maps = {mapVariantB, mapVariantA};

    EXPECT_EQ(serializeSnapshotFullGeometry(snapshotInOrder).dump(),
              serializeSnapshotFullGeometry(snapshotReversed).dump());
    EXPECT_EQ(serializeSnapshotTopologyOnly(snapshotInOrder).dump(),
              serializeSnapshotTopologyOnly(snapshotReversed).dump());
}

/*!
 * @brief           Checks that two wall records that differ only in geometry
 *                  give identical topology-only text but different
 *                  full-geometry text.
 *
 *                  This shows that the topology-only order depends on no
 *                  geometric field at all, not only on the fields that exist
 *                  today.
 */
TEST(SemanticCanonicalSerialization,
     GeometryOnlyPerturbationNeverChangesTopologyOnlyBytesForCollidingRecords)
{
    WallRecord wallNearOrigin;
    wallNearOrigin.key                   = EntityKey{EntityKind::WALL, 3U, 1};
    wallNearOrigin.planeCentroid_world_m = Eigen::Vector3d(0.0, 0.0, 0.0);
    wallNearOrigin.planeEquation_world   = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);

    WallRecord wallFarFromOrigin            = wallNearOrigin;
    wallFarFromOrigin.planeCentroid_world_m = Eigen::Vector3d(100.0, 0.0, 0.0);
    wallFarFromOrigin.planeEquation_world =
        Eigen::Vector4d(1.0, 0.0, 0.0, -100.0);

    MapSnapshot mapWithNear;
    mapWithNear.mapId = 3U;
    mapWithNear.walls = {wallNearOrigin};
    MapSnapshot mapWithFar;
    mapWithFar.mapId = 3U;
    mapWithFar.walls = {wallFarFromOrigin};

    const nlohmann::json topologyNear =
        serializeMapSnapshot(mapWithNear, /*includeGeometry_in=*/false);
    const nlohmann::json topologyFar =
        serializeMapSnapshot(mapWithFar, /*includeGeometry_in=*/false);
    EXPECT_EQ(topologyNear.dump(), topologyFar.dump());

    const nlohmann::json geometryNear =
        serializeMapSnapshot(mapWithNear, /*includeGeometry_in=*/true);
    const nlohmann::json geometryFar =
        serializeMapSnapshot(mapWithFar, /*includeGeometry_in=*/true);
    EXPECT_NE(geometryNear.dump(), geometryFar.dump());
}

/*!
 * @brief           Checks that every enum name function returns the readable
 *                  name for a known value and a fixed UNKNOWN_ name for an
 *                  out-of-range value.
 *
 *                  Covers AxiomCode, AxiomResult, AxiomClass, ReasonCode,
 *                  EntityKind, UnavailableReason, CapabilityLevel and
 *                  MissingProofOwner.
 */
TEST(SemanticCanonicalSerialization,
     EnumNamesEmitKnownValuesAndStableUnknownSentinel)
{
    std::string axiomCodeName2{};
    ASSERT_EQ(
        (axiomCodeName(AxiomCode::AX_FRAME_01, axiomCodeName2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(axiomCodeName2, "AX_FRAME_01");
    std::string axiomResultName2{};
    ASSERT_EQ(
        (axiomResultName(AxiomResult::PASS, axiomResultName2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(axiomResultName2, "PASS");
    std::string axiomClassName2{};
    ASSERT_EQ(
        (axiomClassName(AxiomClass::HARD, axiomClassName2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(axiomClassName2, "HARD");
    std::string reasonCodeName2{};
    ASSERT_EQ(
        (reasonCodeName(ReasonCode::FRAME_TRANSITION_EVALUATION_REQUIRED,
                        reasonCodeName2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(reasonCodeName2, "FRAME_TRANSITION_EVALUATION_REQUIRED");
    std::string entityKindName2{};
    ASSERT_EQ(
        (entityKindName(EntityKind::ROOM, entityKindName2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(entityKindName2, "ROOM");
    std::string unavailableReasonName2{};
    ASSERT_EQ(
        (unavailableReasonName(UnavailableReason::NONE,
                               unavailableReasonName2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(unavailableReasonName2, "NONE");
    std::string capabilityLevelName2{};
    ASSERT_EQ(
        (capabilityLevelName(CapabilityLevel::FULL, capabilityLevelName2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(capabilityLevelName2, "FULL");
    std::string missingProofOwnerName2{};
    ASSERT_EQ(
        (missingProofOwnerName(MissingProofOwner::NONE,
                               missingProofOwnerName2)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(missingProofOwnerName2, "NONE");

    std::string axiomCodeName3{};
    ASSERT_EQ(
        (axiomCodeName(static_cast<AxiomCode>(0xFF), axiomCodeName3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(axiomCodeName3, "UNKNOWN_AXIOM_CODE");
    std::string axiomResultName3{};
    ASSERT_EQ(
        (axiomResultName(static_cast<AxiomResult>(0xFF), axiomResultName3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(axiomResultName3, "UNKNOWN_AXIOM_RESULT");
    std::string axiomClassName3{};
    ASSERT_EQ(
        (axiomClassName(static_cast<AxiomClass>(0xFF), axiomClassName3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(axiomClassName3, "UNKNOWN_AXIOM_CLASS");
    std::string reasonCodeName3{};
    ASSERT_EQ(
        (reasonCodeName(static_cast<ReasonCode>(0xFF), reasonCodeName3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(reasonCodeName3, "UNKNOWN_REASON_CODE");
    std::string entityKindName3{};
    ASSERT_EQ(
        (entityKindName(static_cast<EntityKind>(0xFF), entityKindName3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(entityKindName3, "UNKNOWN_ENTITY_KIND");
    std::string unavailableReasonName3{};
    ASSERT_EQ(
        (unavailableReasonName(static_cast<UnavailableReason>(0xFF),
                               unavailableReasonName3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(unavailableReasonName3, "UNKNOWN_UNAVAILABLE_REASON");
    std::string capabilityLevelName3{};
    ASSERT_EQ(
        (capabilityLevelName(static_cast<CapabilityLevel>(0xFF),
                             capabilityLevelName3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(capabilityLevelName3, "UNKNOWN_CAPABILITY_LEVEL");
    std::string missingProofOwnerName3{};
    ASSERT_EQ(
        (missingProofOwnerName(static_cast<MissingProofOwner>(0xFF),
                               missingProofOwnerName3)),
        SemanticAxiomEvaluatorStatus::SEMANTIC_AXIOM_EVALUATOR_STATUS_SUCCESS);
    EXPECT_EQ(missingProofOwnerName3, "UNKNOWN_MISSING_PROOF_OWNER");
}

/*!
 * @brief           Checks that the serialized findings carry the readable
 *                  axiom, result, classification and reason names.
 *
 *                  The names must appear in the serialized text itself, not
 *                  only through the EnumNames.h functions.
 */
TEST(SemanticCanonicalSerialization, SerializedFindingsCarryReadableNameFields)
{
    Finding finding;
    finding.id             = "id";
    finding.axiomCode      = AxiomCode::AX_WALL_01;
    finding.result         = AxiomResult::FAIL;
    finding.classification = AxiomClass::HARD;
    finding.reasonCode     = ReasonCode::WALL_OWNERSHIP_OWNER_BAD;

    AxiomEvaluationReport report;
    report.findings = {finding};

    const nlohmann::json  json        = serializeEvaluationReport(report);
    const nlohmann::json &findingJson = json["findings"][0];
    EXPECT_EQ(findingJson["axiomCodeName"], "AX_WALL_01");
    EXPECT_EQ(findingJson["resultName"], "FAIL");
    EXPECT_EQ(findingJson["classificationName"], "HARD");
    EXPECT_EQ(findingJson["reasonCodeName"], "WALL_OWNERSHIP_OWNER_BAD");
}

/*!
 * @brief           Checks that open-passage hypotheses that differ only in
 *                  geometry give identical topology-only text, which omits the
 *                  geometry fields, and different full-geometry text.
 *
 *                  Regression test: the serializer once wrote
 *                  openingCentroid_world_m, openingRadius_m and heightSpan_m in
 *                  the topology-only form too, and the sort broke ties on
 *                  openingCentroid_world_m, so that form depended on geometry.
 */
TEST(SemanticCanonicalSerialization,
     OpenPassageHypothesisGeometryOnlyDriftNeverChangesTopologyOnlyBytes)
{
    OpenPassageHypothesisRecord hypothesisNear;
    hypothesisNear.supportingWallRef.mapId          = 3U;
    hypothesisNear.supportingWallRef.planeId        = 7;
    hypothesisNear.supportingWallRef.reason         = UnavailableReason::NONE;
    hypothesisNear.confirmationCount                = 4U;
    hypothesisNear.missedUpdateCount                = 1U;
    hypothesisNear.lastConfirmedSkeletonFingerprint = 42U;
    hypothesisNear.openingCentroid_world_m = Eigen::Vector3d(0.0, 0.0, 0.0);
    hypothesisNear.openingRadius_m         = 0.5;
    hypothesisNear.heightSpan_m            = 1.0;

    OpenPassageHypothesisRecord hypothesisFar = hypothesisNear;
    hypothesisFar.openingCentroid_world_m = Eigen::Vector3d(100.0, 0.0, 0.0);
    hypothesisFar.openingRadius_m         = 5.0;
    hypothesisFar.heightSpan_m            = 9.0;

    SemanticGraphSnapshot snapshotNear;
    snapshotNear.managerPrivateOpenPassageHypotheses = {hypothesisNear};
    SemanticGraphSnapshot snapshotFar;
    snapshotFar.managerPrivateOpenPassageHypotheses = {hypothesisFar};

    EXPECT_EQ(serializeSnapshotTopologyOnly(snapshotNear).dump(),
              serializeSnapshotTopologyOnly(snapshotFar).dump());
    EXPECT_NE(serializeSnapshotFullGeometry(snapshotNear).dump(),
              serializeSnapshotFullGeometry(snapshotFar).dump());

    const nlohmann::json topologyJson =
        serializeSnapshotTopologyOnly(snapshotNear);
    const nlohmann::json &hypothesisJson =
        topologyJson["managerPrivateOpenPassageHypotheses"][0];
    EXPECT_FALSE(hypothesisJson.contains("centroid_World_m"));
    EXPECT_FALSE(hypothesisJson.contains("openingRadius_m"));
    EXPECT_FALSE(hypothesisJson.contains("heightSpan_m"));
    EXPECT_TRUE(hypothesisJson.contains("confirmationCount"));
}

/*!
 * @brief           Checks that open-passage and unresolved-wall hypothesis
 *                  records that differ only in counters the sort once ignored
 *                  serialize identically in either input order.
 *
 *                  Regression test: the sort once compared only
 *                  supportingWallRef and openingCentroid_world_m of
 *                  open-passage hypotheses, and ignored cloudPointCount and
 *                  observationCount of unresolved-wall hypotheses.
 */
TEST(SemanticCanonicalSerialization,
     HypothesisRecordsWithUnequalIgnoredFieldsAreOrderedDeterministically)
{
    OpenPassageHypothesisRecord openA;
    openA.supportingWallRef.mapId   = 1U;
    openA.supportingWallRef.planeId = 1;
    openA.supportingWallRef.reason  = UnavailableReason::NONE;
    openA.confirmationCount         = 1U;

    OpenPassageHypothesisRecord openB = openA;
    openB.confirmationCount           = 9U;

    UnresolvedWallHypothesisRecord wallA;
    wallA.wallRef.mapId    = 1U;
    wallA.wallRef.planeId  = 2;
    wallA.wallRef.reason   = UnavailableReason::NONE;
    wallA.unresolvedCycles = 1U;
    wallA.cloudPointCount  = 1U;

    UnresolvedWallHypothesisRecord wallB = wallA;
    wallB.cloudPointCount                = 9U;
    wallB.observationCount               = 3U;

    SemanticGraphSnapshot snapshotInOrder;
    snapshotInOrder.managerPrivateOpenPassageHypotheses    = {openA, openB};
    snapshotInOrder.managerPrivateUnresolvedWallHypotheses = {wallA, wallB};
    SemanticGraphSnapshot snapshotReversed;
    snapshotReversed.managerPrivateOpenPassageHypotheses    = {openB, openA};
    snapshotReversed.managerPrivateUnresolvedWallHypotheses = {wallB, wallA};

    EXPECT_EQ(serializeSnapshotTopologyOnly(snapshotInOrder).dump(),
              serializeSnapshotTopologyOnly(snapshotReversed).dump());
    EXPECT_EQ(serializeSnapshotFullGeometry(snapshotInOrder).dump(),
              serializeSnapshotFullGeometry(snapshotReversed).dump());
}

/*!
 * @brief           Checks that marking a room as previously visited changes
 *                  neither the topology-only nor the full-geometry snapshot
 *                  text.
 */
TEST(SemanticCanonicalSerialization, VisitedFlagDoesNotChangeDigests)
{
    /* The visited flag is mission state, not identity: flipping it must not
     * alter either canonical digest, or revisiting a room would break merge
     * matching. */
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
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addMapPlane(&wall)), MapStatus::MAP_STATUS_SUCCESS);

    Room room;
    test::makeRoom(room, 1, p_map, &wall, Eigen::Vector3d(1.0, 0.0, 1.0));
    ASSERT_EQ((p_map->addDetectedMapRoom(&room)),
              MapStatus::MAP_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    const std::string topologyBefore =
        serializeSnapshotTopologyOnly(snapshot).dump();
    SemanticGraphSnapshot snapshot2{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot2)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    const std::string geometryBefore =
        serializeSnapshotFullGeometry(snapshot2).dump();

    ASSERT_EQ((room.setPreviouslyVisited(true)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);

    SemanticGraphSnapshot snapshot3{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot3)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    EXPECT_EQ(serializeSnapshotTopologyOnly(snapshot3).dump(), topologyBefore);
    SemanticGraphSnapshot snapshot4{};
    ASSERT_EQ(
        (captureSemanticGraphSnapshot(&atlas, snapshot4)),
        SemanticGraphSnapshotStatus::SEMANTIC_GRAPH_SNAPSHOT_STATUS_SUCCESS);
    EXPECT_EQ(serializeSnapshotFullGeometry(snapshot4).dump(), geometryBefore);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
