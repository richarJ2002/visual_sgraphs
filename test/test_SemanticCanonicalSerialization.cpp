/*!
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

/* White-box: directly exercises serializeDouble(), declared in
 * private_functions.h, since finite/NaN/Infinity/-Infinity/signed-zero
 * encoding is this function's entire documented contract. */
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

TEST(SemanticCanonicalSerialization, SchemaVersionFieldsArePresentAndStable)
{
    Atlas                       atlas(0);
    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);

    EXPECT_EQ(serializeSnapshotTopologyOnly(snapshot)["schema"],
              SEMANTIC_SNAPSHOT_SCHEMA_VERSION);
    EXPECT_EQ(serializeSnapshotFullGeometry(snapshot)["schema"],
              SEMANTIC_SNAPSHOT_SCHEMA_VERSION);
    EXPECT_EQ(serializeEvaluationReport(evaluateState(snapshot))["schema"],
              SEMANTIC_EVALUATION_REPORT_SCHEMA_VERSION);
    EXPECT_EQ(serializeMapCompletenessResults(
                  evaluateMapCompleteness(snapshot))["schema"],
              SEMANTIC_COMPLETENESS_SCHEMA_VERSION);
}

TEST(SemanticCanonicalSerialization, SnapshotTopologyOnlyOmitsGeometryFields)
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

    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
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
    Atlas            atlas(0);
    Map             *p_map = atlas.getCurrentMap();
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
    p_map->addMapPlane(&wall1);
    p_map->addMapPlane(&wall2);
    Room room1;
    test::makeRoom(room1, 1, p_map, &wall1);
    Room room2;
    test::makeRoom(room2, 2, p_map, &wall2);
    p_map->addDetectedMapRoom(&room1);
    p_map->addDetectedMapRoom(&room2);

    const SemanticGraphSnapshot original = captureSemanticGraphSnapshot(&atlas);
    SemanticGraphSnapshot       reordered = original;
    ASSERT_EQ(reordered.maps.size(), 1U);
    std::reverse(reordered.maps.front().rooms.begin(),
                 reordered.maps.front().rooms.end());
    std::reverse(reordered.maps.front().walls.begin(),
                 reordered.maps.front().walls.end());

    EXPECT_EQ(serializeSnapshotFullGeometry(original).dump(),
              serializeSnapshotFullGeometry(reordered).dump());
}

TEST(SemanticCanonicalSerialization,
     SnapshotSerializationPreservesEntityKeyCollisionDuplicates)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();

    Room roomFirst;
    test::makeRoom(roomFirst,
                   1,
                   p_map,
                   nullptr,
                   Eigen::Vector3d(1.0, 0.0, 0.0));
    p_map->addDetectedMapRoom(&roomFirst);
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
    p_map->addCandidateMapRoom(&roomSecondSameId);

    const nlohmann::json json =
        serializeSnapshotTopologyOnly(captureSemanticGraphSnapshot(&atlas));
    EXPECT_EQ(json["maps"][0]["rooms"].size(), 2U);
}

TEST(SemanticCanonicalSerialization,
     EvaluationReportSerializationIsInvariantUnderFindingPermutation)
{
    /* Same rationale as SnapshotSerializationIsInvariantUnderPermutation:
     * re-order a copy of one report's own findings vector in memory rather
     * than comparing across two independently constructed Atlas instances,
     * whose Map ids would genuinely differ. */
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

    const AxiomEvaluationReport original =
        evaluateState(captureSemanticGraphSnapshot(&atlas));
    AxiomEvaluationReport reordered = original;
    std::reverse(reordered.findings.begin(), reordered.findings.end());

    EXPECT_EQ(serializeEvaluationReport(original).dump(),
              serializeEvaluationReport(reordered).dump());
}

TEST(SemanticCanonicalSerialization,
     CompletenessResultsSerializationIsInvariantUnderMapPermutation)
{
    Atlas atlas(0);
    atlas.createNewMap();

    const std::vector<MapCompletenessResult> original =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    ASSERT_EQ(original.size(), 2U);

    /* Serialize the same two results reversed, and confirm the
     * serializer's own defensive re-sort makes both dumps identical
     * regardless of the input vector's order. */
    std::vector<MapCompletenessResult> reordered = original;
    std::reverse(reordered.begin(), reordered.end());
    EXPECT_EQ(serializeMapCompletenessResults(original).dump(),
              serializeMapCompletenessResults(reordered).dump());
}

TEST(SemanticCanonicalSerialization,
     CompletenessSerializationPreservesLegacyMultiplicityDuplicateIds)
{
    Atlas atlas(0);
    Map  *p_map = atlas.getCurrentMap();
    Room  bothCollections;
    test::makeRoom(bothCollections, 1, p_map, nullptr);
    p_map->addDetectedMapRoom(&bothCollections);
    p_map->addCandidateMapRoom(&bothCollections);

    const std::vector<MapCompletenessResult> results =
        evaluateMapCompleteness(captureSemanticGraphSnapshot(&atlas));
    const nlohmann::json  json = serializeMapCompletenessResults(results);
    const nlohmann::json &incompleteRoomIdsJson =
        json["results"][0]["legacy"]["incompleteRoomIds"];
    ASSERT_EQ(incompleteRoomIdsJson.size(), 2U);
    EXPECT_EQ(incompleteRoomIdsJson[0], 1);
    EXPECT_EQ(incompleteRoomIdsJson[1], 1);
}

TEST(SemanticCanonicalSerialization,
     SerializationNeverMutatesInputAndIsRepeatable)
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
    Room roomB;
    test::makeRoom(roomB, 2, p_map, &wall);
    p_map->addDetectedMapRoom(&roomB);
    Room roomA;
    test::makeRoom(roomA, 1, p_map, &wall);
    p_map->addDetectedMapRoom(&roomA);

    const SemanticGraphSnapshot snapshot = captureSemanticGraphSnapshot(&atlas);
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

/* Red-first regression: two findings sharing the same deterministic id
 * (a genuine axiomCode/reasonCode/involvedKeys collision) but different
 * evidence must still serialize identically regardless of input
 * permutation. Before the fix, serializeEvaluationReport() sorted only by
 * id, so std::sort's implementation-defined handling of "equal" elements
 * left the two possible orderings unresolved. */
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

/* Red-first regression: two aggregates sharing axiomCode (a genuine
 * collision -- AxiomEvaluationReport::aggregates is documented as exactly
 * one entry per code, but the serializer must not silently assume that
 * invariant) with unequal payloads must still serialize identically
 * regardless of input permutation. */
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

/* Red-first regression: two completeness results sharing mapId with
 * unequal payloads must still serialize identically regardless of input
 * permutation. */
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

/* Red-first regression: two top-level MapSnapshot entries sharing
 * mapId with unequal payloads must still serialize identically regardless
 * of input permutation, for both projections. */
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

/* Red-first regression: two WallRecord entries colliding on key
 * (identical topology, different geometry only) must produce identical
 * topology-only bytes but a different full-geometry projection -- proving
 * the topology-only order is provably independent of every geometric
 * field, not merely coincidentally so for the current field list. */
TEST(SemanticCanonicalSerialization,
     GeometryOnlyPerturbationNeverChangesTopologyOnlyBytesForCollidingRecords)
{
    WallRecord wallNearOrigin;
    wallNearOrigin.key              = EntityKey{EntityKind::WALL, 3U, 1};
    wallNearOrigin.centroid_World_m = Eigen::Vector3d(0.0, 0.0, 0.0);
    wallNearOrigin.equation_World   = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);

    WallRecord wallFarFromOrigin       = wallNearOrigin;
    wallFarFromOrigin.centroid_World_m = Eigen::Vector3d(100.0, 0.0, 0.0);
    wallFarFromOrigin.equation_World   = Eigen::Vector4d(1.0, 0.0, 0.0, -100.0);

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

/* Every required enum family (AxiomCode::AX_FRAME_01, AxiomResult::
 * PASS, AxiomClass::HARD, ReasonCode::FRAME_TRANSITION_EVALUATION_REQUIRED,
 * EntityKind::ROOM, UnavailableReason::NONE, CapabilityLevel::FULL,
 * MissingProofOwner::NONE) emits a known readable value, and a value
 * outside the declared enum emits a stable "UNKNOWN_..." sentinel rather
 * than an empty string or a crash. */
TEST(SemanticCanonicalSerialization,
     EnumNamesEmitKnownValuesAndStableUnknownSentinel)
{
    EXPECT_EQ(axiomCodeName(AxiomCode::AX_FRAME_01), "AX_FRAME_01");
    EXPECT_EQ(axiomResultName(AxiomResult::PASS), "PASS");
    EXPECT_EQ(axiomClassName(AxiomClass::HARD), "HARD");
    EXPECT_EQ(reasonCodeName(ReasonCode::FRAME_TRANSITION_EVALUATION_REQUIRED),
              "FRAME_TRANSITION_EVALUATION_REQUIRED");
    EXPECT_EQ(entityKindName(EntityKind::ROOM), "ROOM");
    EXPECT_EQ(unavailableReasonName(UnavailableReason::NONE), "NONE");
    EXPECT_EQ(capabilityLevelName(CapabilityLevel::FULL), "FULL");
    EXPECT_EQ(missingProofOwnerName(MissingProofOwner::NONE), "NONE");

    EXPECT_EQ(axiomCodeName(static_cast<AxiomCode>(0xFF)),
              "UNKNOWN_AXIOM_CODE");
    EXPECT_EQ(axiomResultName(static_cast<AxiomResult>(0xFF)),
              "UNKNOWN_AXIOM_RESULT");
    EXPECT_EQ(axiomClassName(static_cast<AxiomClass>(0xFF)),
              "UNKNOWN_AXIOM_CLASS");
    EXPECT_EQ(reasonCodeName(static_cast<ReasonCode>(0xFFFF)),
              "UNKNOWN_REASON_CODE");
    EXPECT_EQ(entityKindName(static_cast<EntityKind>(0xFF)),
              "UNKNOWN_ENTITY_KIND");
    EXPECT_EQ(unavailableReasonName(static_cast<UnavailableReason>(0xFF)),
              "UNKNOWN_UNAVAILABLE_REASON");
    EXPECT_EQ(capabilityLevelName(static_cast<CapabilityLevel>(0xFF)),
              "UNKNOWN_CAPABILITY_LEVEL");
    EXPECT_EQ(missingProofOwnerName(static_cast<MissingProofOwner>(0xFF)),
              "UNKNOWN_MISSING_PROOF_OWNER");
}

/* Readable name fields actually appear in serialized output, not
 * only reachable from a standalone EnumNames.h call. */
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

/* Red-first regression: two OpenPassageHypothesisRecord entries
 * colliding on every topology-only field (supportingWallRef,
 * confirmationCount, missedUpdateCount, lastConfirmedSkeletonFingerprint)
 * but differing only in geometry (centroid_World_m, openingRadius_m,
 * heightSpan_m) must produce identical topology-only bytes but a different
 * full-geometry projection. Before the fix, serializeOpenPassageHypothesis
 * Record() always emitted centroid_World_m/openingRadius_m/heightSpan_m
 * regardless of includeGeometry_in, and the sort used to place these
 * records inside serializeSnapshot() broke ties on centroid_World_m -- a
 * geometric field -- so the topology-only projection was not actually
 * independent of geometry. */
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
    hypothesisNear.centroid_World_m = Eigen::Vector3d(0.0, 0.0, 0.0);
    hypothesisNear.openingRadius_m  = 0.5;
    hypothesisNear.heightSpan_m     = 1.0;

    OpenPassageHypothesisRecord hypothesisFar = hypothesisNear;
    hypothesisFar.centroid_World_m = Eigen::Vector3d(100.0, 0.0, 0.0);
    hypothesisFar.openingRadius_m  = 5.0;
    hypothesisFar.heightSpan_m     = 9.0;

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

/* Red-first regression: two OpenPassageHypothesisRecord entries
 * sharing every field the sort used to compare (supportingWallRef,
 * centroid_World_m) but differing in a field the old comparator ignored
 * (confirmationCount, missedUpdateCount, lastConfirmedSkeletonFingerprint)
 * must still serialize identically regardless of input permutation, for
 * both projections; likewise for UnresolvedWallHypothesisRecord's
 * cloudPointCount/observationCount. */
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

TEST(SemanticCanonicalSerialization, VisitedFlagDoesNotChangeDigests)
{
    /* The visited flag is mission state, not identity: flipping it must not
     * alter either canonical digest, or revisiting a room would break merge
     * matching. */
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
                        1.0,
                        Eigen::Vector3d(0.0, 0.0, 1.0));
    p_map->addMapPlane(&wall);

    Room room;
    test::makeRoom(room, 1, p_map, &wall, Eigen::Vector3d(1.0, 0.0, 1.0));
    p_map->addDetectedMapRoom(&room);

    const std::string topologyBefore =
        serializeSnapshotTopologyOnly(captureSemanticGraphSnapshot(&atlas))
            .dump();
    const std::string geometryBefore =
        serializeSnapshotFullGeometry(captureSemanticGraphSnapshot(&atlas))
            .dump();

    room.setPreviouslyVisited(true);

    EXPECT_EQ(
        serializeSnapshotTopologyOnly(captureSemanticGraphSnapshot(&atlas))
            .dump(),
        topologyBefore);
    EXPECT_EQ(
        serializeSnapshotFullGeometry(captureSemanticGraphSnapshot(&atlas))
            .dump(),
        geometryBefore);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
