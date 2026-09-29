/*!
 * Focused tests: plane-gated geometric verification
 * (semantic::SemanticVerify) against synthetic two-room scenarios with a known
 * ground-truth SE(3) transform, plus the floor gate and the
 * EdgePlaneTransformSE3 residual.
 */

#include "Semantic/SemanticVerify.h"

#include "Geometric/Plane.h"
#include "LoopClosing.h"
#include "Map.h"
#include "OptimizableTypes.h"
#include "Semantic/Floor.h"
#include "Semantic/Room.h"
#include "Types/objects/SystemParams.h"
#include "Utils/Utils/objects/Utils.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <memory>
#include <random>
#include <rclcpp/logging.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace
{

struct RawWall
{
    int             id;
    Eigen::Vector3d normal;
    double          d;
    Eigen::Vector3d centroid;
};

/*! Owns geometric::Plane objects for the lifetime of one synthetic room fixture
 * and associates them with a semantic::Room via the genuine
 * semantic::Room::setWalls() API. */
struct SyntheticRoom
{
    semantic::Room                                 room;
    std::vector<std::unique_ptr<geometric::Plane>> ownedWalls;

    void addWall(const RawWall &wall_in)
    {
        auto wall = std::make_unique<geometric::Plane>();
        ASSERT_EQ((wall->setId(wall_in.id)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((wall->setPlaneType(geometric::Plane::PlaneVariant::WALL)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((wall->setGlobalEquation(
                      g2o::Plane3D(Eigen::Vector4d(wall_in.normal.x(),
                                                   wall_in.normal.y(),
                                                   wall_in.normal.z(),
                                                   wall_in.d)))),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((wall->setCentroid(wall_in.centroid)),
                  geometric::PlaneStatus::PLANE_STATUS_SUCCESS);
        ASSERT_EQ((room.setWalls(wall.get())),
                  vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
        ownedWalls.push_back(std::move(wall));
    }
};

/*! Transforms a raw plane equation the same way
 * geometric::Plane::transformPlaneEquation does (geometric::Plane.cc:410-467,
 * scale fixed at 1): n' = R n; d' = d - n'^T t. */
RawWall transformWall(const RawWall         &source_in,
                      const Eigen::Matrix3d &rotation_in,
                      const Eigen::Vector3d &translation_in,
                      const int              newId_in)
{
    RawWall transformed;
    transformed.id       = newId_in;
    transformed.normal   = rotation_in * source_in.normal;
    transformed.d        = source_in.d - transformed.normal.dot(translation_in);
    transformed.centroid = rotation_in * source_in.centroid + translation_in;
    return transformed;
}

/*! A well-conditioned, deliberately ASYMMETRIC 4-wall room: three
 * axis-aligned faces with distinct, non-zero offsets (breaking the axis-
 * permutation symmetry a shared d=0 origin corner would otherwise have)
 * plus one oblique face with non-uniform normal components. Without this
 * asymmetry, a wrong axis-permuted rotation can satisfy every gate exactly
 * as well as the true one, producing genuine ambiguity rather than a test
 * bug. */
std::vector<RawWall> makeReferenceWalls()
{
    return {
        {1,
         Eigen::Vector3d(1.0, 0.0, 0.0),
         -0.2,
         Eigen::Vector3d(0.2, 1.5, 0.7)},
        {2,
         Eigen::Vector3d(0.0, 1.0, 0.0),
         -3.1,
         Eigen::Vector3d(1.0, 3.1, 0.7)},
        {3,
         Eigen::Vector3d(0.0, 0.0, 1.0),
         -1.6,
         Eigen::Vector3d(1.0, 1.5, 1.6)},
        {4,
         Eigen::Vector3d(2.0, 1.0, 0.5).normalized(),
         -2.7,
         Eigen::Vector3d(1.1, 0.6, 0.3)},
    };
}

semantic::SemanticMergeRoomEvidence
    makeMergeEvidence(const int              roomId_in,
                      const int              farSideRoomId_in,
                      const Eigen::Vector3d &knownSideDirection_in)
{
    semantic::SemanticMergeRoomEvidence evidence;
    evidence.context.roomId  = roomId_in;
    evidence.context.roomTag = "room_" + std::to_string(roomId_in);
    for (const RawWall &wall : makeReferenceWalls())
    {
        semantic::VerifyWallObservation observation;
        observation.wallId         = wall.id;
        observation.normal_World   = wall.normal;
        observation.d              = wall.d;
        observation.centroid_World = wall.centroid;
        evidence.walls.push_back(observation);
    }

    semantic::PassageContext passage;
    passage.id                       = 7;
    passage.isPassable               = true;
    passage.hasKnownSideRoom         = true;
    passage.knownSideRoomId          = roomId_in;
    passage.hasFarSideRoom           = true;
    passage.secondaryRoomId          = farSideRoomId_in;
    passage.hasKnownSideDirection    = true;
    passage.knownSideDirection_World = knownSideDirection_in;
    evidence.context.passageContexts.push_back(passage);
    return evidence;
}

std::unique_ptr<SyntheticRoom> buildRoom(int                         roomId_in,
                                         const std::vector<RawWall> &walls_in,
                                         const Eigen::Vector3d &centroid_in)
{
    auto room = std::make_unique<SyntheticRoom>();
    if (room->room.setId(roomId_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (room->room.setCentroid(centroid_in) !=
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: setCentroid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    for (const RawWall &wall : walls_in)
    {
        room->addWall(wall);
    }
    return room;
}

double rotationAngle_deg(const Eigen::Matrix3d &first_in,
                         const Eigen::Matrix3d &second_in)
{
    const Eigen::AngleAxisd delta(first_in.transpose() * second_in);
    return delta.angle() * 180.0 / M_PI;
}

} // namespace

TEST(GeometricVerify, AcceptsGroundTruthCorrelatedRooms)
{
    const std::vector<RawWall> wallsA = makeReferenceWalls();
    const Eigen::Vector3d      centroidA(0.5, 0.5, 0.5);

    const Eigen::Matrix3d rotationTrue =
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.2, 0.6, 0.3).normalized())
            .toRotationMatrix();
    const Eigen::Vector3d translationTrue(4.0, -2.0, 1.5);

    std::vector<RawWall> wallsB;
    for (const RawWall &wall : wallsA)
    {
        wallsB.push_back(
            transformWall(wall, rotationTrue, translationTrue, wall.id + 100));
    }
    const Eigen::Vector3d centroidB =
        rotationTrue * centroidA + translationTrue;

    const std::unique_ptr<SyntheticRoom> roomA =
        buildRoom(1, wallsA, centroidA);
    const std::unique_ptr<SyntheticRoom> roomB =
        buildRoom(2, wallsB, centroidB);

    semantic::SemanticVerifyConfig               config;
    std::vector<semantic::VerifyWallObservation> observationsA{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomA->room,
                                                           config,
                                                           observationsA)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);
    std::vector<semantic::VerifyWallObservation> observationsB{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomB->room,
                                                           config,
                                                           observationsB)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);
    ASSERT_EQ(observationsA.size(), 4U);
    ASSERT_EQ(observationsB.size(), 4U);

    semantic::SemanticVerifyResult result{};
    ASSERT_EQ((semantic::SemanticVerify::verify(observationsA,
                                                observationsB,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);

    ASSERT_EQ(result.status, semantic::VerificationStatus::PASS);
    EXPECT_TRUE(result.hasPassed);
    EXPECT_GE(result.inliers.size(), 3U);
    EXPECT_LT(
        result.transform_AToB.translation().isApprox(translationTrue, 0.05)
            ? 0.0
            : (result.transform_AToB.translation() - translationTrue).norm(),
        0.05 + 1e-9);
    EXPECT_LT(rotationAngle_deg(result.transform_AToB.linear(), rotationTrue),
              0.5);
}

TEST(GeometricVerify, RejectsRankDeficientCorrespondences)
{
    /* Two parallel wall correspondences only: rotation about the shared
     * normal axis is unobservable. */
    const std::vector<RawWall> wallsA = {
        {1,
         Eigen::Vector3d(1.0, 0.0, 0.0),
         0.0,
         Eigen::Vector3d(0.05, 0.5, 0.5)},
        {2,
         Eigen::Vector3d(1.0, 0.0, 0.0),
         -2.0,
         Eigen::Vector3d(1.95, 0.5, 0.5)},
    };
    const Eigen::Matrix3d rotationTrue = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d translationTrue(1.0, 0.0, 0.0);
    std::vector<RawWall>  wallsB;
    for (const RawWall &wall : wallsA)
    {
        wallsB.push_back(
            transformWall(wall, rotationTrue, translationTrue, wall.id + 100));
    }

    const std::unique_ptr<SyntheticRoom> roomA =
        buildRoom(1, wallsA, Eigen::Vector3d(1.0, 0.5, 0.5));
    const std::unique_ptr<SyntheticRoom> roomB =
        buildRoom(2, wallsB, Eigen::Vector3d(2.0, 0.5, 0.5));

    semantic::SemanticVerifyConfig               config;
    std::vector<semantic::VerifyWallObservation> observationsA{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomA->room,
                                                           config,
                                                           observationsA)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);
    std::vector<semantic::VerifyWallObservation> observationsB{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomB->room,
                                                           config,
                                                           observationsB)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);

    semantic::SemanticVerifyResult result{};
    ASSERT_EQ((semantic::SemanticVerify::verify(observationsA,
                                                observationsB,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.status, semantic::VerificationStatus::REJECTED);
    EXPECT_FALSE(result.hasPassed);
    /* Only 2 walls per side -- never reaches hypothesis search. */
    EXPECT_EQ(result.rejectReason, semantic::VerifyRejectReason::TOO_FEW_WALLS);
}

TEST(GeometricVerify, CorrectHypothesisWinsOverOutlierCorrespondences)
{
    std::vector<RawWall>  wallsA = makeReferenceWalls();
    const Eigen::Vector3d centroidA(0.5, 0.5, 0.5);
    const Eigen::Matrix3d rotationTrue =
        Eigen::AngleAxisd(1.1, Eigen::Vector3d(0.1, 0.9, 0.2).normalized())
            .toRotationMatrix();
    const Eigen::Vector3d translationTrue(-3.0, 5.0, 0.5);

    std::vector<RawWall> wallsB;
    for (const RawWall &wall : wallsA)
    {
        wallsB.push_back(
            transformWall(wall, rotationTrue, translationTrue, wall.id + 100));
    }
    const Eigen::Vector3d centroidB =
        rotationTrue * centroidA + translationTrue;

    /* Add two extra, geometrically unrelated walls to each room -- plausible
     * candidate pairs that must NOT be selected as the winning hypothesis or
     * survive as inliers. */
    wallsA.push_back({50,
                      Eigen::Vector3d(0.0, 1.0, 1.0).normalized(),
                      -5.0,
                      Eigen::Vector3d(0.5, 3.0, 3.0)});
    wallsB.push_back({150,
                      Eigen::Vector3d(1.0, 0.0, -1.0).normalized(),
                      7.0,
                      Eigen::Vector3d(-8.0, 5.0, 4.0)});

    const std::unique_ptr<SyntheticRoom> roomA =
        buildRoom(1, wallsA, centroidA);
    const std::unique_ptr<SyntheticRoom> roomB =
        buildRoom(2, wallsB, centroidB);

    semantic::SemanticVerifyConfig config;
    config.minInlierRatio = 0.5; // 4 true inliers out of min(5,5)=5 walls
    std::vector<semantic::VerifyWallObservation> observationsA{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomA->room,
                                                           config,
                                                           observationsA)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);
    std::vector<semantic::VerifyWallObservation> observationsB{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomB->room,
                                                           config,
                                                           observationsB)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);

    semantic::SemanticVerifyResult result{};
    ASSERT_EQ((semantic::SemanticVerify::verify(observationsA,
                                                observationsB,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    ASSERT_EQ(result.status, semantic::VerificationStatus::PASS);
    EXPECT_EQ(result.rejectReason, semantic::VerifyRejectReason::NONE);
    EXPECT_EQ(result.inliers.size(), 4U);
    for (const semantic::WallInlierPair &inlier : result.inliers)
    {
        EXPECT_NE(inlier.wallIdA, 50);
        EXPECT_NE(inlier.wallIdB, 150);
    }
    EXPECT_LT(rotationAngle_deg(result.transform_AToB.linear(), rotationTrue),
              0.5);
}

TEST(GeometricVerify, RejectsAmbiguousEquallyGoodHypotheses)
{
    /* A cube's four vertical faces are consistent with two hypotheses 90
     * degrees apart (rotational symmetry) -- neither should be accepted
     * without discriminating evidence.
     *
     * The rejection reason this actually produces (confirmed once
     * semantic::SemanticVerifyResult::rejectReason existed to check, previously
     * unobservable since these diagnostic fields were only ever populated
     * on the PASS path) is NO_VALID_HYPOTHESIS, not
     * AMBIGUOUS_TOP_HYPOTHESES -- and for a more fundamental reason than
     * the ambiguity gate. This 4-wall room only has two DISTINCT normal
     * directions (each wall's opposite face is anti-parallel, not
     * independent): {1,2} both span +/-X, {3,4} both span +/-Y. Every
     * verify() hypothesis is seeded from a 3-DISTINCT-wall minimal sample,
     * and with only 4 walls to choose 3 from, every possible triple
     * necessarily contains one full opposing pair (pigeonhole) -- e.g.
     * {1,2,3} contains {1,2}. That collapses the minimal sample's own
     * translation-fit rank to 2, so fitTranslation()'s `rank < 3U` guard
     * (verify()'s per-hypothesis gate, not a downstream inlier check)
     * discards literally every hypothesis before any inlier counting ever
     * happens, leaving allHypotheses empty. This still proves the
     * fixture's intended point -- a room with only two independent wall
     * directions cannot be safely matched from wall geometry alone -- just
     * via an earlier, stricter gate than the test's name assumed: it can't
     * even form a candidate hypothesis, let alone an ambiguous one. */
    const std::vector<RawWall> wallsA = {
        {1,
         Eigen::Vector3d(1.0, 0.0, 0.0),
         0.0,
         Eigen::Vector3d(0.05, 0.5, 0.5)},
        {2,
         Eigen::Vector3d(-1.0, 0.0, 0.0),
         -2.0,
         Eigen::Vector3d(1.95, 0.5, 0.5)},
        {3,
         Eigen::Vector3d(0.0, 1.0, 0.0),
         0.0,
         Eigen::Vector3d(0.5, 0.05, 0.5)},
        {4,
         Eigen::Vector3d(0.0, -1.0, 0.0),
         -2.0,
         Eigen::Vector3d(0.5, 1.95, 0.5)},
    };
    const std::unique_ptr<SyntheticRoom> roomA =
        buildRoom(1, wallsA, Eigen::Vector3d(1.0, 1.0, 0.5));
    const std::unique_ptr<SyntheticRoom> roomB =
        buildRoom(2, wallsA, Eigen::Vector3d(1.0, 1.0, 0.5));

    semantic::SemanticVerifyConfig config;
    config.ambiguityMarginInliers = 1U;
    std::vector<semantic::VerifyWallObservation> observationsA{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomA->room,
                                                           config,
                                                           observationsA)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);
    std::vector<semantic::VerifyWallObservation> observationsB{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomB->room,
                                                           config,
                                                           observationsB)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);

    semantic::SemanticVerifyResult result{};
    ASSERT_EQ((semantic::SemanticVerify::verify(observationsA,
                                                observationsB,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_EQ(result.status, semantic::VerificationStatus::REJECTED);
    /* See the comment above: with only two independent wall directions and
     * exactly 4 walls, every 3-wall minimal sample necessarily contains one
     * full opposing (anti-parallel) pair, so no hypothesis is ever formed
     * at all -- confirmed via rejectReason, previously unobservable since
     * this field was only ever populated on the PASS path. */
    EXPECT_EQ(result.rejectReason,
              semantic::VerifyRejectReason::NO_VALID_HYPOTHESIS);
}

TEST(GeometricVerify, FloorGateAcceptsMatchingAndRejectsMismatchedFloors)
{
    Map survivingMap;
    Map absorbedMap;

    semantic::Floor survivingFloor;
    ASSERT_EQ((survivingFloor.setId(1)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_TRUE(
        (survivingFloor.setPlaneIdentity(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                                         100U,
                                         5U) ==
         vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
    survivingMap.addMapFloor(&survivingFloor);

    semantic::Floor matchingAbsorbedFloor;
    ASSERT_EQ((matchingAbsorbedFloor.setId(2)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_TRUE((matchingAbsorbedFloor.setPlaneIdentity(
                     Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                     100U,
                     5U) ==
                 vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
    absorbedMap.addMapFloor(&matchingAbsorbedFloor);

    const Eigen::Isometry3d        identity = Eigen::Isometry3d::Identity();
    semantic::SemanticVerifyResult acceptedResult;
    bool                           hasPassed{};
    ASSERT_EQ((semantic::SemanticVerify::runFloorGate(acceptedResult,
                                                      &survivingMap,
                                                      &absorbedMap,
                                                      identity,
                                                      hasPassed)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_TRUE(hasPassed);
    EXPECT_TRUE(acceptedResult.hasFloorGateRun);
    EXPECT_TRUE(acceptedResult.hasFloorGatePassed);
    EXPECT_EQ(acceptedResult.floorGateResult, "ACCEPTED");

    Map             mismatchedMap;
    semantic::Floor mismatchedFloor;
    ASSERT_EQ((mismatchedFloor.setId(3)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_TRUE(
        (mismatchedFloor.setPlaneIdentity(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                                          100U,
                                          5U) ==
         vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
    mismatchedMap.addMapFloor(&mismatchedFloor);

    semantic::SemanticVerifyResult rejectedResult;
    bool                           hasPassed2{};
    ASSERT_EQ((semantic::SemanticVerify::runFloorGate(rejectedResult,
                                                      &survivingMap,
                                                      &mismatchedMap,
                                                      identity,
                                                      hasPassed2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_FALSE(hasPassed2);
    EXPECT_TRUE(rejectedResult.hasFloorGateRun);
    EXPECT_FALSE(rejectedResult.hasFloorGatePassed);
    EXPECT_EQ(rejectedResult.floorGateResult, "REJECTED");
}

TEST(GeometricVerify, SemanticMergeGateAcceptsAlignedStableHierarchy)
{
    const semantic::SemanticMergeRoomEvidence surviving =
        makeMergeEvidence(2, 3, Eigen::Vector3d::UnitX());
    const semantic::SemanticMergeRoomEvidence absorbed =
        makeMergeEvidence(2, 3, Eigen::Vector3d::UnitX());

    semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((semantic::SemanticVerify::evaluateMergeAlignment({surviving},
                                                                {absorbed},
                                                                g2o::Sim3(),
                                                                result)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);

    EXPECT_EQ(result.decision, semantic::SemanticMergeDecision::ACCEPT);
    EXPECT_EQ(result.reason, semantic::SemanticMergeReason::ALIGNED);
    EXPECT_EQ(result.sharedRoomCount, 1U);
    EXPECT_EQ(result.alignedRoomCount, 1U);
    EXPECT_EQ(result.matchedWallCount, 4U);
    EXPECT_EQ(result.matchedPassageCount, 1U);
}

TEST(GeometricVerify,
     SemanticMergeGateRejectsIdenticalGeometryWithWrongPassagePredecessor)
{
    const semantic::SemanticMergeRoomEvidence currentRoom =
        makeMergeEvidence(2, 3, Eigen::Vector3d::UnitX());
    semantic::SemanticMergeRoomEvidence incorrectlyMatchedPriorRoom =
        makeMergeEvidence(2, 0, Eigen::Vector3d::UnitX());

    semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((semantic::SemanticVerify::evaluateMergeAlignment(
                  {currentRoom},
                  {incorrectlyMatchedPriorRoom},
                  g2o::Sim3(),
                  result)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);

    EXPECT_EQ(result.decision, semantic::SemanticMergeDecision::REJECT);
    EXPECT_EQ(result.reason,
              semantic::SemanticMergeReason::PASSAGE_ENDPOINT_CONTRADICTION);
    EXPECT_EQ(result.matchedWallCount, 4U);
}

TEST(GeometricVerify, SemanticMergeGateDefersWhenPassageEvidenceIsMissing)
{
    const semantic::SemanticMergeRoomEvidence surviving =
        makeMergeEvidence(2, 3, Eigen::Vector3d::UnitX());
    semantic::SemanticMergeRoomEvidence absorbed =
        makeMergeEvidence(2, 3, Eigen::Vector3d::UnitX());
    absorbed.context.passageContexts.clear();

    semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((semantic::SemanticVerify::evaluateMergeAlignment({surviving},
                                                                {absorbed},
                                                                g2o::Sim3(),
                                                                result)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);

    EXPECT_EQ(result.decision, semantic::SemanticMergeDecision::DEFER);
    EXPECT_EQ(result.reason,
              semantic::SemanticMergeReason::PASSAGE_EVIDENCE_MISSING);
}

TEST(GeometricVerify, SemanticMergeGateRejectsOpposedPassageDirection)
{
    const semantic::SemanticMergeRoomEvidence surviving =
        makeMergeEvidence(2, 3, Eigen::Vector3d::UnitX());
    const semantic::SemanticMergeRoomEvidence absorbed =
        makeMergeEvidence(2, 3, -Eigen::Vector3d::UnitX());

    semantic::SemanticMergeGateResult result{};
    ASSERT_EQ((semantic::SemanticVerify::evaluateMergeAlignment({surviving},
                                                                {absorbed},
                                                                g2o::Sim3(),
                                                                result)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);

    EXPECT_EQ(result.decision, semantic::SemanticMergeDecision::REJECT);
    EXPECT_EQ(result.reason,
              semantic::SemanticMergeReason::PASSAGE_DIRECTION_CONTRADICTION);
}

TEST(GeometricVerify, RoomReconciliationNeverFusesDifferentStableIdentities)
{
    Map            map;
    semantic::Room roomZero;
    semantic::Room roomTwo;
    ASSERT_EQ((roomZero.setId(0)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomZero.setRoomTag("room_0")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomZero.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomZero.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomTwo.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomTwo.setRoomTag("room_2")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomTwo.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((roomTwo.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    map.addDetectedMapRoom(&roomZero);
    map.addDetectedMapRoom(&roomTwo);

    ASSERT_EQ(
        (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(&map, {&roomTwo})),
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS);

    bool isBad2{};
    ASSERT_EQ((roomZero.isBad(isBad2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(isBad2);
    bool isBad3{};
    ASSERT_EQ((roomTwo.isBad(isBad3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(isBad3);
    EXPECT_EQ(map.getAllDetectedMapRooms().size(), 2U);
}

TEST(GeometricVerify, RoomReconciliationCollapsesMatchingStableIdentity)
{
    Map            map;
    semantic::Room retainedRoom;
    semantic::Room importedRoom;
    ASSERT_EQ((retainedRoom.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retainedRoom.setRoomTag("room_2")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retainedRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retainedRoom.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setRoomTag("room_2")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    map.addDetectedMapRoom(&retainedRoom);
    map.addDetectedMapRoom(&importedRoom);

    ASSERT_EQ(
        (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(&map,
                                                           {&importedRoom})),
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS);

    bool isBad2{};
    ASSERT_EQ((retainedRoom.isBad(isBad2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(isBad2);
    bool isBad3{};
    ASSERT_EQ((importedRoom.isBad(isBad3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(isBad3);
    EXPECT_EQ(map.getAllDetectedMapRooms().size(), 1U);
    EXPECT_EQ(map.getAllDetectedMapRooms().front(), &retainedRoom);
}

TEST(GeometricVerify, RoomReconciliationPreservesVisitedFlagOnFusion)
{
    /* Presence on either side proves the UAV has been there: a room fused
     * from a visited duplicate stays visited, and two unvisited rooms stay
     * unvisited. */
    Map            map;
    semantic::Room retainedRoom;
    semantic::Room importedRoom;
    ASSERT_EQ((retainedRoom.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retainedRoom.setRoomTag("room_2")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retainedRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retainedRoom.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setId(2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setRoomTag("room_2")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedRoom.setPreviouslyVisited(true)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    map.addDetectedMapRoom(&retainedRoom);
    map.addDetectedMapRoom(&importedRoom);

    ASSERT_EQ(
        (utils::utils::Utils::fuseDuplicateRoomsAfterMerge(&map,
                                                           {&importedRoom})),
        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS);

    bool isBad2{};
    ASSERT_EQ((importedRoom.isBad(isBad2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(isBad2);
    bool hasPreviouslyVisited2{};
    ASSERT_EQ((retainedRoom.hasPreviouslyVisited(hasPreviouslyVisited2)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(hasPreviouslyVisited2);

    semantic::Room retainedUnvisited;
    semantic::Room importedUnvisited;
    ASSERT_EQ((retainedUnvisited.setId(3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retainedUnvisited.setRoomTag("room_3")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (retainedUnvisited.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((retainedUnvisited.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedUnvisited.setId(3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedUnvisited.setRoomTag("room_3")),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ(
        (importedUnvisited.setRoomVariant(semantic::Room::RoomVariant::ROOM)),
        vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    ASSERT_EQ((importedUnvisited.setCentroid(Eigen::Vector3d::Zero())),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    map.addDetectedMapRoom(&retainedUnvisited);
    map.addDetectedMapRoom(&importedUnvisited);

    ASSERT_EQ((utils::utils::Utils::fuseDuplicateRoomsAfterMerge(
                  &map,
                  {&importedUnvisited})),
              utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS);

    bool isBad3{};
    ASSERT_EQ((importedUnvisited.isBad(isBad3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_TRUE(isBad3);
    bool hasPreviouslyVisited3{};
    ASSERT_EQ((retainedUnvisited.hasPreviouslyVisited(hasPreviouslyVisited3)),
              vs_graphs::core::semantic::RoomStatus::ROOM_STATUS_SUCCESS);
    EXPECT_FALSE(hasPreviouslyVisited3);
}

TEST(GeometricVerify, CombinedVerdictWiresGeometricAndFloorGate)
{
    /* toVerificationVerdict() ANDs the geometric pass with hasFloorGatePassed
     * -- this test exercises that combination through the real public API
     * (verify() -> runFloorGate() -> toVerificationVerdict()), not just each
     * half in isolation, since the wiring between them is exactly what a
     * caller depends on. */
    const std::vector<RawWall> wallsA = makeReferenceWalls();
    const Eigen::Vector3d      centroidA(0.5, 0.5, 0.5);

    /* Pure yaw (Z-axis) rotation + horizontal-only translation: preserves a
     * horizontal floor plane identity by construction (n'=Rn stays (0,0,1);
     * d'=d-n'^T t stays 0 since t has no Z component), so the same
     * ground-truth transform is valid input to both the wall verifier and
     * the floor gate. An arbitrary-axis rotation would legitimately tip a
     * horizontal floor and isn't representative of two maps of the same
     * building. */
    const Eigen::Matrix3d rotationTrue =
        Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d translationTrue(3.0, -1.0, 0.0);

    std::vector<RawWall> wallsB;
    for (const RawWall &wall : wallsA)
    {
        wallsB.push_back(
            transformWall(wall, rotationTrue, translationTrue, wall.id + 100));
    }
    const Eigen::Vector3d centroidB =
        rotationTrue * centroidA + translationTrue;

    const std::unique_ptr<SyntheticRoom> roomA =
        buildRoom(1, wallsA, centroidA);
    const std::unique_ptr<SyntheticRoom> roomB =
        buildRoom(2, wallsB, centroidB);

    semantic::SemanticVerifyConfig               config;
    std::vector<semantic::VerifyWallObservation> observationsA{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomA->room,
                                                           config,
                                                           observationsA)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);
    std::vector<semantic::VerifyWallObservation> observationsB{};
    ASSERT_EQ(
        (semantic::SemanticVerify::collectWallObservations(&roomB->room,
                                                           config,
                                                           observationsB)),
        vs_graphs::core::semantic::SemanticVerifyStatus::
            SEMANTIC_VERIFY_STATUS_SUCCESS);

    semantic::SemanticVerifyResult result{};
    ASSERT_EQ((semantic::SemanticVerify::verify(observationsA,
                                                observationsB,
                                                result,
                                                config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    ASSERT_EQ(result.status, semantic::VerificationStatus::PASS);
    ASSERT_TRUE(result.hasPassed);

    Map             survivingMap;
    Map             absorbedMap;
    semantic::Floor survivingFloor;
    ASSERT_EQ((survivingFloor.setId(1)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_TRUE(
        (survivingFloor.setPlaneIdentity(Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                                         100U,
                                         5U) ==
         vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
    survivingMap.addMapFloor(&survivingFloor);

    semantic::Floor matchingAbsorbedFloor;
    ASSERT_EQ((matchingAbsorbedFloor.setId(2)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_TRUE((matchingAbsorbedFloor.setPlaneIdentity(
                     Eigen::Vector4d(0.0, 0.0, 1.0, 0.0),
                     100U,
                     5U) ==
                 vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
    absorbedMap.addMapFloor(&matchingAbsorbedFloor);

    bool hasPassed2{};
    ASSERT_EQ((semantic::SemanticVerify::runFloorGate(result,
                                                      &survivingMap,
                                                      &absorbedMap,
                                                      result.transform_AToB,
                                                      hasPassed2)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_TRUE(hasPassed2);
    EXPECT_TRUE(result.hasFloorGateRun);
    EXPECT_TRUE(result.hasFloorGatePassed);
    EXPECT_EQ(result.floorGateResult, "ACCEPTED");

    /* This is the exact combination that was silently broken: a genuine
     * geometric pass plus a genuine floor-gate accept must yield an
     * accepted combined verdict. */
    semantic::VerificationVerdict verificationVerdict{};
    ASSERT_EQ((result.toVerificationVerdict(verificationVerdict)),
              semantic::SemanticVerifyResultStatus::
                  SEMANTIC_VERIFY_RESULT_STATUS_SUCCESS);
    EXPECT_TRUE(verificationVerdict.hasPassed);

    /* Same genuine geometric pass, but the floor gate rejects -- the
     * combined verdict must follow the floor gate down. */
    Map             mismatchedMap;
    semantic::Floor mismatchedFloor;
    ASSERT_EQ((mismatchedFloor.setId(3)),
              vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS);
    ASSERT_TRUE(
        (mismatchedFloor.setPlaneIdentity(Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
                                          100U,
                                          5U) ==
         vs_graphs::core::semantic::FloorStatus::FLOOR_STATUS_SUCCESS));
    mismatchedMap.addMapFloor(&mismatchedFloor);

    semantic::SemanticVerifyResult mismatchedResult = result;
    bool                           hasPassed3{};
    ASSERT_EQ((semantic::SemanticVerify::runFloorGate(mismatchedResult,
                                                      &survivingMap,
                                                      &mismatchedMap,
                                                      result.transform_AToB,
                                                      hasPassed3)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);
    EXPECT_FALSE(hasPassed3);
    EXPECT_EQ(mismatchedResult.floorGateResult, "REJECTED");
    semantic::VerificationVerdict verificationVerdict2{};
    ASSERT_EQ((mismatchedResult.toVerificationVerdict(verificationVerdict2)),
              semantic::SemanticVerifyResultStatus::
                  SEMANTIC_VERIFY_RESULT_STATUS_SUCCESS);
    EXPECT_FALSE(verificationVerdict2.hasPassed);
}

TEST(GeometricVerify, ConfigFromSystemParamsWiresLoadedYamlValues)
{
    /* Previously, semantic::SemanticVerifyConfig's own
     * default-member-initialisers happened to literally match
     * types::SystemParams::Verification/Factor's defaults, but nothing ever
     * read the loaded types::SystemParams values into a
     * semantic::SemanticVerifyConfig -- an operator's system_params.yaml edit
     * had zero effect on the verifier. This exercises configFromSystemParams()
     * threading distinctive, non-default loaded values through, proving the
     * wiring actually exists now. */
    types::SystemParams *params = nullptr;
    ASSERT_EQ((types::SystemParams::getParams(params)),
              types::SystemParamsStatus::SYSTEM_PARAMS_STATUS_SUCCESS);
    const auto savedVerification = params->verification;
    const auto savedFactor       = params->factor;

    params->verification.maxNormalAngle_deg      = 17.5F;
    params->verification.maxOffset_m             = 0.42F;
    params->verification.maxSupportDist_m        = 0.31F;
    params->verification.minInlierRatio          = 0.7F;
    params->verification.maxConditionNumber      = 55.0F;
    params->verification.ambiguityMarginInliers  = 2U;
    params->verification.maxWallsPerRoom         = 12U;
    params->verification.maxHypotheses           = 500U;
    params->verification.maxSupportSamplePerWall = 32U;
    params->verification.minAbsCosNormalAngle    = 0.7F;
    params->factor.sigmaTheta_rad                = 0.11F;
    params->factor.sigmaOffset_m                 = 0.09F;
    params->factor.huberDelta                    = 2.0F;
    params->factor.optimizerIterations           = 7U;

    semantic::SemanticVerifyConfig config{};
    ASSERT_EQ((semantic::SemanticVerify::configFromSystemParams(config)),
              vs_graphs::core::semantic::SemanticVerifyStatus::
                  SEMANTIC_VERIFY_STATUS_SUCCESS);

    EXPECT_NEAR(config.maxNormalAngle_deg, 17.5, 1e-5);
    EXPECT_NEAR(config.maxOffset_m, 0.42, 1e-5);
    EXPECT_NEAR(config.maxSupportDist_m, 0.31, 1e-5);
    EXPECT_NEAR(config.minInlierRatio, 0.7, 1e-5);
    EXPECT_NEAR(config.maxConditionNumber, 55.0, 1e-5);
    EXPECT_EQ(config.ambiguityMarginInliers, 2U);
    EXPECT_EQ(config.maxWallsPerRoom, 12U);
    EXPECT_EQ(config.maxHypotheses, 500U);
    EXPECT_EQ(config.maxSupportSamplePerWall, 32U);
    EXPECT_NEAR(config.minAbsCosNormalAngle, 0.7, 1e-5);
    EXPECT_NEAR(config.sigmaTheta_rad, 0.11, 1e-5);
    EXPECT_NEAR(config.sigmaOffset_m, 0.09, 1e-5);
    EXPECT_NEAR(config.huberDelta, 2.0, 1e-5);
    EXPECT_EQ(config.optimizerIterations, 7U);

    params->verification = savedVerification;
    params->factor       = savedFactor;
}

TEST(GeometricVerify, SeededUncorrelatedRoomsProduceNoFalseAccepts)
{
    semantic::SemanticVerifyConfig         config;
    std::mt19937                           generator(1401U);
    std::uniform_real_distribution<double> directionSpread(-4.0, 4.0);
    /* Offsets span a range representative of a real building's coordinate
     * extent (tens of metres), not a tiny unit cube -- otherwise the
     * verifier's 0.35 m offset gate is wide enough, relative to the spread,
     * that some of the 96 tried hypotheses spuriously align 3/4 random
     * pairs by pure chance. This is a property of the test's synthetic
     * scale, not the verifier's (plan-specified) gate thresholds. */
    std::uniform_real_distribution<double> offsetSpread(-60.0, 60.0);

    unsigned int           falseAccepts = 0U;
    constexpr unsigned int kIterations  = 30U;
    for (unsigned int iteration = 0U; iteration < kIterations; ++iteration)
    {
        std::vector<RawWall> wallsA;
        std::vector<RawWall> wallsB;
        for (int index = 0; index < 4; ++index)
        {
            const Eigen::Vector3d normalA =
                Eigen::Vector3d(directionSpread(generator),
                                directionSpread(generator),
                                directionSpread(generator))
                    .normalized();
            wallsA.push_back({index + 1,
                              normalA,
                              offsetSpread(generator),
                              Eigen::Vector3d::Zero()});

            const Eigen::Vector3d normalB =
                Eigen::Vector3d(directionSpread(generator),
                                directionSpread(generator),
                                directionSpread(generator))
                    .normalized();
            wallsB.push_back({index + 101,
                              normalB,
                              offsetSpread(generator),
                              Eigen::Vector3d::Zero()});
        }

        const std::unique_ptr<SyntheticRoom> roomA =
            buildRoom(1, wallsA, Eigen::Vector3d(0.0, 0.0, 0.0));
        const std::unique_ptr<SyntheticRoom> roomB =
            buildRoom(2, wallsB, Eigen::Vector3d(0.0, 0.0, 0.0));

        std::vector<semantic::VerifyWallObservation> observationsA{};
        ASSERT_EQ(
            (semantic::SemanticVerify::collectWallObservations(&roomA->room,
                                                               config,
                                                               observationsA)),
            vs_graphs::core::semantic::SemanticVerifyStatus::
                SEMANTIC_VERIFY_STATUS_SUCCESS);
        std::vector<semantic::VerifyWallObservation> observationsB{};
        ASSERT_EQ(
            (semantic::SemanticVerify::collectWallObservations(&roomB->room,
                                                               config,
                                                               observationsB)),
            vs_graphs::core::semantic::SemanticVerifyStatus::
                SEMANTIC_VERIFY_STATUS_SUCCESS);

        semantic::SemanticVerifyResult result{};
        ASSERT_EQ((semantic::SemanticVerify::verify(observationsA,
                                                    observationsB,
                                                    result,
                                                    config)),
                  vs_graphs::core::semantic::SemanticVerifyStatus::
                      SEMANTIC_VERIFY_STATUS_SUCCESS);
        if (result.status == semantic::VerificationStatus::PASS)
        {
            ++falseAccepts;
        }
    }
    EXPECT_EQ(falseAccepts, 0U);
}

TEST(GeometricVerify,
     PlaneTransformEdgeResidualIsZeroAtGroundTruthAndNonzeroNearby)
{
    const Eigen::Matrix3d rotationTrue =
        Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.0, 0.0, 1.0))
            .toRotationMatrix();
    const Eigen::Vector3d translationTrue(1.0, 2.0, 3.0);

    PlanePairMeasurement measurement;
    measurement.n_A   = Eigen::Vector3d(1.0, 0.0, 0.0);
    measurement.d_A   = -0.5;
    measurement.sigma = 1;
    measurement.n_B   = rotationTrue * measurement.n_A;
    measurement.d_B   = measurement.d_A - measurement.n_B.dot(translationTrue);

    g2o::VertexSE3Expmap vertex;
    vertex.setEstimate(g2o::SE3Quat(rotationTrue, translationTrue));
    vertex.setId(0);

    EdgePlaneTransformSE3 edge;
    edge.setVertex(0, &vertex);
    edge.setMeasurement(measurement);
    edge.computeError();
    EXPECT_NEAR(edge.error()(0), 0.0, 1e-9);
    EXPECT_NEAR(edge.error()(1), 0.0, 1e-9);
    EXPECT_NEAR(edge.error()(2), 0.0, 1e-9);

    /* Perturb translation away from ground truth: the offset residual must
     * become clearly nonzero. */
    g2o::VertexSE3Expmap perturbedVertex;
    perturbedVertex.setEstimate(
        g2o::SE3Quat(rotationTrue,
                     translationTrue + Eigen::Vector3d(0.2, 0.0, 0.0)));
    perturbedVertex.setId(0);
    EdgePlaneTransformSE3 perturbedEdge;
    perturbedEdge.setVertex(0, &perturbedVertex);
    perturbedEdge.setMeasurement(measurement);
    perturbedEdge.computeError();
    EXPECT_GT(std::abs(perturbedEdge.error()(2)), 0.05);
}

TEST(GeometricVerify,
     PlaneTransformEdgeResidualIsNonzeroAtAntipodalMisalignment)
{
    /* At a 180 degree normal misalignment, axis = n_pred.cross(n_B)
     * underflows to zero exactly like the true-zero-error case at 0 degrees
     * does. Before the antipodal branch in computeError(), both cases fell
     * through the same `sinAngle > 1e-12` guard and reported a zero
     * orientation residual, silently masking a maximal misalignment as a
     * perfect fit. */
    PlanePairMeasurement measurement;
    measurement.n_A   = Eigen::Vector3d(1.0, 0.0, 0.0);
    measurement.d_A   = 0.0;
    measurement.sigma = 1;
    measurement.n_B   = Eigen::Vector3d(-1.0, 0.0, 0.0);
    measurement.d_B   = 0.0;

    g2o::VertexSE3Expmap vertex;
    vertex.setEstimate(
        g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
    vertex.setId(0);

    EdgePlaneTransformSE3 edge;
    edge.setVertex(0, &vertex);
    edge.setMeasurement(measurement);
    edge.computeError();

    /* axisU/axisV (built from n_B) and the antipodal logVec (built
     * orthogonal to n_pred, which is collinear with n_B here) all lie in the
     * same plane, so the residual's magnitude is exactly pi regardless of
     * which specific in-plane direction the antipodal branch picks. */
    const double orientationResidualNorm =
        Eigen::Vector2d(edge.error()(0), edge.error()(1)).norm();
    EXPECT_NEAR(orientationResidualNorm, M_PI, 1e-6);
}

} // namespace core
} // namespace vs_graphs
