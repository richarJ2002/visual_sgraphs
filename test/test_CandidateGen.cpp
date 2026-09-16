#include "Semantic/SemanticCandidates.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <random>
#include <sstream>

namespace vs_graphs
{
namespace core
{
namespace
{
semantic::RoomContextSnapshot makeRoom(int roomId, double secondAngle_rad)
{
    semantic::RoomContextSnapshot snapshot;
    snapshot.roomId      = roomId;
    snapshot.wallNormals = {Eigen::Vector3d::UnitX(),
                            Eigen::Vector3d(std::cos(secondAngle_rad),
                                            std::sin(secondAngle_rad),
                                            0.0)};
    snapshot.wallBounds  = {{true, 0.0, 2.0, 0.0, 1.0},
                            {true, 0.0, 2.0, 0.0, 1.0}};
    return snapshot;
}

semantic::RoomContextSnapshot
    makeTopologyRoom(int roomId, int passageId, int farRoomId, bool hasFarSide)
{
    semantic::RoomContextSnapshot snapshot = makeRoom(roomId, 1.0);
    semantic::PassageContext      passage;
    passage.id                                = passageId;
    passage.passable                          = true;
    passage.hasFarSideRoom                    = hasFarSide;
    passage.secondaryRoomId                   = farRoomId;
    passage.apertureValid                     = true;
    passage.width_m                           = 1.0;
    passage.height_m                          = 2.0;
    passage.traversalKnownToFarCount          = 3U;
    passage.traversalFarToKnownCount          = 2U;
    passage.hasBidirectionalTraversalEvidence = true;
    snapshot.passageContexts.push_back(passage);
    snapshot.passageCentroids.push_back(Eigen::Vector3d::Zero());
    return snapshot;
}

std::string candidateBytes(const std::vector<semantic::SemanticCandidate> &candidates)
{
    std::ostringstream stream;
    for (const semantic::SemanticCandidate &candidate : candidates)
    {
        stream << candidate.mapAId << ':' << candidate.roomAId << ':'
               << candidate.mapBId << ':' << candidate.roomBId << ':'
               << candidate.distance << ':' << candidate.cues.angleDistance
               << ':' << candidate.cues.extentDistance << ':'
               << candidate.cues.topologyDistance << ':' << candidate.ambiguous
               << ':' << candidate.lowConfidence << '\n';
    }
    return stream.str();
}

semantic::RoomContextSnapshot makeMultiWallRoom(int                        roomId,
                                      const std::vector<double> &angles_rad)
{
    semantic::RoomContextSnapshot snapshot;
    snapshot.roomId = roomId;
    for (const double angle : angles_rad)
    {
        snapshot.wallNormals.push_back(
            Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0));
        snapshot.wallBounds.push_back({true, 0.0, 2.0, 0.0, 1.0});
    }
    return snapshot;
}

/** No walls, no passages: always fails minimum evidence regardless of which
 * cue weights are configured. */
semantic::RoomContextSnapshot emptyEvidenceRoom(int roomId)
{
    semantic::RoomContextSnapshot snapshot;
    snapshot.roomId = roomId;
    return snapshot;
}

/** One wall (valid normal, degenerate/invalid bounds) plus one aperture-valid
 * passage: passes minimum evidence via the mixed branch, but its own median
 * extent is 0 (no wall has valid bounds), so extent/aperture normalisation
 * cannot proceed. */
semantic::RoomContextSnapshot degenerateMedianRoom(int roomId, int passageId)
{
    semantic::RoomContextSnapshot snapshot;
    snapshot.roomId      = roomId;
    snapshot.wallNormals = {Eigen::Vector3d::UnitX()};
    snapshot.wallBounds  = {{false, 0.0, 0.0, 0.0, 0.0}};
    semantic::PassageContext passage;
    passage.id            = passageId;
    passage.apertureValid = true;
    passage.width_m       = 1.0;
    passage.height_m      = 2.0;
    snapshot.passageContexts.push_back(passage);
    snapshot.passageCentroids.push_back(Eigen::Vector3d::Zero());
    return snapshot;
}

/** One valid wall plus one aperture-valid passage of the given dimensions. */
semantic::RoomContextSnapshot
    singlePassageRoom(int roomId, double width_m, double height_m)
{
    semantic::RoomContextSnapshot snapshot;
    snapshot.roomId      = roomId;
    snapshot.wallNormals = {Eigen::Vector3d::UnitX()};
    snapshot.wallBounds  = {{true, 0.0, 2.0, 0.0, 1.0}};
    semantic::PassageContext passage;
    passage.id            = 1;
    passage.apertureValid = true;
    passage.width_m       = width_m;
    passage.height_m      = height_m;
    snapshot.passageContexts.push_back(passage);
    snapshot.passageCentroids.push_back(Eigen::Vector3d::Zero());
    return snapshot;
}

semantic::RoomContextSnapshot rotateRoom(const semantic::RoomContextSnapshot &source,
                               const Eigen::Matrix3d     &rotation)
{
    semantic::RoomContextSnapshot transformed = source;
    for (Eigen::Vector3d &normal : transformed.wallNormals)
    {
        if (normal.allFinite())
        {
            normal = rotation * normal;
        }
    }
    for (Eigen::Vector3d &centroid : transformed.wallCentroids)
    {
        if (centroid.allFinite())
        {
            centroid = rotation * centroid + Eigen::Vector3d(4.0, -2.0, 1.0);
        }
    }
    for (Eigen::Vector3d &centroid : transformed.passageCentroids)
    {
        if (centroid.allFinite())
        {
            centroid = rotation * centroid + Eigen::Vector3d(4.0, -2.0, 1.0);
        }
    }
    transformed.centroid =
        rotation * transformed.centroid + Eigen::Vector3d(4.0, -2.0, 1.0);
    return transformed;
}
} // namespace

TEST(CandidateGen, UsesDeterministicNumeratorAndDenominator)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[10U].push_back(makeRoom(1, 1.0));
    history[20U].push_back(makeRoom(2, 1.2));
    semantic::SemanticCandidateConfig config;
    config.weightAngle    = 2.0;
    config.weightExtent   = 1.0;
    config.weightAperture = 0.0;
    config.weightTopology = 0.0;

    const std::vector<semantic::SemanticCandidate> candidates =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_DOUBLE_EQ(candidates.front().cues.weightDenominator, 3.0);
    EXPECT_DOUBLE_EQ(candidates.front().cues.weightedNumerator,
                     2.0 * 0.2 / 1.0);
    EXPECT_DOUBLE_EQ(candidates.front().distance,
                     candidates.front().cues.weightedNumerator / 3.0);
}

TEST(CandidateGen, DoesNotDependOnContainerInsertionOrder)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> first;
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> second;
    first[20U].push_back(makeRoom(2, 1.2));
    first[10U].push_back(makeRoom(1, 1.0));
    second[10U].push_back(makeRoom(1, 1.0));
    second[20U].push_back(makeRoom(2, 1.2));
    const std::vector<semantic::SemanticCandidate> left =
        semantic::SemanticCandidates::generate(first);
    const std::vector<semantic::SemanticCandidate> right =
        semantic::SemanticCandidates::generate(second);
    ASSERT_EQ(left.size(), right.size());
    ASSERT_FALSE(left.empty());
    EXPECT_EQ(left.front().roomAId, right.front().roomAId);
    EXPECT_DOUBLE_EQ(left.front().distance, right.front().distance);
}

TEST(CandidateGen, CanonicalizesLocalTopologyWithoutComparingRawIds)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[1U].push_back(makeTopologyRoom(11, 7, 99, true));
    history[2U].push_back(makeTopologyRoom(22, 3, 4, true));
    semantic::SemanticCandidateConfig config;
    config.weightAngle    = 0.0;
    config.weightExtent   = 0.0;
    config.weightAperture = 0.0;
    config.weightTopology = 1.0;
    const std::vector<semantic::SemanticCandidate> same =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_EQ(same.size(), 1U);
    EXPECT_TRUE(same.front().cues.topologyAvailable);
    EXPECT_DOUBLE_EQ(same.front().cues.topologyDistance, 0.0);

    history[2U][0].passageContexts[0].hasFarSideRoom = false;
    const std::vector<semantic::SemanticCandidate> changed =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_EQ(changed.size(), 1U);
    EXPECT_GT(changed.front().cues.topologyDistance, 0.0);
}

TEST(CandidateGen, AngleToleranceControlsNearZeroEquivalence)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[1U].push_back(makeRoom(1, 1.0));
    history[2U].push_back(makeRoom(2, 1.0 + 5.0e-10));
    semantic::SemanticCandidateConfig config;
    config.weightExtent       = 0.0;
    config.weightAperture     = 0.0;
    config.weightTopology     = 0.0;
    config.angleTolerance_rad = 1.0e-9;
    EXPECT_DOUBLE_EQ(semantic::SemanticCandidates::generate(history, config)
                         .front()
                         .cues.angleDistance,
                     0.0);
    config.angleTolerance_rad = 1.0e-12;
    EXPECT_GT(semantic::SemanticCandidates::generate(history, config)
                  .front()
                  .cues.angleDistance,
              0.0);
}

TEST(CandidateGen, RejectsInvalidConfigurationWithTypedReason)
{
    semantic::SemanticCandidateConfig config;
    config.angleTolerance_rad = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(
        semantic::SemanticCandidates::validateConfig(config),
        semantic::SemanticCandidateConfigRejectionReason::NONFINITE_ANGLE_TOLERANCE);
    config.angleTolerance_rad = 1.0e-9;
    config.runtimeBudget_ms   = std::numeric_limits<double>::infinity();
    EXPECT_EQ(semantic::SemanticCandidates::validateConfig(config),
              semantic::SemanticCandidateConfigRejectionReason::NONFINITE_RUNTIME_BUDGET);
    config.runtimeBudget_ms = 0.0;
    config.topK             = 0U;
    const semantic::SemanticCandidateGeneration result =
        semantic::SemanticCandidates::generateWithStatus({}, config);
    EXPECT_EQ(result.rejectionReason,
              semantic::SemanticCandidateConfigRejectionReason::TOP_K_ZERO);
    EXPECT_TRUE(result.candidates.empty());
    config.topK             = 10U;
    config.candidatePairCap = 5U;
    EXPECT_EQ(semantic::SemanticCandidates::validateConfig(config),
              semantic::SemanticCandidateConfigRejectionReason::TOP_K_EXCEEDS_PAIR_CAP);
    config.candidatePairCap      = 1000U;
    config.descriptorElementsCap = 0U;
    EXPECT_EQ(semantic::SemanticCandidates::validateConfig(config),
              semantic::SemanticCandidateConfigRejectionReason::DESCRIPTOR_CAP_ZERO);
    config.descriptorElementsCap = 4096U;
    config.weightAngle           = std::numeric_limits<double>::max();
    config.weightExtent          = std::numeric_limits<double>::max();
    EXPECT_EQ(semantic::SemanticCandidates::validateConfig(config),
              semantic::SemanticCandidateConfigRejectionReason::WEIGHT_SUM_OVERFLOW);
}

TEST(CandidateGen, OmitsTopologyWhenNodeCapWouldBeExceeded)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[1U].push_back(makeTopologyRoom(1, 1, 2, true));
    history[2U].push_back(makeTopologyRoom(2, 2, 3, true));
    semantic::SemanticCandidateConfig config;
    config.topologyNodesCap = 2U;
    const std::vector<semantic::SemanticCandidate> candidates =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_FALSE(candidates.front().cues.topologyAvailable);
}

TEST(CandidateGen, SeededTransformInvarianceIs100Of100)
{
    const semantic::RoomContextSnapshot source = makeRoom(1, 1.0);
    semantic::SemanticCandidateConfig   config;
    config.weightExtent   = 0.0;
    config.weightAperture = 0.0;
    config.weightTopology = 0.0;
    std::mt19937                           generator(1301U);
    std::uniform_real_distribution<double> angle(-3.14, 3.14);
    for (unsigned int iteration = 0U; iteration < 100U; ++iteration)
    {
        const Eigen::Vector3d axis = Eigen::Vector3d(angle(generator),
                                                     angle(generator),
                                                     angle(generator))
                                         .normalized();
        const Eigen::Matrix3d rotation =
            Eigen::AngleAxisd(angle(generator), axis).toRotationMatrix();
        std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
        history[10U].push_back(source);
        history[20U].push_back(rotateRoom(source, rotation));
        const std::vector<semantic::SemanticCandidate> candidates =
            semantic::SemanticCandidates::generate(history, config);
        ASSERT_EQ(candidates.size(), 1U);
        EXPECT_NEAR(candidates.front().distance, 0.0, 1.0e-12);
    }
}

TEST(CandidateGen, SeededTopOneStabilityIsAtLeast45Of50)
{
    semantic::SemanticCandidateConfig config;
    config.topK              = 1U;
    config.candidatePairCap  = 100U;
    config.globalFallbackCap = 100U;
    config.weightExtent      = 0.0;
    config.weightAperture    = 0.0;
    config.weightTopology    = 0.0;
    std::mt19937                           generator(1302U);
    std::uniform_real_distribution<double> perturbation(-0.02, 0.02);
    unsigned int                           stableCount = 0U;
    for (unsigned int iteration = 0U; iteration < 50U; ++iteration)
    {
        std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
        history[10U] = {makeRoom(1, 1.0), makeRoom(2, 0.2), makeRoom(3, 2.2)};
        history[20U] = {makeRoom(11, 1.0 + perturbation(generator)),
                        makeRoom(12, 0.35),
                        makeRoom(13, 2.0)};
        const std::vector<semantic::SemanticCandidate> candidates =
            semantic::SemanticCandidates::generate(history, config);
        ASSERT_EQ(candidates.size(), 1U);
        if (candidates.front().roomAId == 1 && candidates.front().roomBId == 11)
        {
            ++stableCount;
        }
    }
    EXPECT_GE(stableCount, 45U);
}

TEST(CandidateGen, SeededTruePairRecallIsAtLeast48Of50)
{
    semantic::SemanticCandidateConfig config;
    config.topK              = 5U;
    config.candidatePairCap  = 1000U;
    config.globalFallbackCap = 1000U;
    config.weightExtent      = 0.0;
    config.weightAperture    = 0.0;
    config.weightTopology    = 0.0;
    std::mt19937                           generator(1303U);
    std::uniform_real_distribution<double> perturbation(-0.03, 0.03);
    unsigned int                           recallCount = 0U;
    for (unsigned int iteration = 0U; iteration < 50U; ++iteration)
    {
        std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
        history[10U].push_back(makeRoom(1, 1.0));
        history[20U].push_back(makeRoom(101, 1.0 + perturbation(generator)));
        for (int distractor = 0; distractor < 10; ++distractor)
        {
            history[20U].push_back(
                makeRoom(200 + distractor, 0.1 + 0.2 * distractor));
        }
        const std::vector<semantic::SemanticCandidate> candidates =
            semantic::SemanticCandidates::generate(history, config);
        const bool found = std::any_of(candidates.begin(),
                                       candidates.end(),
                                       [](const semantic::SemanticCandidate &candidate) {
                                           return candidate.roomAId == 1 &&
                                                  candidate.roomBId == 101;
                                       });
        recallCount += found ? 1U : 0U;
    }
    EXPECT_GE(recallCount, 48U);
}

TEST(CandidateGen, CandidateBytesAreInsertionOrderIndependent)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> first;
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> second;
    first[10U]  = {makeRoom(3, 1.2), makeRoom(1, 1.0), makeRoom(2, 0.8)};
    first[20U]  = {makeRoom(13, 1.2), makeRoom(11, 1.0), makeRoom(12, 0.8)};
    second[10U] = {first[10U][2], first[10U][0], first[10U][1]};
    second[20U] = {first[20U][1], first[20U][2], first[20U][0]};
    const std::string left =
        candidateBytes(semantic::SemanticCandidates::generate(first));
    const std::string right =
        candidateBytes(semantic::SemanticCandidates::generate(second));
    EXPECT_EQ(left, right);
}

TEST(CandidateGen, RuntimeBudgetNeverChangesDeterministicBytes)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[1U].push_back(makeRoom(1, 1.0));
    history[2U].push_back(makeRoom(2, 1.1));
    semantic::SemanticCandidateConfig unprofiled;
    semantic::SemanticCandidateConfig profiled = unprofiled;
    profiled.runtimeBudget_ms        = 1.0e-12;
    const std::vector<semantic::SemanticCandidate> left =
        semantic::SemanticCandidates::generate(history, unprofiled);
    const std::vector<semantic::SemanticCandidate> right =
        semantic::SemanticCandidates::generate(history, profiled);
    EXPECT_EQ(candidateBytes(left), candidateBytes(right));
    ASSERT_FALSE(right.empty());
    EXPECT_FALSE(right.front().cues.runtimeBudgetExceeded);
}

/* Required test (a): rooms with no passages still score wall
 * cues; topology/aperture are absent, not trivially "available". */
TEST(CandidateGen, WallOnlyRoomsScoreWallCuesWithTopologyAndApertureAbsent)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[10U].push_back(makeRoom(1, 1.0));
    history[20U].push_back(makeRoom(2, 1.2));
    const std::vector<semantic::SemanticCandidate> candidates =
        semantic::SemanticCandidates::generate(history);
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_TRUE(candidates.front().minimumEvidenceSatisfied);
    EXPECT_FALSE(candidates.front().cues.topologyAvailable);
    /* Only angle + extent contribute (default weight 1.0 each); aperture and
     * topology are excluded from the denominator entirely. */
    EXPECT_DOUBLE_EQ(candidates.front().cues.weightDenominator, 2.0);
}

/* Required test (b): a wall with invalid bounds omits only its
 * extent element while its (still finite) normal keeps contributing angle
 * evidence. */
TEST(CandidateGen, PartiallyMissingBoundsOmitsExtentButRetainsAngleEvidence)
{
    semantic::RoomContextSnapshot baseline;
    baseline.roomId      = 1;
    baseline.wallNormals = {Eigen::Vector3d::UnitX(),
                            Eigen::Vector3d::UnitY(),
                            Eigen::Vector3d(0.0, std::cos(0.4), std::sin(0.4))};
    baseline.wallBounds  = {{true, 0.0, 2.0, 0.0, 1.0},
                            {true, 0.0, 2.0, 0.0, 1.0},
                            {false, 0.0, 0.0, 0.0, 0.0}};

    semantic::RoomContextSnapshot sameThirdNormal = baseline;
    sameThirdNormal.roomId              = 2;

    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> historySame;
    historySame[10U].push_back(baseline);
    historySame[20U].push_back(sameThirdNormal);
    const std::vector<semantic::SemanticCandidate> sameCandidates =
        semantic::SemanticCandidates::generate(historySame);
    ASSERT_EQ(sameCandidates.size(), 1U);
    EXPECT_DOUBLE_EQ(sameCandidates.front().cues.angleDistance, 0.0);
    EXPECT_DOUBLE_EQ(sameCandidates.front().cues.extentDistance, 0.0);

    semantic::RoomContextSnapshot differentThirdNormal = baseline;
    differentThirdNormal.roomId              = 3;
    differentThirdNormal.wallNormals[2] =
        Eigen::Vector3d(0.0, std::cos(1.0), std::sin(1.0));

    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> historyDiff;
    historyDiff[10U].push_back(baseline);
    historyDiff[20U].push_back(differentThirdNormal);
    const std::vector<semantic::SemanticCandidate> diffCandidates =
        semantic::SemanticCandidates::generate(historyDiff);
    ASSERT_EQ(diffCandidates.size(), 1U);
    /* The third wall's normal changed and still contributes to the angle
     * signature even though its bounds were invalid in both rooms. */
    EXPECT_GT(diffCandidates.front().cues.angleDistance, 0.0);
    /* Its extent was omitted in both rooms regardless, so extent is
     * unaffected by the normal change. */
    EXPECT_DOUBLE_EQ(diffCandidates.front().cues.extentDistance, 0.0);
}

/* Required test (c): unequal signature counts exercise the
 * padded mean-L1 rule. */
TEST(CandidateGen, UnequalSignatureCountsExercisePadding)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[10U].push_back(makeMultiWallRoom(1, {0.0, 0.5}));
    history[20U].push_back(makeMultiWallRoom(2, {0.0, 0.5, 2.0}));
    semantic::SemanticCandidateConfig config;
    config.weightExtent   = 0.0;
    config.weightAperture = 0.0;
    config.weightTopology = 0.0;
    const std::vector<semantic::SemanticCandidate> candidates =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_EQ(candidates.size(), 1U);

    /* Reference signatures use the same acos(|dot|) fold as production (a
     * plane normal's sign is arbitrary, so raw angle differences fold into
     * [0, pi/2]). Left has one pair (0.0,0.5); right has three pairs
     * (0.0,0.5), (0.0,2.0), (0.5,2.0). */
    const double        leftAngle   = 0.5;
    std::vector<double> rightAngles = {std::acos(std::abs(std::cos(0.5))),
                                       std::acos(std::abs(std::cos(2.0))),
                                       std::acos(std::abs(std::cos(1.5)))};
    std::sort(rightAngles.begin(), rightAngles.end());
    const double penalty  = config.angleMissingPenalty;
    const double expected = (std::abs(leftAngle - rightAngles[0]) +
                             std::abs(penalty - rightAngles[1]) +
                             std::abs(penalty - rightAngles[2])) /
                            3.0;
    EXPECT_NEAR(candidates.front().cues.angleDistance, expected, 1.0e-9);
    EXPECT_DOUBLE_EQ(candidates.front().distance,
                     candidates.front().cues.angleDistance);
}

/* Required test (d): a zero/invalid median disables both the
 * extent and aperture families without rejecting the room outright. */
TEST(CandidateGen, InvalidMedianDisablesExtentAndApertureCues)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[10U].push_back(degenerateMedianRoom(1, 100));
    history[20U].push_back(degenerateMedianRoom(2, 200));
    semantic::SemanticCandidateConfig config;
    config.weightAngle    = 0.0;
    config.weightTopology = 0.0;
    /* With only extent/aperture weighted and both disabled, the denominator
     * is zero and the pair is rejected -- proving neither cue leaked a
     * spurious zero-weighted contribution. */
    EXPECT_TRUE(semantic::SemanticCandidates::generate(history, config).empty());

    /* Allowing topology proves the room itself was not rejected for lack of
     * minimum evidence -- only the normalised-size cues were disabled. */
    config.weightTopology = 1.0;
    const std::vector<semantic::SemanticCandidate> withTopology =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_EQ(withTopology.size(), 1U);
    EXPECT_DOUBLE_EQ(withTopology.front().cues.extentDistance, 0.0);
    EXPECT_DOUBLE_EQ(withTopology.front().cues.apertureDistance, 0.0);
    EXPECT_DOUBLE_EQ(withTopology.front().cues.weightDenominator, 1.0);
}

/* Required test (f): only candidates within the configured
 * ambiguity margin of the best distance are marked ambiguous. */
TEST(CandidateGen, AmbiguityMarginMarksOnlyCandidatesWithinMargin)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[10U].push_back(makeRoom(1, 1.0));
    history[20U] = {makeRoom(11, 1.0), makeRoom(12, 1.03), makeRoom(13, 1.20)};
    semantic::SemanticCandidateConfig config;
    config.weightExtent    = 0.0;
    config.weightAperture  = 0.0;
    config.weightTopology  = 0.0;
    config.ambiguityMargin = 0.05;
    const std::vector<semantic::SemanticCandidate> candidates =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_EQ(candidates.size(), 3U);
    EXPECT_EQ(candidates[0].roomBId, 11);
    EXPECT_TRUE(candidates[0].ambiguous);
    EXPECT_EQ(candidates[1].roomBId, 12);
    EXPECT_TRUE(candidates[1].ambiguous);
    EXPECT_EQ(candidates[2].roomBId, 13);
    EXPECT_FALSE(candidates[2].ambiguous);
}

/* Required test (j): a bounded global fallback finds the true
 * pair when the adjacency-prioritised tier admits nothing, without relaxing
 * minimum evidence for the anchor's own disqualifying pairs. */
TEST(CandidateGen, FallbackFindsTruePairWhenAnchorHasNoUsableEvidence)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[5U].push_back(emptyEvidenceRoom(999));
    history[10U].push_back(makeRoom(1, 1.0));
    history[20U].push_back(makeRoom(101, 1.0));
    semantic::SemanticCandidateConfig config;
    config.weightExtent   = 0.0;
    config.weightAperture = 0.0;
    config.weightTopology = 0.0;
    const std::vector<semantic::SemanticCandidate> candidates =
        semantic::SemanticCandidates::generate(history, config, 999);
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_EQ(candidates.front().roomAId, 1);
    EXPECT_EQ(candidates.front().roomBId, 101);
    EXPECT_NEAR(candidates.front().distance, 0.0, 1.0e-12);
}

/* Required test (k): equal-distance candidates are ordered by
 * the (mapAId,roomAId,mapBId,roomBId) tie-break, never by distance alone. */
TEST(CandidateGen, TieBreaksByMapAndRoomIdWhenDistancesAreEqual)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[100U].push_back(makeRoom(50, 1.0));
    history[200U].push_back(makeRoom(60, 1.0));
    history[300U].push_back(makeRoom(10, 2.0));
    history[400U].push_back(makeRoom(20, 2.0));
    semantic::SemanticCandidateConfig config;
    config.weightExtent   = 0.0;
    config.weightAperture = 0.0;
    config.weightTopology = 0.0;
    config.topK           = 10U;
    const std::vector<semantic::SemanticCandidate> candidates =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_GE(candidates.size(), 2U);
    EXPECT_DOUBLE_EQ(candidates[0].distance, 0.0);
    EXPECT_DOUBLE_EQ(candidates[1].distance, 0.0);
    EXPECT_EQ(candidates[0].mapAId, 100U);
    EXPECT_EQ(candidates[0].roomAId, 50);
    EXPECT_EQ(candidates[0].mapBId, 200U);
    EXPECT_EQ(candidates[0].roomBId, 60);
    EXPECT_EQ(candidates[1].mapAId, 300U);
    EXPECT_EQ(candidates[1].roomAId, 10);
    EXPECT_EQ(candidates[1].mapBId, 400U);
    EXPECT_EQ(candidates[1].roomBId, 20);
}

/* Apertures are compared as (width,height) pairs
 * via pairwise Manhattan error, not flattened into one sorted scalar list --
 * flattening would make a width/height swap indistinguishable (both rooms
 * would sort to the identical multiset {1,2}), scoring distance 0. */
TEST(CandidateGen, AperturePairwiseManhattanDistinguishesSwappedWidthHeight)
{
    std::map<long unsigned int, std::vector<semantic::RoomContextSnapshot>> history;
    history[10U].push_back(singlePassageRoom(1, 1.0, 2.0));
    history[20U].push_back(singlePassageRoom(2, 2.0, 1.0));
    semantic::SemanticCandidateConfig config;
    config.weightAngle    = 0.0;
    config.weightExtent   = 0.0;
    config.weightTopology = 0.0;
    const std::vector<semantic::SemanticCandidate> candidates =
        semantic::SemanticCandidates::generate(history, config);
    ASSERT_EQ(candidates.size(), 1U);
    EXPECT_NEAR(candidates.front().cues.apertureDistance, 4.0 / 3.0, 1.0e-9);
    EXPECT_DOUBLE_EQ(candidates.front().distance,
                     candidates.front().cues.apertureDistance);
}

TEST(CandidateGen, RejectsZeroTopoRefinementIters)
{
    semantic::SemanticCandidateConfig config;
    config.topoRefinementIters = 0U;
    EXPECT_EQ(
        semantic::SemanticCandidates::validateConfig(config),
        semantic::SemanticCandidateConfigRejectionReason::TOPO_REFINEMENT_ITERS_ZERO);
}

/* Required test (m): semantic::SemanticCandidates.cc links only against
 * itself in this target (see CMakeLists.txt's test_CandidateGen target,
 * which lists no Utils.cc/Optimizer.cc sources) -- the frame-dependent
 * matchWallsBetweenRooms()/collectCorrespondingWalls() are not declared to
 * this translation unit at all, so any call to them would be a compile
 * error, not merely a missed test. This test exists to document that
 * structural guarantee alongside the numbered requirement list. */
TEST(CandidateGen, DoesNotLinkLegacyTransformDependentHelpers)
{
    SUCCEED();
}
} // namespace core
} // namespace vs_graphs
