/**
 * @file test_GlobalWallMetrics.cpp
 * @brief Self-test for the scoped global-wall-metrics evaluation adapter
 *.
 *
 * `compare_sgraph_to_ground_truth.py`'s wall precision/recall denominator
 * only counts walls belonging to a room that matched between truth
 * and generated. A truth room with no matching generated room (or a
 * hallucinated generated room with no matching truth room) silently drops
 * that room's walls out of the denominator instead of counting them as
 * false negatives/positives. `GlobalWallMetrics` fixes the denominator while
 * reusing the comparator's own matching gates.
 */

#include "GlobalWallMetrics.h"

#include <gtest/gtest.h>

#include <fstream>

#ifndef VS_GRAPHS_WORKSPACE_ROOT
#error "VS_GRAPHS_WORKSPACE_ROOT must be defined by CMakeLists.txt"
#endif

namespace vs_graphs
{
namespace core
{
namespace test
{
namespace
{

nlohmann::json loadOfficeCleanGroundTruth()
{
    const std::string path = std::string(VS_GRAPHS_WORKSPACE_ROOT) +
                             "/src/environment/scenes/office_clean/"
                             "office_clean_ground_truth_sgraph.json";
    std::ifstream file(path);
    if (!file.is_open())
    {
        ADD_FAILURE() << "could not open ground truth fixture: " << path;
        return nlohmann::json{{"rooms", nlohmann::json::array()},
                              {"walls", nlohmann::json::array()}};
    }
    nlohmann::json parsed;
    file >> parsed;
    return parsed;
}

} // namespace

TEST(GlobalWallMetrics, IdenticalGraphScoresPerfectGlobalMatch)
{
    const nlohmann::json truth = loadOfficeCleanGroundTruth();
    ASSERT_EQ(truth.at("walls").size(), 56U)
        << "office_clean ground truth fixture drifted from the "
           "recorded baseline (14 rooms, 56 walls, 13 passages)";

    const WallPrfResult result = computeGlobalWallMetrics(truth, truth);

    EXPECT_EQ(result.matched, 56U);
    EXPECT_EQ(result.generated, 56U);
    EXPECT_EQ(result.groundTruth, 56U);
    EXPECT_DOUBLE_EQ(result.recall, 1.0);
    EXPECT_DOUBLE_EQ(result.precision, 1.0);
    EXPECT_DOUBLE_EQ(result.f1, 1.0);
}

/**
 * Toy case proving the G17 fix: truth room 3 has no matching generated room,
 * and generated room 103 has no matching truth room. Both rooms' single
 * walls must surface as, respectively, a false negative and a false
 * positive in the *global* denominator -- even though the comparator's
 * per-matched-room wall denominator would never see either wall (2 truth
 * walls in matched rooms, 2 generated walls in matched rooms, both
 * matching: a misleading 1.0/1.0).
 *
 * Truth room 3 and generated room 103 are placed close to each other
 * (4 m apart, just past `MAX_ROOM_MATCH_DIST_M`) but far from the two good
 * pairs' cluster near the origin. This is deliberate: with equal truth/
 * generated room counts (3 vs 3), the full-matrix assignment is a complete
 * permutation, so an outlier placed asymmetrically relative to the good
 * cluster (verified against `scipy.optimize.linear_sum_assignment` while
 * designing this fixture) can make the optimal assignment "trade" a good
 * pair for a slightly cheaper total cost -- room 3 and room 103 being
 * mutually near, but both far from every other room, keeps the natural
 * pairing (1<->101, 2<->102, 3<->103) unambiguously cheapest, so no such
 * reshuffling can occur here.
 */
TEST(GlobalWallMetrics,
     EntirelyUnmatchedRoomsBecomeFalseNegativeAndFalsePositive)
{
    const nlohmann::json truth = {
        {"rooms",
         {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}},
          {{"id", 2}, {"centroid_xy", {10.0, 0.0}}},
          {{"id", 3}, {"centroid_xy", {500.0, 500.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}},
          {{"room_id", 2}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 10.0}},
          {{"room_id", 3}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 100.0}}}}};

    const nlohmann::json generated = {
        {"rooms",
         {{{"id", 101}, {"centroid_xy", {0.1, 0.0}}},
          {{"id", 102}, {"centroid_xy", {10.05, 0.0}}},
          {{"id", 103}, {"centroid_xy", {504.0, 500.0}}}}},
        {"walls",
         {{{"room_id", 101}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.05}},
          {{"room_id", 102}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 10.02}},
          {{"room_id", 103},
           {"normal", {1.0, 0.0, 0.0}},
           {"offset_d", -50.0}}}}};

    const WallPrfResult result = computeGlobalWallMetrics(truth, generated);

    /* Rooms 1<->101 and 2<->102 match and their walls match; room 3 (truth)
     * and room 103 (generated) are each other's cheapest remaining
     * assignment (4 m apart) but that pair still exceeds the 3 m gate, so
     * neither of their walls is counted as matched. */
    EXPECT_EQ(result.matched, 2U);
    EXPECT_EQ(result.groundTruth, 3U) << "truth room 3's wall must still "
                                         "count in the denominator";
    EXPECT_EQ(result.generated, 3U) << "generated room 103's wall must "
                                       "still count in the denominator";
    EXPECT_NEAR(result.recall, 2.0 / 3.0, 1e-9);
    EXPECT_NEAR(result.precision, 2.0 / 3.0, 1e-9);
    EXPECT_LT(result.recall, 1.0);
    EXPECT_LT(result.precision, 1.0);
}

TEST(GlobalWallMetrics, EmptyGraphsReportZeroRatherThanDivideByZero)
{
    const nlohmann::json empty = {{"rooms", nlohmann::json::array()},
                                  {"walls", nlohmann::json::array()}};

    const WallPrfResult result = computeGlobalWallMetrics(empty, empty);

    EXPECT_EQ(result.matched, 0U);
    EXPECT_DOUBLE_EQ(result.recall, 0.0);
    EXPECT_DOUBLE_EQ(result.precision, 0.0);
    EXPECT_DOUBLE_EQ(result.f1, 0.0);
}

/**
 * Red-first counterexample (semantic-axiom-reliability-plan.md, plan-owner
 * decision reopening P0.5): nearest-edge greedy room matching and the Python
 * comparator's full-matrix Hungarian assignment accept *different* room
 * pairs once the post-assignment MAX_ROOM_MATCH_DIST_M gate is applied, and
 * the wrong pair changes the global matched-wall count.
 *
 * Truth room 1 (x=0) and room 2 (x=2); generated room 101 (x=1.1) and room
 * 102 (x=10). Local nearest-edge greedy sorts candidates within the 3 m gate
 * by ascending distance and takes room2<->room101 first (0.9 m); room101 is
 * then used up, so room1<->room101 (1.1 m) is never considered, and room1 /
 * room102 are left unmatched. The full assignment instead minimizes total
 * cost over both pairs before gating anything: room1<->room101 +
 * room2<->room102 (1.1 + 8.0 = 9.1) beats room1<->room102 +
 * room2<->room101 (10.0 + 0.9 = 10.9), so the optimal assignment is
 * room1<->room101 and room2<->room102; only then does the 3 m gate reject
 * room2<->room102 (8.0 m), leaving room1<->room101 as the sole accepted pair.
 *
 * semantic::Room 1 and generated room 101 share the same wall plane (matched); room
 * 2's wall is orthogonal to it. So the two methods disagree not just on
 * which rooms pair, but on the resulting global wall match count: 1
 * (correct, room1<->room101) versus 0 (greedy's wrong room2<->room101,
 * whose orthogonal walls never satisfy the normal-angle gate).
 */
TEST(GlobalWallMetrics,
     FullAssignmentAcceptsDifferentRoomPairThanGreedyAndChangesWallMatchCount)
{
    const nlohmann::json truth = {
        {"rooms",
         {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}},
          {{"id", 2}, {"centroid_xy", {2.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}},
          {{"room_id", 2}, {"normal", {0.0, 1.0, 0.0}}, {"offset_d", 0.0}}}}};

    const nlohmann::json generated = {
        {"rooms",
         {{{"id", 101}, {"centroid_xy", {1.1, 0.0}}},
          {{"id", 102}, {"centroid_xy", {10.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 101},
           {"normal", {1.0, 0.0, 0.0}},
           {"offset_d", 0.05}}}}};

    const WallPrfResult result = computeGlobalWallMetrics(truth, generated);

    EXPECT_EQ(result.matched, 1U)
        << "the optimal assignment must pair truth room 1 (x=0) with "
           "generated room 101 (x=1.1), not greedy's nearest-edge pair of "
           "truth room 2 (x=2) with generated room 101";
    EXPECT_EQ(result.groundTruth, 2U);
    EXPECT_EQ(result.generated, 1U);
    EXPECT_DOUBLE_EQ(result.recall, 0.5);
    EXPECT_DOUBLE_EQ(result.precision, 1.0);
}

TEST(GlobalWallMetrics, MoreTruthRoomsThanGeneratedRoomsAssignsTheNearestRow)
{
    const nlohmann::json truth = {
        {"rooms",
         {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}},
          {{"id", 2}, {"centroid_xy", {1.0, 0.0}}},
          {{"id", 3}, {"centroid_xy", {2.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {0.0, 1.0, 0.0}}, {"offset_d", 0.0}},
          {{"room_id", 2}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}},
          {{"room_id", 3}, {"normal", {0.0, 0.0, 1.0}}, {"offset_d", 0.0}}}}};

    const nlohmann::json generated = {
        {"rooms", {{{"id", 101}, {"centroid_xy", {1.02, 0.0}}}}},
        {"walls",
         {{{"room_id", 101},
           {"normal", {1.0, 0.0, 0.0}},
           {"offset_d", 0.02}}}}};

    const WallPrfResult result = computeGlobalWallMetrics(truth, generated);

    EXPECT_EQ(result.matched, 1U)
        << "the single generated room must be assigned to truth room 2 "
           "(x=1, the closest row), not truth room 1 or 3";
    EXPECT_EQ(result.groundTruth, 3U);
    EXPECT_EQ(result.generated, 1U);
}

TEST(GlobalWallMetrics, MoreGeneratedRoomsThanTruthRoomsAssignsTheNearestColumn)
{
    const nlohmann::json truth = {
        {"rooms", {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}}}}};

    const nlohmann::json generated = {
        {"rooms",
         {{{"id", 101}, {"centroid_xy", {5.0, 0.0}}},
          {{"id", 102}, {"centroid_xy", {0.02, 0.0}}},
          {{"id", 103}, {"centroid_xy", {-5.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 101}, {"normal", {0.0, 1.0, 0.0}}, {"offset_d", 0.0}},
          {{"room_id", 102}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.02}},
          {{"room_id", 103}, {"normal", {0.0, 0.0, 1.0}}, {"offset_d", 0.0}}}}};

    const WallPrfResult result = computeGlobalWallMetrics(truth, generated);

    EXPECT_EQ(result.matched, 1U)
        << "the single truth room must be assigned to generated room 102 "
           "(x=0.02, the closest column), not 101 or 103";
    EXPECT_EQ(result.groundTruth, 1U);
    EXPECT_EQ(result.generated, 3U);
}

TEST(GlobalWallMetrics, AllRoomPairsBeyondGateProduceNoRoomMatches)
{
    const nlohmann::json truth = {
        {"rooms", {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}}}}};

    const nlohmann::json generated = {
        {"rooms", {{{"id", 101}, {"centroid_xy", {100.0, 100.0}}}}},
        {"walls",
         {{{"room_id", 101}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}}}}};

    const WallPrfResult result = computeGlobalWallMetrics(truth, generated);

    EXPECT_EQ(result.matched, 0U)
        << "the only room pair is far beyond MAX_ROOM_MATCH_DIST_M and must "
           "never be room-matched, even though its walls would otherwise "
           "pass the wall-plane gate";
    EXPECT_EQ(result.groundTruth, 1U);
    EXPECT_EQ(result.generated, 1U);
    EXPECT_DOUBLE_EQ(result.recall, 0.0);
    EXPECT_DOUBLE_EQ(result.precision, 0.0);
}

/**
 * A fully symmetric 2x2 room layout: truth rooms at (0,0)/(1,1) and generated
 * rooms at (1,0)/(0,1) are all mutually 1 m apart, so the two disjoint
 * assignments (1<->101,2<->102 or 1<->102,2<->101) tie at the same total
 * cost. All four rooms share identical wall geometry, so both optimal
 * assignments happen to produce the same wall-match outcome here; the point
 * of this test is that repeated calls on identical input never disagree
 * with each other or with themselves.
 */
TEST(GlobalWallMetrics, AssignmentIsDeterministicAcrossRepeatedCalls)
{
    const nlohmann::json truth = {
        {"rooms",
         {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}},
          {{"id", 2}, {"centroid_xy", {1.0, 1.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}},
          {{"room_id", 2}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}}}}};

    const nlohmann::json generated = {
        {"rooms",
         {{{"id", 101}, {"centroid_xy", {1.0, 0.0}}},
          {{"id", 102}, {"centroid_xy", {0.0, 1.0}}}}},
        {"walls",
         {{{"room_id", 101}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}},
          {{"room_id", 102}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}}}}};

    for (int attempt = 0; attempt < 5; ++attempt)
    {
        const WallPrfResult result = computeGlobalWallMetrics(truth, generated);
        EXPECT_EQ(result.matched, 2U) << "attempt " << attempt;
        EXPECT_EQ(result.groundTruth, 2U) << "attempt " << attempt;
        EXPECT_EQ(result.generated, 2U) << "attempt " << attempt;
        EXPECT_DOUBLE_EQ(result.recall, 1.0) << "attempt " << attempt;
        EXPECT_DOUBLE_EQ(result.precision, 1.0) << "attempt " << attempt;
    }
}

/**
 * Mirrors compare_sgraph_to_ground_truth.py's wall_matches(): for each truth
 * wall, in input order, iterate generated walls in input order and keep the
 * first one on an exact score tie (only a strictly lower score replaces the
 * current best). Truth wall 1 ties between generated walls 1 and 2 (both
 * 0.30 m from its offset); if the implementation instead kept the *second*
 * tied candidate, truth wall 2 (0.35 m offset, only 0.05 m from generated
 * wall 1) would also find a gate-passing match and the total matched count
 * would be 2 instead of the correct 1.
 */
TEST(GlobalWallMetrics, WallMatchingRetainsFirstGeneratedWallOnExactScoreTie)
{
    const nlohmann::json truth = {
        {"rooms", {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}},
          {{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.35}}}}};

    const nlohmann::json generated = {
        {"rooms", {{{"id", 101}, {"centroid_xy", {0.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 101}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.30}},
          {{"room_id", 101},
           {"normal", {1.0, 0.0, 0.0}},
           {"offset_d", -0.30}}}}};

    const WallPrfResult result = computeGlobalWallMetrics(truth, generated);

    EXPECT_EQ(result.matched, 1U)
        << "truth wall 1 must keep the first tied generated wall (offset "
           "0.30), leaving generated wall (offset -0.30) unused so truth "
           "wall 2 (offset 0.35) has no gate-passing candidate left";
    EXPECT_EQ(result.groundTruth, 2U);
    EXPECT_EQ(result.generated, 2U);
}

/**
 * Every accept/reject gate in this file (`kMaxRoomMatchDistM`,
 * `kMaxNormalAngleDeg`, `kMaxOffsetM`) is a `<=` comparison, matching the
 * comparator's own inclusive gates. None of the other tests in this file
 * happen to sit exactly on a boundary, so a regression that quietly
 * tightened any one gate to a strict `<` would pass every other test here.
 *
 * This covers the room-distance gate: room 1/101 centroids are exactly
 * `kMaxRoomMatchDistM` (3.0 m) apart. `std::sqrt(9.0)` is an exactly
 * representable perfect square, so this distance is bit-exact `3.0`, not an
 * approximation -- unlike an angle or an arbitrary distance, there is no
 * floating-point rounding risk here. If the gate were silently tightened to
 * `<`, this room pair -- and therefore its wall -- would stop matching.
 */
TEST(GlobalWallMetrics, RoomMatchGateBoundaryIsInclusiveAtExactly3Meters)
{
    const nlohmann::json truth = {
        {"rooms", {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}}}}};

    const nlohmann::json generated = {
        {"rooms", {{{"id", 101}, {"centroid_xy", {3.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 101}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.1}}}}};

    const WallPrfResult result = computeGlobalWallMetrics(truth, generated);

    EXPECT_EQ(result.matched, 1U)
        << "room 1/101 centroids are exactly sqrt(9.0) = 3.0 m apart "
           "(kMaxRoomMatchDistM); the inclusive <= gate must still accept "
           "this pair, and their (well within tolerance) walls must match";
    EXPECT_EQ(result.groundTruth, 1U);
    EXPECT_EQ(result.generated, 1U);
}

/**
 * Covers the wall-offset gate: truth offset 0.0 and generated offset 0.35
 * are exactly `kMaxOffsetM` apart, with identical normals (zero angle
 * error, so only the offset gate is exercised). Unlike an angle boundary,
 * this is reliable: `0.35` is parsed once from this file's own JSON literal
 * and independently as the `kMaxOffsetM` C++ literal in GlobalWallMetrics.cc
 * -- both go through the same correctly-rounded decimal-to-double
 * conversion (no library round-trip through a transcendental function is
 * involved), so `offsetErrorM` and `kMaxOffsetM` compare bit-identical, and
 * `offsetErrorM <= kMaxOffsetM` is a same-value (not near-value) comparison.
 * If the gate were silently tightened to `<`, this pair would stop matching.
 */
TEST(GlobalWallMetrics, WallOffsetGateBoundaryIsInclusiveAtExactly035Meters)
{
    const nlohmann::json truth = {
        {"rooms", {{{"id", 1}, {"centroid_xy", {0.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 1}, {"normal", {1.0, 0.0, 0.0}}, {"offset_d", 0.0}}}}};

    const nlohmann::json generated = {
        {"rooms", {{{"id", 101}, {"centroid_xy", {0.0, 0.0}}}}},
        {"walls",
         {{{"room_id", 101},
           {"normal", {1.0, 0.0, 0.0}},
           {"offset_d", 0.35}}}}};

    const WallPrfResult result = computeGlobalWallMetrics(truth, generated);

    EXPECT_EQ(result.matched, 1U)
        << "the offset error is exactly kMaxOffsetM (0.35 m, same JSON-vs-"
           "C++-literal double value); the inclusive <= gate must still "
           "accept this pair";
    EXPECT_EQ(result.groundTruth, 1U);
    EXPECT_EQ(result.generated, 1U);
}

/**
 * The wall-angle gate (`kMaxNormalAngleDeg`, 10 deg) is deliberately NOT
 * given an exact-boundary test analogous to the two above. Unlike a
 * perfect-square distance or a decimal literal, there is no portable way to
 * construct two unit normals whose angle -- as this file's own
 * `std::acos(dot) * 180.0 / M_PI` independently recomputes it -- is
 * bit-exactly `10.0`: building the input via `std::cos`/`std::sin` of
 * `10.0 * M_PI / 180.0` and having the gate recompute the angle via
 * `std::acos` round-trips through two transcendental functions, and on this
 * toolchain that round trip lands at approximately `10.000000000000012`, on
 * the *rejecting* side of an inclusive `<=` gate -- not because the gate is
 * wrong, but because the "nominal 10 deg" input was never exactly 10 deg to
 * begin with. A hardcoded cosine literal tuned to this toolchain's specific
 * `acos` rounding would only mask the same fragility on a different libm.
 * `<=` vs `<` for this one gate is instead verified by direct source
 * reading (see the review recorded in this plan's P0.5 evidence block) and
 * by every other test in this file that exercises small, non-boundary
 * angles (0 deg matches, 90 deg does not).
 */

} // namespace test
} // namespace core
} // namespace vs_graphs
