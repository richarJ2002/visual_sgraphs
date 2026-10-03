/*!
 * @file            GlobalWallMetrics.h
 *
 * @brief           Scoped evaluation adapter for
 *                  semantic-axiom-reliability-plan.md
 *                         P0.5.
 *
 *                  `post_processing/compare_sgraph_to_ground_truth.py` matches
 *                  rooms by nearest centroid, then matches walls only *within*
 *                  each matched room pair (`compare()`'s
 *                  `total_wall_gen`/`total_wall_truth` denominators are built
 *                  from `truth_walls`/`gen_walls`, which are filtered to one
 *                  matched room's `room_id` each -- see its `wall_matches()`
 *                  call site). A truth room with no matching generated room (or
 *                  a hallucinated generated room with no matching truth room)
 *                  therefore drops every one of that room's walls out of the
 *                  wall precision/recall denominator entirely, instead of
 *                  counting them as false negatives/positives.
 *
 *                  This adapter reproduces the same room- and wall-matching
 *                  gates (so a generated wall counts as matched under exactly
 *                  the same rule the comparator uses) but computes
 *                  precision/recall/F1 over *every* truth and generated wall,
 *                  matched or not:
 *                    - every wall the room/wall matching accepts is a true
 *                      positive;
 *                    - every other generated wall (whether its room matched or
 *                      not) is a false positive;
 *                    - every other truth wall (whether its room matched or not)
 *                      is a false negative.
 *
 *                  semantic::Room matching here reproduces the Python
 *                  comparator's own algorithm exactly: build the full
 *                  truth-by-generated centroid-distance matrix, solve it with a
 *                  minimum-total-cost one-to-one assignment (the same
 *                  rectangular linear-sum-assignment problem
 *                  `scipy.optimize.linear_sum_assignment` solves), and only
 *                  then reject any assigned pair whose distance exceeds
 *                  `MAX_ROOM_MATCH_DIST_M`. Local nearest-edge greedy selection
 *                  is not equivalent -- it can accept a different, non-optimal
 *                  room pair once the gate is applied (see
 *                  `test_GlobalWallMetrics.cpp`'s
 *                  `FullAssignmentAcceptsDifferentRoomPairThanGreedyAndChangesWallMatchCount`),
 *                  so this file implements its own minimal rectangular
 *                  assignment solver (the classic O(n^2 m)
 *                  shortest-augmenting-path Hungarian algorithm) rather than an
 *                  approximation. This is not a line-for-line port of `scipy`'s
 *                  solver, but it solves the identical minimum-total-cost
 *                  one-to-one assignment problem, deterministically, for any
 *                  rectangular input.
 */

#pragma once

#include "Thirdparty/nlohmann/json.hpp"

#include <cstddef>

namespace vs_graphs
{
namespace core
{
namespace test
{

/*!
 * @brief           Precision/recall/F1 plus raw counts for one entity kind,
 *                  matching the comparator's own `prf()` field names.
 */
struct WallPrfResult
{
    /*!
     * @brief           Number of generated walls paired one-to-one with a
     *                  ground-truth wall.
     */
    std::size_t matched = 0;
    /*!
     * @brief           Number of walls in the generated graph (the precision
     *                  denominator).
     */
    std::size_t generated = 0;
    /*!
     * @brief           Number of walls in the ground-truth graph (the recall
     *                  denominator).
     */
    std::size_t groundTruth = 0;
    /*!
     * @brief           Fraction of ground-truth walls that were matched; 0.0
     *                  when the ground truth has no walls.
     */
    double      recall = 0.0;
    /*!
     * @brief           Fraction of generated walls that were matched; 0.0 when
     *                  no wall was generated.
     */
    double      precision = 0.0;
    /*!
     * @brief           Harmonic mean of precision and recall.
     */
    double      f1 = 0.0;
};

/*!
 * @brief           Computes global wall precision/recall/F1 across an entire
 *                  SGraph comparison: every truth and every generated wall
 *                  counts in the denominator, not only walls inside a matched
 *                  room pair.
 *
 * @param[in]       truth_in
 *                  Ground-truth SGraph JSON (`rooms`, `walls`; same schema as
 *                  `compare_sgraph_to_ground_truth.py`'s inputs).
 *
 * @param[in]       generated_in
 *                  Generated SGraph JSON, same schema.
 *
 * @return          Global wall precision/recall/F1 and raw
 *                  matched/generated/truth counts.
 */
WallPrfResult computeGlobalWallMetrics(const nlohmann::json &truth_in,
                                       const nlohmann::json &generated_in);

} // namespace test
} // namespace core
} // namespace vs_graphs
