/*!
 * @file            private_functions.h
 *
 * @brief           Declares module-internal helpers shared between the
 *                  SemanticCandidates translation units.
 *
 * @note            These entities were file-scope members of the
 *                  anonymous namespace of SemanticCandidates.cc;
 *                  external linkage here is module-internal only.
 */

#ifndef VS_GRAPHS_CORE_SEMANTIC_SEMANTICCANDIDATES_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SEMANTIC_SEMANTICCANDIDATES_PRIVATE_FUNCTIONS_H

#include "Semantic/SemanticCandidates.h"
#include "Semantic/SemanticCandidatesStatus.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <tuple>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*!
 * @brief        One node (the room, a passage or a far-side room) of the
 *               small graph built to compare room topology.
 */
struct TopologyNode
{
    /*!
     * @brief        Text describing the node and its degree, the starting
     *               colour of the neighbour-label refinement.
     */
    std::string              label;
    /*!
     * @brief        Indices of the adjacent nodes in the graph vector.
     */
    std::vector<std::size_t> neighbours;
};

/*!
 * @brief        Tells whether a number is finite and not negative.
 *
 * @param[in]    value_in
 *               Number to test.
 *
 * @param[out]   isFiniteNonnegative_out
 *               True when value_in is finite and at least 0.
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    finiteNonnegative(const double value_in, bool &isFiniteNonnegative_out);

/*!
 * @brief        Mean absolute difference between two number lists, after
 *               padding the shorter list with a penalty value.
 *
 * @param[in]    left_in
 *               First list, compared element by element.
 *
 * @param[in]    right_in
 *               Second list, compared element by element.
 *
 * @param[in]    penalty_in
 *               Value used in place of the elements the shorter list lacks.
 *
 * @param[out]   distance_out
 *               Sum of absolute differences divided by the longer length;
 *               0 when both lists are empty.
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    paddedMeanL1(const std::vector<double> &left_in,
                 const std::vector<double> &right_in,
                 const double               penalty_in,
                 double                    &distance_out);

/*!
 * @brief        Builds the sorted list of angles between every pair of wall
 *               normals of a room, as a descriptor for comparing rooms.
 *
 * @param[in]    snapshot_in
 *               Saved room context to read.
 *
 * @param[in]    tolerance_in
 *               Angles at or below this are stored as 0, radians.
 *
 * @param[in]    cap_in
 *               Maximum number of normals read and of angles produced.
 *
 * @param[out]   angleSignature_out
 *               Ascending angles in [0, pi/2], radians, between the lines
 *               of valid wall normals (sign ignored).
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    angleSignature(const RoomContextSnapshot &snapshot_in,
                   const double               tolerance_in,
                   const std::size_t          cap_in,
                   std::vector<double>       &angleSignature_out);

/*!
 * @brief        Counts the wall normals of a room that are finite and not
 *               zero-length.
 *
 * @param[in]    snapshot_in
 *               Saved room context to read.
 *
 * @param[out]   validNormalCount_out
 *               Number of usable wall normals.
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    validNormalCount(const RoomContextSnapshot &snapshot_in,
                     std::size_t               &validNormalCount_out);

/*!
 * @brief        Tells whether wall bounds are usable: flagged valid, finite
 *               and spanning a non-empty rectangle.
 *
 * @param[in]    bounds_in
 *               Wall bounds to test.
 *
 * @param[out]   isValidWallBounds_out
 *               True when the bounds are usable.
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    isValidWallBounds(const WallBounds &bounds_in, bool &isValidWallBounds_out);

[[nodiscard]] SemanticCandidatesStatus
    validWallEvidenceCount(const RoomContextSnapshot &snapshot_in,
                           std::size_t &validWallEvidenceCount_out);

[[nodiscard]] SemanticCandidatesStatus
    missingBoundsFraction(const RoomContextSnapshot &snapshot_in,
                          double                    &missingBoundsFraction_out);

/*!
 * @brief        Median of the U and V spans of the valid wall bounds of a room,
 *               used to make extents independent of room size.
 *
 * @param[in]    snapshot_in
 *               Saved room context to read.
 *
 * @param[in]    cap_in
 *               Limit on how many spans are collected.
 *
 * @param[out]   medianExtent_out
 *               Median span, metres; 0 when no wall has valid bounds.
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    medianExtent(const RoomContextSnapshot &snapshot_in,
                 const std::size_t          cap_in,
                 double                    &medianExtent_out);

/*!
 * @brief        Builds the sorted list of wall U and V spans of a room, each
 *               divided by the median span.
 *
 * @param[in]    snapshot_in
 *               Saved room context to read.
 *
 * @param[in]    median_in
 *               Median span of the room, metres; a non-finite or non-positive
 *               value gives an empty result.
 *
 * @param[in]    cap_in
 *               Maximum number of elements produced.
 *
 * @param[out]   extentSignature_out
 *               Ascending normalised spans (unitless) of the walls with valid
 *               bounds.
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    extentSignature(const RoomContextSnapshot &snapshot_in,
                    const double               median_in,
                    const std::size_t          cap_in,
                    std::vector<double>       &extentSignature_out);

[[nodiscard]] SemanticCandidatesStatus apertureSignature(
    const RoomContextSnapshot              &snapshot_in,
    const double                            median_in,
    const std::size_t                       cap_in,
    std::vector<std::pair<double, double>> &apertureSignature_out);

[[nodiscard]] SemanticCandidatesStatus pairedManhattanDistance(
    const std::vector<std::pair<double, double>> &left_in,
    const std::vector<std::pair<double, double>> &right_in,
    const double                                  penalty_in,
    double                                       &distance_out);

/*!
 * @brief        Builds a descriptor of how a room connects to its passages and
 *               far-side rooms, by relabelling a small graph with the labels of
 *               its neighbours.
 *
 * @param[in]    snapshot_in
 *               Saved room context to read.
 *
 * @param[in]    cap_in
 *               Maximum number of graph nodes; a larger graph gives an
 *               empty result.
 *
 * @param[in]    refinementIters_in
 *               Number of neighbour-label refinement rounds.
 *
 * @param[out]   topologySignature_out
 *               Sorted node labels; empty when the room has no passage or the
 *               graph would exceed cap_in.
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    topologySignature(const RoomContextSnapshot &snapshot_in,
                      const std::size_t          cap_in,
                      const unsigned int         refinementIters_in,
                      std::vector<std::string>  &topologySignature_out);

/*!
 * @brief        Fraction of positions at which two label lists differ.
 *
 * @param[in]    left_in
 *               First label list.
 *
 * @param[in]    right_in
 *               Second label list.
 *
 * @param[out]   distance_out
 *               Mismatches divided by the longer length, in [0, 1]; a missing
 *               position counts as a mismatch; 0 when both lists are empty.
 *
 * @return       SEMANTIC_CANDIDATES_STATUS_SUCCESS always.
 */
[[nodiscard]] SemanticCandidatesStatus
    stringDistance(const std::vector<std::string> &left_in,
                   const std::vector<std::string> &right_in,
                   double                         &distance_out);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // VS_GRAPHS_CORE_SEMANTIC_SEMANTICCANDIDATES_PRIVATE_FUNCTIONS_H
