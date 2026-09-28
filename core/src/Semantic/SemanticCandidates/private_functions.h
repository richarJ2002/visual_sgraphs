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

struct TopologyNode
{
    std::string              label;
    std::vector<std::size_t> neighbours;
};

[[nodiscard]] SemanticCandidatesStatus
    finiteNonnegative(const double value_in, bool &isFiniteNonnegative_out);

[[nodiscard]] SemanticCandidatesStatus
    paddedMeanL1(const std::vector<double> &left_in,
                 const std::vector<double> &right_in,
                 const double               penalty_in,
                 double                    &distance_out);

[[nodiscard]] SemanticCandidatesStatus
    angleSignature(const RoomContextSnapshot &snapshot_in,
                   const double               tolerance_in,
                   const std::size_t          cap_in,
                   std::vector<double>       &angleSignature_out);

[[nodiscard]] SemanticCandidatesStatus
    validNormalCount(const RoomContextSnapshot &snapshot_in,
                     std::size_t               &validNormalCount_out);

[[nodiscard]] SemanticCandidatesStatus
    isValidWallBounds(const WallBounds &bounds_in, bool &isValidWallBounds_out);

[[nodiscard]] SemanticCandidatesStatus
    validWallEvidenceCount(const RoomContextSnapshot &snapshot_in,
                           std::size_t &validWallEvidenceCount_out);

[[nodiscard]] SemanticCandidatesStatus
    missingBoundsFraction(const RoomContextSnapshot &snapshot_in,
                          double                    &missingBoundsFraction_out);

[[nodiscard]] SemanticCandidatesStatus
    medianExtent(const RoomContextSnapshot &snapshot_in,
                 const std::size_t          cap_in,
                 double                    &medianExtent_out);

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

[[nodiscard]] SemanticCandidatesStatus
    topologySignature(const RoomContextSnapshot &snapshot_in,
                      const std::size_t          cap_in,
                      const unsigned int         refinementIters_in,
                      std::vector<std::string>  &topologySignature_out);

[[nodiscard]] SemanticCandidatesStatus
    stringDistance(const std::vector<std::string> &left_in,
                   const std::vector<std::string> &right_in,
                   double                         &distance_out);

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // VS_GRAPHS_CORE_SEMANTIC_SEMANTICCANDIDATES_PRIVATE_FUNCTIONS_H
