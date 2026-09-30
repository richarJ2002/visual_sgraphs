/*!
 * @file            pairedManhattanDistance.cc
 *
 * @brief           Implements pairedManhattanDistance(), declared in
 *                  Semantic/SemanticCandidates/private_functions.h.
 */

#include "Semantic/SemanticCandidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <tuple>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

/*! Pads the shorter list of aperture pairs with (penalty,penalty), then sums
 * the pairwise Manhattan error |Δwidth|+|Δheight| divided by the longer
 * length. Mirrors paddedMeanL1's padding rule, generalised to 2D pairs. */
SemanticCandidatesStatus pairedManhattanDistance(
    const std::vector<std::pair<double, double>> &left_in,
    const std::vector<std::pair<double, double>> &right_in,
    const double                                  penalty_in,
    double                                       &distance_out)
{
    const std::size_t length = std::max(left_in.size(), right_in.size());
    if (length == 0U)
    {
        distance_out = 0.0;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    double sum = 0.0;
    for (std::size_t index = 0U; index < length; ++index)
    {
        const std::pair<double, double> left =
            index < left_in.size() ? left_in[index]
                                   : std::make_pair(penalty_in, penalty_in);
        const std::pair<double, double> right =
            index < right_in.size() ? right_in[index]
                                    : std::make_pair(penalty_in, penalty_in);
        sum += std::abs(left.first - right.first) +
               std::abs(left.second - right.second);
    }
    distance_out = sum / static_cast<double>(length);
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
