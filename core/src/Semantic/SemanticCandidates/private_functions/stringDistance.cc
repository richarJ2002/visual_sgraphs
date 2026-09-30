/*!
 * @file            stringDistance.cc
 *
 * @brief           Implements stringDistance(), declared in
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

SemanticCandidatesStatus
    stringDistance(const std::vector<std::string> &left_in,
                   const std::vector<std::string> &right_in,
                   double                         &distance_out)
{
    const std::size_t length = std::max(left_in.size(), right_in.size());
    if (length == 0U)
    {
        distance_out = 0.0;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    std::size_t mismatches = 0U;
    for (std::size_t index = 0U; index < length; ++index)
    {
        if (index >= left_in.size() || index >= right_in.size() ||
            left_in[index] != right_in[index])
        {
            ++mismatches;
        }
    }
    distance_out =
        static_cast<double>(mismatches) / static_cast<double>(length);
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
