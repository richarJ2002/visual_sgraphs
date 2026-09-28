

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

SemanticCandidatesStatus paddedMeanL1(const std::vector<double> &left_in,
                                      const std::vector<double> &right_in,
                                      const double               penalty_in,
                                      double                    &distance_out)
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
        const double left =
            index < left_in.size() ? left_in[index] : penalty_in;
        const double right =
            index < right_in.size() ? right_in[index] : penalty_in;
        sum += std::abs(left - right);
    }
    distance_out = sum / static_cast<double>(length);
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
