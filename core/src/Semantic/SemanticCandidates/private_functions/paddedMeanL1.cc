

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

double paddedMeanL1(const std::vector<double> &left_in,
                    const std::vector<double> &right_in,
                    const double               penalty_in)
{
    const std::size_t length = std::max(left_in.size(), right_in.size());
    if (length == 0U)
    {
        return 0.0;
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
    return sum / static_cast<double>(length);
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
