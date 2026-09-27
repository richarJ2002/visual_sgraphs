

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

bool finiteNonnegative(const double value_in)
{
    return std::isfinite(value_in) && value_in >= 0.0;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
