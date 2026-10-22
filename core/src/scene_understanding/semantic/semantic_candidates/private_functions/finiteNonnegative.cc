/*!
 * @file            finiteNonnegative.cc
 *
 * @brief           Implements finiteNonnegative(), declared in
 *                  Semantic/SemanticCandidates/private_functions.h.
 */

#include "scene_understanding/semantic/semantic_candidates/semantic_candidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <tuple>

#include "scene_understanding/semantic/semantic_candidates/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticCandidatesStatus finiteNonnegative(const double value_in,
                                           bool        &isFiniteNonnegative_out)
{
    isFiniteNonnegative_out = std::isfinite(value_in) && value_in >= 0.0;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
