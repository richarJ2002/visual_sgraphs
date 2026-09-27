

#include "Semantic/SemanticCandidates.h"

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

const char *SemanticCandidates::rejectionReasonName(
    const SemanticCandidateConfigRejectionReason reason_in)
{
    switch (reason_in)
    {
    case SemanticCandidateConfigRejectionReason::NONE:
        return "none";
    case SemanticCandidateConfigRejectionReason::TOP_K_ZERO:
        return "top_k_zero";
    case SemanticCandidateConfigRejectionReason::PAIR_CAP_ZERO:
        return "candidate_pair_cap_zero";
    case SemanticCandidateConfigRejectionReason::TOPOLOGY_CAP_ZERO:
        return "topology_nodes_cap_zero";
    case SemanticCandidateConfigRejectionReason::GLOBAL_FALLBACK_CAP_ZERO:
        return "global_fallback_cap_zero";
    case SemanticCandidateConfigRejectionReason::TOP_K_EXCEEDS_PAIR_CAP:
        return "top_k_exceeds_pair_cap";
    case SemanticCandidateConfigRejectionReason::
        GLOBAL_FALLBACK_EXCEEDS_PAIR_CAP:
        return "global_fallback_exceeds_pair_cap";
    case SemanticCandidateConfigRejectionReason::DESCRIPTOR_CAP_ZERO:
        return "descriptor_elements_cap_zero";
    case SemanticCandidateConfigRejectionReason::TOPO_REFINEMENT_ITERS_ZERO:
        return "topo_refinement_iters_zero";
    case SemanticCandidateConfigRejectionReason::NONFINITE_WEIGHT:
        return "nonfinite_weight";
    case SemanticCandidateConfigRejectionReason::NEGATIVE_WEIGHT:
        return "negative_weight";
    case SemanticCandidateConfigRejectionReason::WEIGHT_SUM_OVERFLOW:
        return "weight_sum_overflow";
    case SemanticCandidateConfigRejectionReason::ALL_WEIGHTS_ZERO:
        return "all_weights_zero";
    case SemanticCandidateConfigRejectionReason::NONFINITE_PENALTY:
        return "nonfinite_penalty";
    case SemanticCandidateConfigRejectionReason::NEGATIVE_PENALTY:
        return "negative_penalty";
    case SemanticCandidateConfigRejectionReason::NONFINITE_AMBIGUITY_MARGIN:
        return "nonfinite_ambiguity_margin";
    case SemanticCandidateConfigRejectionReason::NEGATIVE_AMBIGUITY_MARGIN:
        return "negative_ambiguity_margin";
    case SemanticCandidateConfigRejectionReason::NONFINITE_ANGLE_TOLERANCE:
        return "nonfinite_angle_tolerance";
    case SemanticCandidateConfigRejectionReason::NEGATIVE_ANGLE_TOLERANCE:
        return "negative_angle_tolerance";
    case SemanticCandidateConfigRejectionReason::NONFINITE_RUNTIME_BUDGET:
        return "nonfinite_runtime_budget";
    case SemanticCandidateConfigRejectionReason::NEGATIVE_RUNTIME_BUDGET:
        return "negative_runtime_budget";
    }
    return "unknown";
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
