

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

SemanticCandidatesStatus SemanticCandidates::rejectionReasonName(
    const SemanticCandidateConfigRejectionReason reason_in,
    const char                                 *&p_name_out)
{
    switch (reason_in)
    {
    case SemanticCandidateConfigRejectionReason::NONE:
    {
        p_name_out = "none";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::TOP_K_ZERO:
    {
        p_name_out = "top_k_zero";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::PAIR_CAP_ZERO:
    {
        p_name_out = "candidate_pair_cap_zero";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::TOPOLOGY_CAP_ZERO:
    {
        p_name_out = "topology_nodes_cap_zero";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::GLOBAL_FALLBACK_CAP_ZERO:
    {
        p_name_out = "global_fallback_cap_zero";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::TOP_K_EXCEEDS_PAIR_CAP:
    {
        p_name_out = "top_k_exceeds_pair_cap";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::
        GLOBAL_FALLBACK_EXCEEDS_PAIR_CAP:
    {
        p_name_out = "global_fallback_exceeds_pair_cap";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::DESCRIPTOR_CAP_ZERO:
    {
        p_name_out = "descriptor_elements_cap_zero";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::TOPO_REFINEMENT_ITERS_ZERO:
    {
        p_name_out = "topo_refinement_iters_zero";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NONFINITE_WEIGHT:
    {
        p_name_out = "nonfinite_weight";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NEGATIVE_WEIGHT:
    {
        p_name_out = "negative_weight";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::WEIGHT_SUM_OVERFLOW:
    {
        p_name_out = "weight_sum_overflow";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::ALL_WEIGHTS_ZERO:
    {
        p_name_out = "all_weights_zero";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NONFINITE_PENALTY:
    {
        p_name_out = "nonfinite_penalty";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NEGATIVE_PENALTY:
    {
        p_name_out = "negative_penalty";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NONFINITE_AMBIGUITY_MARGIN:
    {
        p_name_out = "nonfinite_ambiguity_margin";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NEGATIVE_AMBIGUITY_MARGIN:
    {
        p_name_out = "negative_ambiguity_margin";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NONFINITE_ANGLE_TOLERANCE:
    {
        p_name_out = "nonfinite_angle_tolerance";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NEGATIVE_ANGLE_TOLERANCE:
    {
        p_name_out = "negative_angle_tolerance";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NONFINITE_RUNTIME_BUDGET:
    {
        p_name_out = "nonfinite_runtime_budget";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    case SemanticCandidateConfigRejectionReason::NEGATIVE_RUNTIME_BUDGET:
    {
        p_name_out = "negative_runtime_budget";
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    }
    p_name_out = "unknown";
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
