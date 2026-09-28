

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

SemanticCandidatesStatus SemanticCandidates::validateConfig(
    const SemanticCandidateConfig          &configuration_in,
    SemanticCandidateConfigRejectionReason &rejectionReason_out)
{
    if (configuration_in.topK == 0U)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::TOP_K_ZERO;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.candidatePairCap == 0U)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::PAIR_CAP_ZERO;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.topologyNodesCap == 0U)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::TOPOLOGY_CAP_ZERO;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.globalFallbackCap == 0U)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::GLOBAL_FALLBACK_CAP_ZERO;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.descriptorElementsCap == 0U)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::DESCRIPTOR_CAP_ZERO;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.topoRefinementIters == 0U)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::TOPO_REFINEMENT_ITERS_ZERO;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.topK > configuration_in.candidatePairCap)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::TOP_K_EXCEEDS_PAIR_CAP;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.globalFallbackCap > configuration_in.candidatePairCap)
    {
        rejectionReason_out = SemanticCandidateConfigRejectionReason::
            GLOBAL_FALLBACK_EXCEEDS_PAIR_CAP;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    const double weights[] = {configuration_in.weightAngle,
                              configuration_in.weightExtent,
                              configuration_in.weightAperture,
                              configuration_in.weightTopology};
    double       weightSum = 0.0;
    for (const double weight : weights)
    {
        if (!std::isfinite(weight))
        {
            rejectionReason_out =
                SemanticCandidateConfigRejectionReason::NONFINITE_WEIGHT;
            return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
        }
        if (weight < 0.0)
        {
            rejectionReason_out =
                SemanticCandidateConfigRejectionReason::NEGATIVE_WEIGHT;
            return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
        }
        weightSum += weight;
    }
    if (!std::isfinite(weightSum))
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::WEIGHT_SUM_OVERFLOW;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (weightSum == 0.0)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::ALL_WEIGHTS_ZERO;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    const double penalties[] = {configuration_in.angleMissingPenalty,
                                configuration_in.extentMissingPenalty,
                                configuration_in.apertureMissingPenalty};
    for (const double penalty : penalties)
    {
        if (!std::isfinite(penalty))
        {
            rejectionReason_out =
                SemanticCandidateConfigRejectionReason::NONFINITE_PENALTY;
            return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
        }
        if (penalty < 0.0)
        {
            rejectionReason_out =
                SemanticCandidateConfigRejectionReason::NEGATIVE_PENALTY;
            return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
        }
    }
    if (!std::isfinite(configuration_in.ambiguityMargin))
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::NONFINITE_AMBIGUITY_MARGIN;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.ambiguityMargin < 0.0)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::NEGATIVE_AMBIGUITY_MARGIN;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (!std::isfinite(configuration_in.angleTolerance_rad))
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::NONFINITE_ANGLE_TOLERANCE;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.angleTolerance_rad < 0.0)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::NEGATIVE_ANGLE_TOLERANCE;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (!std::isfinite(configuration_in.runtimeBudget_ms))
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::NONFINITE_RUNTIME_BUDGET;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    if (configuration_in.runtimeBudget_ms < 0.0)
    {
        rejectionReason_out =
            SemanticCandidateConfigRejectionReason::NEGATIVE_RUNTIME_BUDGET;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }
    rejectionReason_out = SemanticCandidateConfigRejectionReason::NONE;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
