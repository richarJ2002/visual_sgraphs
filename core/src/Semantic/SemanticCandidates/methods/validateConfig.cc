

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

SemanticCandidateConfigRejectionReason SemanticCandidates::validateConfig(
    const SemanticCandidateConfig &configuration_in)
{
    if (configuration_in.topK == 0U)
        return SemanticCandidateConfigRejectionReason::TOP_K_ZERO;
    if (configuration_in.candidatePairCap == 0U)
        return SemanticCandidateConfigRejectionReason::PAIR_CAP_ZERO;
    if (configuration_in.topologyNodesCap == 0U)
        return SemanticCandidateConfigRejectionReason::TOPOLOGY_CAP_ZERO;
    if (configuration_in.globalFallbackCap == 0U)
        return SemanticCandidateConfigRejectionReason::GLOBAL_FALLBACK_CAP_ZERO;
    if (configuration_in.descriptorElementsCap == 0U)
        return SemanticCandidateConfigRejectionReason::DESCRIPTOR_CAP_ZERO;
    if (configuration_in.topoRefinementIters == 0U)
        return SemanticCandidateConfigRejectionReason::
            TOPO_REFINEMENT_ITERS_ZERO;
    if (configuration_in.topK > configuration_in.candidatePairCap)
        return SemanticCandidateConfigRejectionReason::TOP_K_EXCEEDS_PAIR_CAP;
    if (configuration_in.globalFallbackCap > configuration_in.candidatePairCap)
        return SemanticCandidateConfigRejectionReason::
            GLOBAL_FALLBACK_EXCEEDS_PAIR_CAP;
    const double weights[] = {configuration_in.weightAngle,
                              configuration_in.weightExtent,
                              configuration_in.weightAperture,
                              configuration_in.weightTopology};
    double       weightSum = 0.0;
    for (const double weight : weights)
    {
        if (!std::isfinite(weight))
            return SemanticCandidateConfigRejectionReason::NONFINITE_WEIGHT;
        if (weight < 0.0)
            return SemanticCandidateConfigRejectionReason::NEGATIVE_WEIGHT;
        weightSum += weight;
    }
    if (!std::isfinite(weightSum))
        return SemanticCandidateConfigRejectionReason::WEIGHT_SUM_OVERFLOW;
    if (weightSum == 0.0)
        return SemanticCandidateConfigRejectionReason::ALL_WEIGHTS_ZERO;
    const double penalties[] = {configuration_in.angleMissingPenalty,
                                configuration_in.extentMissingPenalty,
                                configuration_in.apertureMissingPenalty};
    for (const double penalty : penalties)
    {
        if (!std::isfinite(penalty))
            return SemanticCandidateConfigRejectionReason::NONFINITE_PENALTY;
        if (penalty < 0.0)
            return SemanticCandidateConfigRejectionReason::NEGATIVE_PENALTY;
    }
    if (!std::isfinite(configuration_in.ambiguityMargin))
        return SemanticCandidateConfigRejectionReason::
            NONFINITE_AMBIGUITY_MARGIN;
    if (configuration_in.ambiguityMargin < 0.0)
        return SemanticCandidateConfigRejectionReason::
            NEGATIVE_AMBIGUITY_MARGIN;
    if (!std::isfinite(configuration_in.angleTolerance_rad))
        return SemanticCandidateConfigRejectionReason::
            NONFINITE_ANGLE_TOLERANCE;
    if (configuration_in.angleTolerance_rad < 0.0)
        return SemanticCandidateConfigRejectionReason::NEGATIVE_ANGLE_TOLERANCE;
    if (!std::isfinite(configuration_in.runtimeBudget_ms))
        return SemanticCandidateConfigRejectionReason::NONFINITE_RUNTIME_BUDGET;
    if (configuration_in.runtimeBudget_ms < 0.0)
        return SemanticCandidateConfigRejectionReason::NEGATIVE_RUNTIME_BUDGET;
    return SemanticCandidateConfigRejectionReason::NONE;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
