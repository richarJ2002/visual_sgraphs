/*!
 * @file            SemanticCandidates.h
 *
 * @brief           Declares the semantic candidate generator: its
 *                  configuration, the candidates it proposes and the cues
 *                  behind each one.
 */

/*! Declares deterministic, pre-verification semantic room candidates. */
#ifndef SEMANTIC_CANDIDATES_H
#define SEMANTIC_CANDIDATES_H

#include "Semantic/RoomContextSnapshot.h"
#include "Semantic/SemanticCandidatesStatus.h"

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

struct CandidateCueBreakdown
{
    double angleDistance{0.0};
    double extentDistance{0.0};
    double apertureDistance{0.0};
    double topologyDistance{0.0};
    bool   isTopologyAvailable{false};
    bool   isTruncated{false};
    bool   isRuntimeBudgetExceeded{false};
    double weightedNumerator{0.0};
    double weightDenominator{0.0};
};

struct SemanticCandidate
{
    long unsigned int     mapAId{0U};
    int                   roomAId{0};
    long unsigned int     mapBId{0U};
    int                   roomBId{0};
    double                distance{0.0};
    CandidateCueBreakdown cues;
    bool                  hasLowConfidence{false};
    bool                  isAmbiguous{false};
    bool                  isMinimumEvidenceSatisfied{false};
};

struct SemanticCandidateConfig
{
    std::size_t  topK{10U};
    std::size_t  candidatePairCap{1000U};
    std::size_t  globalFallbackCap{1000U};
    std::size_t  topologyNodesCap{128U};
    double       weightAngle{1.0};
    double       weightExtent{1.0};
    double       weightAperture{1.0};
    double       weightTopology{1.0};
    double       angleMissingPenalty{1.0};
    double       extentMissingPenalty{1.0};
    double       apertureMissingPenalty{1.0};
    double       ambiguityMargin{0.05};
    double       angleTolerance_rad{1e-9};
    double       runtimeBudget_ms{0.0};
    std::size_t  descriptorElementsCap{4096U};
    unsigned int topoRefinementIters{3U};
};

enum class SemanticCandidateConfigRejectionReason
{
    NONE,
    TOP_K_ZERO,
    PAIR_CAP_ZERO,
    TOPOLOGY_CAP_ZERO,
    GLOBAL_FALLBACK_CAP_ZERO,
    TOP_K_EXCEEDS_PAIR_CAP,
    GLOBAL_FALLBACK_EXCEEDS_PAIR_CAP,
    DESCRIPTOR_CAP_ZERO,
    TOPO_REFINEMENT_ITERS_ZERO,
    NONFINITE_WEIGHT,
    NEGATIVE_WEIGHT,
    WEIGHT_SUM_OVERFLOW,
    ALL_WEIGHTS_ZERO,
    NONFINITE_PENALTY,
    NEGATIVE_PENALTY,
    NONFINITE_AMBIGUITY_MARGIN,
    NEGATIVE_AMBIGUITY_MARGIN,
    NONFINITE_ANGLE_TOLERANCE,
    NEGATIVE_ANGLE_TOLERANCE,
    NONFINITE_RUNTIME_BUDGET,
    NEGATIVE_RUNTIME_BUDGET
};

struct SemanticCandidateGeneration
{
    std::vector<SemanticCandidate>         candidates;
    SemanticCandidateConfigRejectionReason rejectionReason{
        SemanticCandidateConfigRejectionReason::NONE};
};

class SemanticCandidates
{
  public:
    /*! Validates every candidate-generation field without touching map data. */
    [[nodiscard]] static SemanticCandidatesStatus validateConfig(
        const SemanticCandidateConfig          &configuration_in,
        SemanticCandidateConfigRejectionReason &rejectionReason_out);

    /*! Returns a stable diagnostic name for a typed rejection reason. */
    [[nodiscard]] static SemanticCandidatesStatus
        rejectionReasonName(SemanticCandidateConfigRejectionReason reason_in,
                            const char                           *&p_name_out);

    /*! Generates bounded candidates and reports configuration rejection.
     *
     *  @param history_in  Room context snapshots per map id.
     *  @param generation_out  Generated candidates, and the reason the
     *  configuration was rejected when it is invalid.
     *  @param configuration_in  Candidate-generation limits and thresholds.
     *  @param anchorRoomId_in  Optional identity of the last-confirmed
     *  room. When present, room pairs sharing a passage with that room
     *  (or involving it directly) are enumerated first, bounded by
     *  \c candidatePairCap; only if that adjacency-prioritised tier
     *  yields no candidate passing minimum evidence does a bounded
     *  global fallback (additionally capped by \c globalFallbackCap)
     *  enumerate the remaining pairs. When absent, behaviour is the
     *  unrestricted global enumeration only. */
    [[nodiscard]] static SemanticCandidatesStatus generateWithStatus(
        const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                      &history_in,
        SemanticCandidateGeneration   &generation_out,
        const SemanticCandidateConfig &configuration_in =
            SemanticCandidateConfig(),
        std::optional<int> anchorRoomId_in = std::nullopt);

    /*! Scores copied room snapshots only; no map or transform is touched. */
    [[nodiscard]] static SemanticCandidatesStatus generate(
        const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                       &history_in,
        std::vector<SemanticCandidate> &candidates_out,
        const SemanticCandidateConfig  &configuration_in =
            SemanticCandidateConfig(),
        std::optional<int> anchorRoomId_in = std::nullopt);
};

} // namespace semantic
} // namespace core
} // namespace vs_graphs

#endif // SEMANTIC_CANDIDATES_H
