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

/*!
 * @brief        Per-cue distances between two rooms and the weights that
 *               combined them into one candidate distance.
 */
struct CandidateCueBreakdown
{
    /*!
     * @brief        Distance between the rooms' wall-angle descriptors
     *               (mean absolute difference, radians).
     */
    double angleDistance{0.0};
    /*!
     * @brief        Distance between the rooms' wall-extent descriptors
     *               (extents divided by each room's median extent, so
     *               unitless).
     */
    double extentDistance{0.0};
    /*!
     * @brief        Distance between the rooms' passage width/height
     *               descriptors (normalised like the extents, unitless).
     */
    double apertureDistance{0.0};
    /*!
     * @brief        Fraction of mismatching entries between the rooms'
     *               passage-graph signatures, in [0, 1]; 0 when
     *               isTopologyAvailable is false.
     */
    double topologyDistance{0.0};
    /*!
     * @brief        True when both rooms produced a topology signature, so
     *               topologyDistance took part in the distance.
     */
    bool   isTopologyAvailable{false};
    /*!
     * @brief        Reserved for a truncation flag; nothing sets it yet, so it
     *               is always false.
     */
    bool   isTruncated{false};
    /*!
     * @brief        Reserved for a runtime-budget flag; nothing sets it yet,
     *               so it is always false.
     */
    bool   isRuntimeBudgetExceeded{false};
    /*!
     * @brief        Sum of each used cue's weight times its distance.
     */
    double weightedNumerator{0.0};
    /*!
     * @brief        Sum of the weights of the cues that were used; the
     *               candidate distance is weightedNumerator divided by it.
     */
    double weightDenominator{0.0};
};

/*!
 * @brief        Proposed pairing of a room from one map with a room from
 *               another map, scored by how alike their saved context is.
 *               A proposal only; it is verified later.
 */
struct SemanticCandidate
{
    /*!
     * @brief        Atlas map id of the first room's map.
     */
    long unsigned int     mapAId{0U};
    /*!
     * @brief        Id of the first room.
     */
    int                   roomAId{0};
    /*!
     * @brief        Atlas map id of the second room's map.
     */
    long unsigned int     mapBId{0U};
    /*!
     * @brief        Id of the second room.
     */
    int                   roomBId{0};
    /*!
     * @brief        Combined cue distance (weightedNumerator over
     *               weightDenominator); smaller means more alike.
     */
    double                distance{0.0};
    /*!
     * @brief        Individual cue distances and weights behind distance.
     */
    CandidateCueBreakdown cues;
    /*!
     * @brief        True when either room has fewer than two valid wall
     *               normals or more than half of its walls lack valid
     *               bounds.
     */
    bool                  hasLowConfidence{false};
    /*!
     * @brief        True when distance is within ambiguityMargin of the best
     *               candidate's distance (the best one included).
     */
    bool                  isAmbiguous{false};
    /*!
     * @brief        True when both rooms met the minimum evidence (two valid
     *               walls, or one wall and one passage). Pairs that fail are
     *               not produced, so this is true for every generated
     *               candidate.
     */
    bool                  isMinimumEvidenceSatisfied{false};
};

/*!
 * @brief        Limits, weights and thresholds of candidate generation.
 *               validateConfig() rejects invalid values.
 */
struct SemanticCandidateConfig
{
    /*!
     * @brief        Maximum number of candidates returned, best first.
     */
    std::size_t  topK{10U};
    /*!
     * @brief        Maximum number of room snapshots taken from the history
     *               (lowest map and room ids first) and of room pairs scored
     *               in the adjacency-prioritised pass.
     */
    std::size_t  candidatePairCap{1000U};
    /*!
     * @brief        Maximum number of room pairs scored in the global
     *               fallback pass.
     */
    std::size_t  globalFallbackCap{1000U};
    /*!
     * @brief        Maximum nodes of a room's passage graph; a room needing
     *               more gets no topology signature.
     */
    std::size_t  topologyNodesCap{128U};
    /*!
     * @brief        Weight of the angle cue in the combined distance.
     */
    double       weightAngle{1.0};
    /*!
     * @brief        Weight of the extent cue in the combined distance.
     */
    double       weightExtent{1.0};
    /*!
     * @brief        Weight of the aperture cue in the combined distance.
     */
    double       weightAperture{1.0};
    /*!
     * @brief        Weight of the topology cue in the combined distance.
     */
    double       weightTopology{1.0};
    /*!
     * @brief        Value used for angle elements one room lacks when the
     *               shorter descriptor is padded, radians.
     */
    double       angleMissingPenalty{1.0};
    /*!
     * @brief        Value used for extent elements one room lacks when the
     *               shorter descriptor is padded (normalised, unitless).
     */
    double       extentMissingPenalty{1.0};
    /*!
     * @brief        Value used for both width and height of aperture pairs
     *               one room lacks when the shorter descriptor is padded
     *               (normalised, unitless).
     */
    double       apertureMissingPenalty{1.0};
    /*!
     * @brief        Distance margin above the best candidate within which a
     *               candidate is marked ambiguous.
     */
    double       ambiguityMargin{0.05};
    /*!
     * @brief        Angles at or below this are treated as exactly parallel
     *               (0), radians.
     */
    double       angleTolerance_rad{1e-9};
    /*!
     * @brief        Runtime budget, milliseconds. Validated but not read by
     *               generation, which never consults a clock.
     */
    double       runtimeBudget_ms{0.0};
    /*!
     * @brief        Maximum number of elements in each room descriptor.
     */
    std::size_t  descriptorElementsCap{4096U};
    /*!
     * @brief        Number of neighbour-label refinement rounds used to
     *               build the topology signature.
     */
    unsigned int topoRefinementIters{3U};
};

/*!
 * @brief        Why validateConfig() rejected a SemanticCandidateConfig;
 *               NONE means it is valid.
 */
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

/*!
 * @brief        Output of candidate generation: the candidates, or the
 *               reason the configuration was rejected.
 */
struct SemanticCandidateGeneration
{
    /*!
     * @brief        Candidates sorted by ascending distance, at most topK;
     *               empty when the configuration was rejected.
     */
    std::vector<SemanticCandidate>         candidates;
    /*!
     * @brief        NONE when the configuration was valid, otherwise the first
     *               failed check.
     */
    SemanticCandidateConfigRejectionReason rejectionReason{
        SemanticCandidateConfigRejectionReason::NONE};
};

/*!
 * @brief        Generates scored room-pair candidates across maps from
 *               copied room snapshots, without touching live map data.
 */
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
