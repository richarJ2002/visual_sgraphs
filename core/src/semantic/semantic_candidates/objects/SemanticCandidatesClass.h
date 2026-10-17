/*!
 * @file            SemanticCandidatesClass.h
 *
 * @brief           Declares the semantic candidate generator: its
 *                  configuration, the candidates it proposes and the cues
 *                  behind each one.
 */

/*!
 * @brief           Declares deterministic, pre-verification semantic room
 *                  candidates.
 */
#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATES_CLASS_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_SEMANTIC_CANDIDATES_OBJECTS_SEMANTIC_CANDIDATES_CLASS_H

#include "semantic/room/objects/RoomContextSnapshotStruct.h"
#include "semantic/semantic_candidates/objects/CandidateCueBreakdownStruct.h"
#include "semantic/semantic_candidates/objects/SemanticCandidateConfigRejectionReasonEnum.h"
#include "semantic/semantic_candidates/objects/SemanticCandidateConfigStruct.h"
#include "semantic/semantic_candidates/objects/SemanticCandidateGenerationStruct.h"
#include "semantic/semantic_candidates/objects/SemanticCandidateStruct.h"
#include "semantic/semantic_candidates/objects/SemanticCandidatesStatusEnum.h"

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
 * @brief           Generates scored room-pair candidates across maps from
 *                  copied room snapshots, without touching live map data.
 */
class SemanticCandidates
{
  public:
    /*!
     * @brief           Validates every candidate-generation field without
     *                  touching map data.
     */
    [[nodiscard]] static SemanticCandidatesStatus validateConfig(
        const SemanticCandidateConfig          &configuration_in,
        SemanticCandidateConfigRejectionReason &rejectionReason_out);

    /*!
     * @brief           Returns a stable diagnostic name for a typed rejection
     *                  reason.
     */
    [[nodiscard]] static SemanticCandidatesStatus
        rejectionReasonName(SemanticCandidateConfigRejectionReason reason_in,
                            const char                           *&p_name_out);

    /*!
     * @brief           Generates bounded candidates and reports configuration
     *                  rejection.
     *
     * @param[in]       history_in
     *                  Room context snapshots per map id.
     *
     * @param[out]      generation_out
     *                  Generated candidates, and the reason the configuration
     *                  was rejected when it is invalid.
     *
     * @param[in]       configuration_in
     *                  Candidate-generation limits and thresholds.
     *
     * @param[in]       anchorRoomId_in
     *                  Optional identity of the last-confirmed room. When
     *                  present, room pairs sharing a passage with that room (or
     *                  involving it directly) are enumerated first, bounded by
     *                  \c candidatePairCap; only if that adjacency-prioritised
     *                  tier yields no candidate passing minimum evidence does a
     *                  bounded global fallback (additionally capped by
     *                  \c globalFallbackCap) enumerate the remaining pairs.
     *                  When absent, behaviour is the unrestricted global
     *                  enumeration only.
     */
    [[nodiscard]] static SemanticCandidatesStatus generateWithStatus(
        const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                      &history_in,
        SemanticCandidateGeneration   &generation_out,
        const SemanticCandidateConfig &configuration_in =
            SemanticCandidateConfig(),
        std::optional<int> anchorRoomId_in = std::nullopt);

    /*!
     * @brief           Scores copied room snapshots only; no map or transform
     *                  is touched.
     */
    [[nodiscard]] static SemanticCandidatesStatus generate(
        const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                       &history_in,
        std::vector<SemanticCandidate> &candidates_out,
        const SemanticCandidateConfig  &configuration_in =
            SemanticCandidateConfig(),
        std::optional<int> anchorRoomId_in = std::nullopt);
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* SEMANTIC_CANDIDATES_H */
