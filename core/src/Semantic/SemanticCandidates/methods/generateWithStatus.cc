

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

SemanticCandidatesStatus SemanticCandidates::generateWithStatus(
    const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                  &history_in,
    SemanticCandidateGeneration   &generation_out,
    const SemanticCandidateConfig &configuration_in,
    const std::optional<int>       anchorRoomId_in)
{
    SemanticCandidateGeneration            result;
    SemanticCandidateConfigRejectionReason rejectionReason2{};
    if (validateConfig(configuration_in, rejectionReason2) !=
        SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
    {
        // validateConfig cannot fail; continue as before.
    }
    result.rejectionReason = rejectionReason2;
    if (result.rejectionReason != SemanticCandidateConfigRejectionReason::NONE)
    {
        generation_out = result;
        return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
    }

    const std::size_t roomCap = configuration_in.candidatePairCap;
    std::vector<std::tuple<long unsigned int, int, std::size_t>> roomRefs;
    roomRefs.reserve(roomCap);
    for (const auto &mapEntry : history_in)
    {
        for (std::size_t index = 0U; index < mapEntry.second.size(); ++index)
        {
            const RoomContextSnapshot &snapshot = mapEntry.second[index];
            const auto                 key =
                std::make_tuple(mapEntry.first, snapshot.roomId, index);
            auto insertion =
                std::lower_bound(roomRefs.begin(),
                                 roomRefs.end(),
                                 key,
                                 [](const auto &left, const auto &right)
                                 { return left < right; });
            if (roomRefs.size() < roomCap)
            {
                roomRefs.insert(insertion, key);
            }
            else if (key < roomRefs.back())
            {
                roomRefs.insert(insertion, key);
                roomRefs.pop_back();
            }
        }
    }
    std::vector<std::pair<long unsigned int, RoomContextSnapshot>> rooms;
    rooms.reserve(roomRefs.size());
    for (const auto &reference : roomRefs)
    {
        const auto mapIt = history_in.find(std::get<0>(reference));
        rooms.emplace_back(std::get<0>(reference),
                           mapIt->second[std::get<2>(reference)]);
    }

    /* Adjacency-prioritised enumeration (rooms sharing a passage with the
     * last-confirmed room are enumerated first). The anchor and its
     * same-map passage-neighbours are necessarily all in one map (a Passage
     * only resolves a far-side room within its own live map), so priority
     * membership is used to admit a PAIR (one side qualifying is enough),
     * never to restrict both sides to the anchor's own map -- that would
     * make every priority pair same-map and therefore always skipped below.
     */
    std::set<int> priorityRoomIds;
    if (anchorRoomId_in.has_value())
    {
        const auto anchorIt =
            std::find_if(rooms.begin(),
                         rooms.end(),
                         [anchor = *anchorRoomId_in](const auto &room_in)
                         { return room_in.second.roomId == anchor; });
        if (anchorIt != rooms.end())
        {
            priorityRoomIds.insert(*anchorRoomId_in);
            for (const PassageContext &passage :
                 anchorIt->second.passageContexts)
            {
                if (passage.hasFarSideRoom)
                {
                    priorityRoomIds.insert(passage.secondaryRoomId);
                }
            }
        }
    }

    const auto scorePair =
        [&](const std::pair<long unsigned int, RoomContextSnapshot>
                &firstRoom_in,
            const std::pair<long unsigned int, RoomContextSnapshot>
                &secondRoom_in) -> std::optional<SemanticCandidate>
    {
        const RoomContextSnapshot &left  = firstRoom_in.second;
        const RoomContextSnapshot &right = secondRoom_in.second;
        const std::vector<double>  leftAngles =
            angleSignature(left,
                           configuration_in.angleTolerance_rad,
                           configuration_in.descriptorElementsCap);
        const std::vector<double> rightAngles =
            angleSignature(right,
                           configuration_in.angleTolerance_rad,
                           configuration_in.descriptorElementsCap);
        const double leftMedian =
            medianExtent(left, configuration_in.descriptorElementsCap);
        const double rightMedian =
            medianExtent(right, configuration_in.descriptorElementsCap);
        const std::vector<double> leftExtents =
            extentSignature(left,
                            leftMedian,
                            configuration_in.descriptorElementsCap);
        const std::vector<double> rightExtents =
            extentSignature(right,
                            rightMedian,
                            configuration_in.descriptorElementsCap);
        const std::vector<std::pair<double, double>> leftApertures =
            apertureSignature(left,
                              leftMedian,
                              configuration_in.descriptorElementsCap);
        const std::vector<std::pair<double, double>> rightApertures =
            apertureSignature(right,
                              rightMedian,
                              configuration_in.descriptorElementsCap);
        const std::vector<std::string> leftTopology =
            topologySignature(left,
                              configuration_in.topologyNodesCap,
                              configuration_in.topoRefinementIters);
        const std::vector<std::string> rightTopology =
            topologySignature(right,
                              configuration_in.topologyNodesCap,
                              configuration_in.topoRefinementIters);

        /* Minimum evidence: "at least 2 walls with valid normals and bounds,
         * or 1 wall + 1 passage." The "with valid normals and bounds"
         * qualifier grammatically attaches only to the 2-wall branch; a lone
         * wall in the mixed branch needs only a valid normal (its own bounds,
         * if invalid, simply omit that wall's extent element per the missing-
         * data rule -- it does not disqualify the room). */
        const bool leftWallEvidence  = validWallEvidenceCount(left) >= 2U;
        const bool rightWallEvidence = validWallEvidenceCount(right) >= 2U;
        const bool leftMixedEvidence =
            validNormalCount(left) >= 1U && !left.passageContexts.empty();
        const bool rightMixedEvidence =
            validNormalCount(right) >= 1U && !right.passageContexts.empty();
        if (!(leftWallEvidence || leftMixedEvidence) ||
            !(rightWallEvidence || rightMixedEvidence))
        {
            return std::nullopt;
        }

        CandidateCueBreakdown cues;
        cues.angleDistance = paddedMeanL1(leftAngles,
                                          rightAngles,
                                          configuration_in.angleMissingPenalty);
        if (cues.angleDistance <= configuration_in.angleTolerance_rad)
        {
            cues.angleDistance = 0.0;
        }
        cues.extentDistance =
            paddedMeanL1(leftExtents,
                         rightExtents,
                         configuration_in.extentMissingPenalty);
        cues.apertureDistance =
            pairedManhattanDistance(leftApertures,
                                    rightApertures,
                                    configuration_in.apertureMissingPenalty);
        cues.isTopologyAvailable =
            !leftTopology.empty() && !rightTopology.empty();
        cues.topologyDistance =
            cues.isTopologyAvailable
                ? stringDistance(leftTopology, rightTopology)
                : 0.0;
        const double angleWeight = !leftAngles.empty() && !rightAngles.empty()
                                       ? configuration_in.weightAngle
                                       : 0.0;
        const double extentWeight =
            !leftExtents.empty() && !rightExtents.empty()
                ? configuration_in.weightExtent
                : 0.0;
        const double apertureWeight =
            !leftApertures.empty() && !rightApertures.empty()
                ? configuration_in.weightAperture
                : 0.0;
        const double topologyWeight =
            cues.isTopologyAvailable ? configuration_in.weightTopology : 0.0;
        const double denominator =
            angleWeight + extentWeight + apertureWeight + topologyWeight;
        if (!std::isfinite(denominator) || denominator <= 0.0)
        {
            return std::nullopt;
        }

        SemanticCandidate candidate;
        candidate.mapAId       = firstRoom_in.first;
        candidate.roomAId      = left.roomId;
        candidate.mapBId       = secondRoom_in.first;
        candidate.roomBId      = right.roomId;
        cues.weightedNumerator = angleWeight * cues.angleDistance +
                                 extentWeight * cues.extentDistance +
                                 apertureWeight * cues.apertureDistance +
                                 topologyWeight * cues.topologyDistance;
        cues.weightDenominator = denominator;
        candidate.cues         = cues;
        candidate.distance     = cues.weightedNumerator / denominator;
        candidate.isMinimumEvidenceSatisfied = true;
        candidate.hasLowConfidence           = validNormalCount(left) < 2U ||
                                     validNormalCount(right) < 2U ||
                                     missingBoundsFraction(left) > 0.5 ||
                                     missingBoundsFraction(right) > 0.5;
        return candidate;
    };

    const auto isPriorityRoom = [&priorityRoomIds](const int roomId_in)
    { return priorityRoomIds.count(roomId_in) > 0U; };

    const auto enumerate =
        [&](const std::size_t cap_in,
            const bool        priorityOnly_in) -> std::vector<SemanticCandidate>
    {
        std::vector<SemanticCandidate> candidates;
        std::size_t                    consideredPairs = 0U;
        for (std::size_t first = 0U; first < rooms.size(); ++first)
        {
            for (std::size_t second = first + 1U; second < rooms.size();
                 ++second)
            {
                if (rooms[first].first == rooms[second].first)
                    continue;
                if (priorityOnly_in &&
                    !isPriorityRoom(rooms[first].second.roomId) &&
                    !isPriorityRoom(rooms[second].second.roomId))
                    continue;
                if (consideredPairs == cap_in)
                    break;
                ++consideredPairs;
                const std::optional<SemanticCandidate> candidate =
                    scorePair(rooms[first], rooms[second]);
                if (candidate)
                {
                    candidates.push_back(*candidate);
                }
            }
            if (consideredPairs == cap_in)
                break;
        }
        return candidates;
    };

    /* Try the adjacency-prioritised tier first (bounded by
     * candidate_pair_cap); a bounded global fallback (bounded separately by
     * global_fallback_cap) runs only when that tier admits zero candidates
     * passing minimum evidence. With no anchor (priorityRoomIds empty), this
     * degenerates to the single global enumeration, identical to prior
     * behaviour because global_fallback_cap <= candidate_pair_cap is already
     * enforced by validateConfig. */
    std::vector<SemanticCandidate> priorityCandidates;
    if (!priorityRoomIds.empty())
    {
        priorityCandidates = enumerate(configuration_in.candidatePairCap,
                                       /*priorityOnly_in=*/true);
    }
    result.candidates = !priorityCandidates.empty()
                            ? std::move(priorityCandidates)
                            : enumerate(configuration_in.globalFallbackCap,
                                        /*priorityOnly_in=*/false);

    std::sort(result.candidates.begin(),
              result.candidates.end(),
              [](const SemanticCandidate &left, const SemanticCandidate &right)
              {
                  return std::tie(left.distance,
                                  left.mapAId,
                                  left.roomAId,
                                  left.mapBId,
                                  left.roomBId) < std::tie(right.distance,
                                                           right.mapAId,
                                                           right.roomAId,
                                                           right.mapBId,
                                                           right.roomBId);
              });
    if (result.candidates.size() > configuration_in.topK)
        result.candidates.resize(configuration_in.topK);
    if (!result.candidates.empty())
    {
        const double threshold = result.candidates.front().distance +
                                 configuration_in.ambiguityMargin;
        for (SemanticCandidate &candidate : result.candidates)
            candidate.isAmbiguous = candidate.distance <= threshold;
    }
    /* Runtime budgets are profiling metadata only. Deliberately do not read a
     * clock here: deterministic candidate bytes must not depend on scheduling.
     */
    generation_out = result;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
