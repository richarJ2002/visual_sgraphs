

#include "Semantic/SemanticCandidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <rclcpp/logging.hpp>
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
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: validateConfig returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
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
    for (const std::pair<const unsigned long, std::vector<RoomContextSnapshot>>
             &mapEntry : history_in)
    {
        for (std::size_t index = 0U; index < mapEntry.second.size(); ++index)
        {
            const RoomContextSnapshot &snapshot = mapEntry.second[index];
            const std::tuple<unsigned long, int, unsigned long> key =
                std::make_tuple(mapEntry.first, snapshot.roomId, index);
            std::vector<std::tuple<unsigned long, int, unsigned long>>::iterator
                insertion =
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
    for (const std::tuple<unsigned long, int, unsigned long> &reference :
         roomRefs)
    {
        const std::map<unsigned long,
                       std::vector<RoomContextSnapshot>>::const_iterator mapIt =
            history_in.find(std::get<0>(reference));
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
        const std::vector<
            std::pair<unsigned long, RoomContextSnapshot>>::iterator anchorIt =
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
        std::vector<double>        leftAngles{};
        if (angleSignature(left,
                           configuration_in.angleTolerance_rad,
                           configuration_in.descriptorElementsCap,
                           leftAngles) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: angleSignature returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<double> rightAngles{};
        if (angleSignature(right,
                           configuration_in.angleTolerance_rad,
                           configuration_in.descriptorElementsCap,
                           rightAngles) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: angleSignature returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        double leftMedian{};
        if (medianExtent(left,
                         configuration_in.descriptorElementsCap,
                         leftMedian) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: medianExtent returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        double rightMedian{};
        if (medianExtent(right,
                         configuration_in.descriptorElementsCap,
                         rightMedian) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: medianExtent returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<double> leftExtents{};
        if (extentSignature(left,
                            leftMedian,
                            configuration_in.descriptorElementsCap,
                            leftExtents) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: extentSignature returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<double> rightExtents{};
        if (extentSignature(right,
                            rightMedian,
                            configuration_in.descriptorElementsCap,
                            rightExtents) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: extentSignature returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<std::pair<double, double>> leftApertures{};
        if (apertureSignature(left,
                              leftMedian,
                              configuration_in.descriptorElementsCap,
                              leftApertures) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: apertureSignature returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<std::pair<double, double>> rightApertures{};
        if (apertureSignature(right,
                              rightMedian,
                              configuration_in.descriptorElementsCap,
                              rightApertures) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: apertureSignature returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<std::string> leftTopology{};
        if (topologySignature(left,
                              configuration_in.topologyNodesCap,
                              configuration_in.topoRefinementIters,
                              leftTopology) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: topologySignature returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::vector<std::string> rightTopology{};
        if (topologySignature(right,
                              configuration_in.topologyNodesCap,
                              configuration_in.topoRefinementIters,
                              rightTopology) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: topologySignature returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        /* Minimum evidence: "at least 2 walls with valid normals and bounds,
         * or 1 wall + 1 passage." The "with valid normals and bounds"
         * qualifier grammatically attaches only to the 2-wall branch; a lone
         * wall in the mixed branch needs only a valid normal (its own bounds,
         * if invalid, simply omit that wall's extent element per the missing-
         * data rule -- it does not disqualify the room). */
        std::size_t validWallEvidenceCount2{};
        if (validWallEvidenceCount(left, validWallEvidenceCount2) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: validWallEvidenceCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool  leftWallEvidence = validWallEvidenceCount2 >= 2U;
        std::size_t validWallEvidenceCount3{};
        if (validWallEvidenceCount(right, validWallEvidenceCount3) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: validWallEvidenceCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool  rightWallEvidence = validWallEvidenceCount3 >= 2U;
        std::size_t validNormalCount2{};
        if (validNormalCount(left, validNormalCount2) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: validNormalCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool leftMixedEvidence =
            validNormalCount2 >= 1U && !left.passageContexts.empty();
        std::size_t validNormalCount3{};
        if (validNormalCount(right, validNormalCount3) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: validNormalCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        const bool rightMixedEvidence =
            validNormalCount3 >= 1U && !right.passageContexts.empty();
        if (!(leftWallEvidence || leftMixedEvidence) ||
            !(rightWallEvidence || rightMixedEvidence))
        {
            return std::nullopt;
        }

        CandidateCueBreakdown cues;
        double                distance2{};
        if (paddedMeanL1(leftAngles,
                         rightAngles,
                         configuration_in.angleMissingPenalty,
                         distance2) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: paddedMeanL1 returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        cues.angleDistance = distance2;
        if (cues.angleDistance <= configuration_in.angleTolerance_rad)
        {
            cues.angleDistance = 0.0;
        }
        double distance3{};
        if (paddedMeanL1(leftExtents,
                         rightExtents,
                         configuration_in.extentMissingPenalty,
                         distance3) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: paddedMeanL1 returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        cues.extentDistance = distance3;
        double distance4{};
        if (pairedManhattanDistance(leftApertures,
                                    rightApertures,
                                    configuration_in.apertureMissingPenalty,
                                    distance4) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: pairedManhattanDistance returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }
        cues.apertureDistance = distance4;
        cues.isTopologyAvailable =
            !leftTopology.empty() && !rightTopology.empty();
        double distance5{};
        if ((cues.isTopologyAvailable) &&
            stringDistance(leftTopology, rightTopology, distance5) !=
                SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: stringDistance returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        cues.topologyDistance    = cues.isTopologyAvailable ? distance5 : 0.0;
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
        std::size_t validNormalCount4{};
        if (validNormalCount(left, validNormalCount4) !=
            SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: validNormalCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        std::size_t validNormalCount5{};
        if (!(validNormalCount4 < 2U) &&
            validNormalCount(right, validNormalCount5) !=
                SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: validNormalCount returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        double missingBoundsFraction2{};
        if (!(validNormalCount4 < 2U || validNormalCount5 < 2U) &&
            missingBoundsFraction(left, missingBoundsFraction2) !=
                SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: missingBoundsFraction returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        double missingBoundsFraction3{};
        if (!(validNormalCount4 < 2U || validNormalCount5 < 2U ||
              missingBoundsFraction2 > 0.5) &&
            missingBoundsFraction(right, missingBoundsFraction3) !=
                SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: missingBoundsFraction returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        candidate.hasLowConfidence =
            validNormalCount4 < 2U || validNormalCount5 < 2U ||
            missingBoundsFraction2 > 0.5 || missingBoundsFraction3 > 0.5;
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
