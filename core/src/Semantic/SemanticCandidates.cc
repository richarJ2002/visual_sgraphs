#include "Semantic/SemanticCandidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <tuple>

namespace ORB_SLAM3
{
namespace
{
bool finiteNonnegative(const double value_in)
{
    return std::isfinite(value_in) && value_in >= 0.0;
}

double paddedMeanL1(const std::vector<double> &left_in,
                    const std::vector<double> &right_in,
                    const double               penalty_in)
{
    const std::size_t length = std::max(left_in.size(), right_in.size());
    if (length == 0U)
    {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t index = 0U; index < length; ++index)
    {
        const double left =
            index < left_in.size() ? left_in[index] : penalty_in;
        const double right =
            index < right_in.size() ? right_in[index] : penalty_in;
        sum += std::abs(left - right);
    }
    return sum / static_cast<double>(length);
}

std::vector<double> angleSignature(const RoomContextSnapshot &snapshot_in,
                                   const double               tolerance_in,
                                   const std::size_t          cap_in)
{
    std::vector<Eigen::Vector3d> normals;
    normals.reserve(std::min(snapshot_in.wallNormals.size(), cap_in));
    for (const Eigen::Vector3d &normal : snapshot_in.wallNormals)
    {
        if (normals.size() == cap_in)
        {
            break;
        }
        if (normal.allFinite() && normal.norm() > 1e-12)
        {
            normals.push_back(normal.normalized());
        }
    }
    std::vector<double> signature;
    const std::size_t   pairCap =
        std::min(cap_in, normals.size() * (normals.size() - 1U) / 2U);
    signature.reserve(pairCap);
    for (std::size_t first = 0U; first < normals.size(); ++first)
    {
        for (std::size_t second = first + 1U; second < normals.size(); ++second)
        {
            if (signature.size() == cap_in)
            {
                break;
            }
            const double dot =
                std::clamp(std::abs(normals[first].dot(normals[second])),
                           0.0,
                           1.0);
            const double angle = std::acos(dot);
            signature.push_back(angle <= tolerance_in ? 0.0 : angle);
        }
        if (signature.size() == cap_in)
        {
            break;
        }
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

std::size_t validNormalCount(const RoomContextSnapshot &snapshot_in)
{
    std::size_t count = 0U;
    for (const Eigen::Vector3d &normal : snapshot_in.wallNormals)
    {
        if (normal.allFinite() && normal.norm() > 1e-12)
        {
            ++count;
        }
    }
    return count;
}

bool isValidWallBounds(const WallBounds &bounds_in)
{
    return bounds_in.valid && std::isfinite(bounds_in.minU_m) &&
           std::isfinite(bounds_in.maxU_m) && std::isfinite(bounds_in.minV_m) &&
           std::isfinite(bounds_in.maxV_m) &&
           bounds_in.maxU_m > bounds_in.minU_m &&
           bounds_in.maxV_m > bounds_in.minV_m;
}

/** Counts walls whose SAME index has both a valid finite unit-able normal and
 * valid bounds (Section 9.2: "walls with valid normals and bounds"). */
std::size_t validWallEvidenceCount(const RoomContextSnapshot &snapshot_in)
{
    const std::size_t pairedCount =
        std::min(snapshot_in.wallNormals.size(), snapshot_in.wallBounds.size());
    std::size_t count = 0U;
    for (std::size_t index = 0U; index < pairedCount; ++index)
    {
        const Eigen::Vector3d &normal = snapshot_in.wallNormals[index];
        if (normal.allFinite() && normal.norm() > 1e-12 &&
            isValidWallBounds(snapshot_in.wallBounds[index]))
        {
            ++count;
        }
    }
    return count;
}

/** Fraction of wallBounds entries that are invalid; 0.0 when there are no
 * walls to be missing from (Section 9.2's ">50% missing bounds" guard). */
double missingBoundsFraction(const RoomContextSnapshot &snapshot_in)
{
    if (snapshot_in.wallBounds.empty())
    {
        return 0.0;
    }
    std::size_t validCount = 0U;
    for (const WallBounds &bounds : snapshot_in.wallBounds)
    {
        if (isValidWallBounds(bounds))
        {
            ++validCount;
        }
    }
    return 1.0 - static_cast<double>(validCount) /
                     static_cast<double>(snapshot_in.wallBounds.size());
}

double medianExtent(const RoomContextSnapshot &snapshot_in,
                    const std::size_t          cap_in)
{
    std::vector<double> spans;
    spans.reserve(std::min(snapshot_in.wallBounds.size(), cap_in));
    for (const WallBounds &bounds : snapshot_in.wallBounds)
    {
        if (spans.size() + 1U >= cap_in)
        {
            break;
        }
        if (isValidWallBounds(bounds))
        {
            spans.push_back(bounds.maxU_m - bounds.minU_m);
            if (spans.size() < cap_in)
            {
                spans.push_back(bounds.maxV_m - bounds.minV_m);
            }
        }
    }
    if (spans.empty())
    {
        return 0.0;
    }
    std::sort(spans.begin(), spans.end());
    const std::size_t middle = spans.size() / 2U;
    return spans.size() % 2U == 0U ? (spans[middle - 1U] + spans[middle]) / 2.0
                                   : spans[middle];
}

std::vector<double> extentSignature(const RoomContextSnapshot &snapshot_in,
                                    const double               median_in,
                                    const std::size_t          cap_in)
{
    std::vector<double> signature;
    if (!std::isfinite(median_in) || median_in <= 0.0)
    {
        return signature;
    }
    signature.reserve(std::min(snapshot_in.wallBounds.size(), cap_in));
    for (const WallBounds &bounds : snapshot_in.wallBounds)
    {
        if (signature.size() == cap_in)
        {
            break;
        }
        if (isValidWallBounds(bounds))
        {
            signature.push_back((bounds.maxU_m - bounds.minU_m) / median_in);
            if (signature.size() < cap_in)
            {
                signature.push_back((bounds.maxV_m - bounds.minV_m) /
                                    median_in);
            }
        }
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

/** One (width,height) aperture pair, normalised by the room's valid median
 * extent. Kept paired (not flattened) so lexicographic sort and pairwise
 * Manhattan distance compare a passage's own width against its own height,
 * per Section 19.4 point 2(c)/3. */
std::vector<std::pair<double, double>>
    apertureSignature(const RoomContextSnapshot &snapshot_in,
                      const double               median_in,
                      const std::size_t          cap_in)
{
    std::vector<std::pair<double, double>> signature;
    if (!std::isfinite(median_in) || median_in <= 0.0)
    {
        return signature;
    }
    signature.reserve(std::min(snapshot_in.passageContexts.size(), cap_in));
    for (const PassageContext &passage : snapshot_in.passageContexts)
    {
        if (signature.size() == cap_in)
        {
            break;
        }
        if (passage.apertureValid && finiteNonnegative(passage.width_m) &&
            finiteNonnegative(passage.height_m) && passage.width_m > 0.0 &&
            passage.height_m > 0.0)
        {
            signature.emplace_back(passage.width_m / median_in,
                                   passage.height_m / median_in);
        }
    }
    std::sort(signature.begin(), signature.end());
    return signature;
}

/** Pads the shorter list of aperture pairs with (penalty,penalty), then sums
 * the pairwise Manhattan error |Δwidth|+|Δheight| divided by the longer
 * length. Mirrors paddedMeanL1's padding rule, generalised to 2D pairs. */
double pairedManhattanDistance(
    const std::vector<std::pair<double, double>> &left_in,
    const std::vector<std::pair<double, double>> &right_in,
    const double                                  penalty_in)
{
    const std::size_t length = std::max(left_in.size(), right_in.size());
    if (length == 0U)
    {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t index = 0U; index < length; ++index)
    {
        const std::pair<double, double> left =
            index < left_in.size() ? left_in[index]
                                   : std::make_pair(penalty_in, penalty_in);
        const std::pair<double, double> right =
            index < right_in.size() ? right_in[index]
                                    : std::make_pair(penalty_in, penalty_in);
        sum += std::abs(left.first - right.first) +
               std::abs(left.second - right.second);
    }
    return sum / static_cast<double>(length);
}

struct TopologyNode
{
    std::string              label;
    std::vector<std::size_t> neighbours;
};

std::vector<std::string>
    topologySignature(const RoomContextSnapshot &snapshot_in,
                      const std::size_t          cap_in,
                      const unsigned int         refinementIters_in)
{
    const std::size_t passageCount = snapshot_in.passageContexts.size();
    /* Section 9.2: "absent passages omit the topology... cue" -- a trivial
     * single-node (room-only) graph is not usable topology evidence. */
    if (passageCount == 0U || passageCount >= cap_in)
    {
        return {};
    }
    std::vector<int> farIds;
    farIds.reserve(std::min(passageCount, cap_in - passageCount - 1U));
    for (const PassageContext &passage : snapshot_in.passageContexts)
    {
        if (passage.hasFarSideRoom &&
            std::find(farIds.begin(), farIds.end(), passage.secondaryRoomId) ==
                farIds.end())
        {
            if (1U + passageCount + farIds.size() == cap_in)
            {
                return {};
            }
            farIds.push_back(passage.secondaryRoomId);
        }
    }
    const std::size_t nodeCount = 1U + passageCount + farIds.size();
    if (nodeCount > cap_in)
    {
        return {};
    }

    std::vector<TopologyNode> graph(nodeCount);
    graph[0U].label = "room";
    for (std::size_t index = 0U; index < passageCount; ++index)
    {
        const PassageContext &passage = snapshot_in.passageContexts[index];
        const std::size_t     node    = index + 1U;
        graph[node].label =
            std::string("passage:") + (passage.passable ? "1" : "0") + ":" +
            (passage.hasKnownSideDirection ? "1" : "0") + ":" +
            std::to_string(std::min(passage.traversalKnownToFarCount,
                                    passage.traversalFarToKnownCount)) +
            ":" +
            std::to_string(std::max(passage.traversalKnownToFarCount,
                                    passage.traversalFarToKnownCount)) +
            ":" + std::to_string(passage.traversalUnknownCount) + ":" +
            std::to_string(passage.associatedWallCount) + ":" +
            std::to_string(passage.hasBidirectionalTraversalEvidence) + ":" +
            (passage.hasFarSideRoom ? "far=1" : "far=0");
        graph[0U].neighbours.push_back(node);
        graph[node].neighbours.push_back(0U);
        if (passage.hasFarSideRoom)
        {
            const std::size_t farIndex =
                static_cast<std::size_t>(std::find(farIds.begin(),
                                                   farIds.end(),
                                                   passage.secondaryRoomId) -
                                         farIds.begin());
            const std::size_t farNode = passageCount + 1U + farIndex;
            graph[farNode].label      = "far_room";
            graph[node].neighbours.push_back(farNode);
            graph[farNode].neighbours.push_back(node);
        }
    }
    graph[0U].label += ":degree=" + std::to_string(graph[0U].neighbours.size());
    for (std::size_t index = passageCount + 1U; index < nodeCount; ++index)
    {
        graph[index].label +=
            ":degree=" + std::to_string(graph[index].neighbours.size());
    }

    std::vector<std::string> colors;
    colors.reserve(nodeCount);
    for (const TopologyNode &node : graph)
    {
        colors.push_back(node.label);
    }
    for (unsigned int iteration = 0U; iteration < refinementIters_in;
         ++iteration)
    {
        std::vector<std::string> refined;
        refined.reserve(nodeCount);
        for (std::size_t nodeIndex = 0U; nodeIndex < nodeCount; ++nodeIndex)
        {
            std::vector<std::string> neighbours;
            neighbours.reserve(graph[nodeIndex].neighbours.size());
            for (const std::size_t neighbour : graph[nodeIndex].neighbours)
            {
                neighbours.push_back(colors[neighbour]);
            }
            std::sort(neighbours.begin(), neighbours.end());
            std::string value = colors[nodeIndex];
            for (const std::string &neighbour : neighbours)
            {
                value += "|" + neighbour;
            }
            refined.push_back(std::move(value));
        }
        colors = std::move(refined);
    }
    std::sort(colors.begin(), colors.end());
    return colors;
}

double stringDistance(const std::vector<std::string> &left_in,
                      const std::vector<std::string> &right_in)
{
    const std::size_t length = std::max(left_in.size(), right_in.size());
    if (length == 0U)
    {
        return 0.0;
    }
    std::size_t mismatches = 0U;
    for (std::size_t index = 0U; index < length; ++index)
    {
        if (index >= left_in.size() || index >= right_in.size() ||
            left_in[index] != right_in[index])
        {
            ++mismatches;
        }
    }
    return static_cast<double>(mismatches) / static_cast<double>(length);
}
} // namespace

SemanticCandidateConfigRejectionReason
    SemanticCandidates::validateConfig(const SemanticCandidateConfig &config_in)
{
    if (config_in.topK == 0U)
        return SemanticCandidateConfigRejectionReason::TOP_K_ZERO;
    if (config_in.candidatePairCap == 0U)
        return SemanticCandidateConfigRejectionReason::PAIR_CAP_ZERO;
    if (config_in.topologyNodesCap == 0U)
        return SemanticCandidateConfigRejectionReason::TOPOLOGY_CAP_ZERO;
    if (config_in.globalFallbackCap == 0U)
        return SemanticCandidateConfigRejectionReason::GLOBAL_FALLBACK_CAP_ZERO;
    if (config_in.descriptorElementsCap == 0U)
        return SemanticCandidateConfigRejectionReason::DESCRIPTOR_CAP_ZERO;
    if (config_in.topoRefinementIters == 0U)
        return SemanticCandidateConfigRejectionReason::
            TOPO_REFINEMENT_ITERS_ZERO;
    if (config_in.topK > config_in.candidatePairCap)
        return SemanticCandidateConfigRejectionReason::TOP_K_EXCEEDS_PAIR_CAP;
    if (config_in.globalFallbackCap > config_in.candidatePairCap)
        return SemanticCandidateConfigRejectionReason::
            GLOBAL_FALLBACK_EXCEEDS_PAIR_CAP;
    const double weights[] = {config_in.weightAngle,
                              config_in.weightExtent,
                              config_in.weightAperture,
                              config_in.weightTopology};
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
    const double penalties[] = {config_in.angleMissingPenalty,
                                config_in.extentMissingPenalty,
                                config_in.apertureMissingPenalty};
    for (const double penalty : penalties)
    {
        if (!std::isfinite(penalty))
            return SemanticCandidateConfigRejectionReason::NONFINITE_PENALTY;
        if (penalty < 0.0)
            return SemanticCandidateConfigRejectionReason::NEGATIVE_PENALTY;
    }
    if (!std::isfinite(config_in.ambiguityMargin))
        return SemanticCandidateConfigRejectionReason::
            NONFINITE_AMBIGUITY_MARGIN;
    if (config_in.ambiguityMargin < 0.0)
        return SemanticCandidateConfigRejectionReason::
            NEGATIVE_AMBIGUITY_MARGIN;
    if (!std::isfinite(config_in.angleTolerance_rad))
        return SemanticCandidateConfigRejectionReason::
            NONFINITE_ANGLE_TOLERANCE;
    if (config_in.angleTolerance_rad < 0.0)
        return SemanticCandidateConfigRejectionReason::NEGATIVE_ANGLE_TOLERANCE;
    if (!std::isfinite(config_in.runtimeBudget_ms))
        return SemanticCandidateConfigRejectionReason::NONFINITE_RUNTIME_BUDGET;
    if (config_in.runtimeBudget_ms < 0.0)
        return SemanticCandidateConfigRejectionReason::NEGATIVE_RUNTIME_BUDGET;
    return SemanticCandidateConfigRejectionReason::NONE;
}

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

SemanticCandidateGeneration SemanticCandidates::generateWithStatus(
    const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                  &history_in,
    const SemanticCandidateConfig &config_in,
    const std::optional<int>       anchorRoomId_in)
{
    SemanticCandidateGeneration result;
    result.rejectionReason = validateConfig(config_in);
    if (result.rejectionReason != SemanticCandidateConfigRejectionReason::NONE)
    {
        return result;
    }

    const std::size_t roomCap = config_in.candidatePairCap;
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

    /* Section 9.2: "adjacency-prioritised (rooms sharing a passage with the
     * last-confirmed room are enumerated first)". The anchor and its
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
                           config_in.angleTolerance_rad,
                           config_in.descriptorElementsCap);
        const std::vector<double> rightAngles =
            angleSignature(right,
                           config_in.angleTolerance_rad,
                           config_in.descriptorElementsCap);
        const double leftMedian =
            medianExtent(left, config_in.descriptorElementsCap);
        const double rightMedian =
            medianExtent(right, config_in.descriptorElementsCap);
        const std::vector<double> leftExtents =
            extentSignature(left, leftMedian, config_in.descriptorElementsCap);
        const std::vector<double> rightExtents =
            extentSignature(right,
                            rightMedian,
                            config_in.descriptorElementsCap);
        const std::vector<std::pair<double, double>> leftApertures =
            apertureSignature(left,
                              leftMedian,
                              config_in.descriptorElementsCap);
        const std::vector<std::pair<double, double>> rightApertures =
            apertureSignature(right,
                              rightMedian,
                              config_in.descriptorElementsCap);
        const std::vector<std::string> leftTopology =
            topologySignature(left,
                              config_in.topologyNodesCap,
                              config_in.topoRefinementIters);
        const std::vector<std::string> rightTopology =
            topologySignature(right,
                              config_in.topologyNodesCap,
                              config_in.topoRefinementIters);

        /* Section 9.2: "at least 2 walls with valid normals and bounds, or
         * 1 wall + 1 passage." The "with valid normals and bounds" qualifier
         * grammatically attaches only to the 2-wall branch; a lone wall in
         * the mixed branch needs only a valid normal (its own bounds, if
         * invalid, simply omit that wall's extent element per the missing-
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
                                          config_in.angleMissingPenalty);
        if (cues.angleDistance <= config_in.angleTolerance_rad)
        {
            cues.angleDistance = 0.0;
        }
        cues.extentDistance = paddedMeanL1(leftExtents,
                                           rightExtents,
                                           config_in.extentMissingPenalty);
        cues.apertureDistance =
            pairedManhattanDistance(leftApertures,
                                    rightApertures,
                                    config_in.apertureMissingPenalty);
        cues.topologyAvailable =
            !leftTopology.empty() && !rightTopology.empty();
        cues.topologyDistance =
            cues.topologyAvailable ? stringDistance(leftTopology, rightTopology)
                                   : 0.0;
        const double angleWeight = !leftAngles.empty() && !rightAngles.empty()
                                       ? config_in.weightAngle
                                       : 0.0;
        const double extentWeight =
            !leftExtents.empty() && !rightExtents.empty()
                ? config_in.weightExtent
                : 0.0;
        const double apertureWeight =
            !leftApertures.empty() && !rightApertures.empty()
                ? config_in.weightAperture
                : 0.0;
        const double topologyWeight =
            cues.topologyAvailable ? config_in.weightTopology : 0.0;
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
        candidate.minimumEvidenceSatisfied = true;
        candidate.lowConfidence            = validNormalCount(left) < 2U ||
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

    /* Section 9.2: try the adjacency-prioritised tier first (bounded by
     * candidate_pair_cap); a bounded global fallback (bounded separately by
     * global_fallback_cap) runs only when that tier admits zero candidates
     * passing minimum evidence. With no anchor (priorityRoomIds empty), this
     * degenerates to the single global enumeration, identical to prior
     * behaviour because global_fallback_cap <= candidate_pair_cap is already
     * enforced by validateConfig. */
    std::vector<SemanticCandidate> priorityCandidates;
    if (!priorityRoomIds.empty())
    {
        priorityCandidates =
            enumerate(config_in.candidatePairCap, /*priorityOnly_in=*/true);
    }
    result.candidates = !priorityCandidates.empty()
                            ? std::move(priorityCandidates)
                            : enumerate(config_in.globalFallbackCap,
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
    if (result.candidates.size() > config_in.topK)
        result.candidates.resize(config_in.topK);
    if (!result.candidates.empty())
    {
        const double threshold =
            result.candidates.front().distance + config_in.ambiguityMargin;
        for (SemanticCandidate &candidate : result.candidates)
            candidate.ambiguous = candidate.distance <= threshold;
    }
    /* Runtime budgets are profiling metadata only. Deliberately do not read a
     * clock here: deterministic candidate bytes must not depend on scheduling.
     */
    return result;
}

std::vector<SemanticCandidate> SemanticCandidates::generate(
    const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                  &history_in,
    const SemanticCandidateConfig &config_in,
    const std::optional<int>       anchorRoomId_in)
{
    return generateWithStatus(history_in, config_in, anchorRoomId_in)
        .candidates;
}
} // namespace ORB_SLAM3
