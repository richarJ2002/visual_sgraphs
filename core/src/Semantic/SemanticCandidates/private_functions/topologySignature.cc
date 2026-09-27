

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

std::vector<std::string>
    topologySignature(const RoomContextSnapshot &snapshot_in,
                      const std::size_t          cap_in,
                      const unsigned int         refinementIters_in)
{
    const std::size_t passageCount = snapshot_in.passageContexts.size();
    /* "Absent passages omit the topology cue" -- a trivial
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

} // namespace semantic
} // namespace core
} // namespace vs_graphs
