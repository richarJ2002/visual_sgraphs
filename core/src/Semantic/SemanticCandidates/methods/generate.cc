

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

std::vector<SemanticCandidate> SemanticCandidates::generate(
    const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                  &history_in,
    const SemanticCandidateConfig &config_in,
    const std::optional<int>       anchorRoomId_in)
{
    return generateWithStatus(history_in, config_in, anchorRoomId_in)
        .candidates;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
