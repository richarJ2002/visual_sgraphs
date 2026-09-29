

#include "Semantic/SemanticCandidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <rclcpp/logging.hpp>
#include <set>
#include <string>
#include <tuple>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticCandidatesStatus SemanticCandidates::generate(
    const std::map<long unsigned int, std::vector<RoomContextSnapshot>>
                                   &history_in,
    std::vector<SemanticCandidate> &candidates_out,
    const SemanticCandidateConfig  &configuration_in,
    const std::optional<int>        anchorRoomId_in)
{
    SemanticCandidateGeneration generation{};
    if (generateWithStatus(history_in,
                           generation,
                           configuration_in,
                           anchorRoomId_in) !=
        SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: generateWithStatus returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    candidates_out = generation.candidates;
    return SemanticCandidatesStatus::SEMANTIC_CANDIDATES_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
