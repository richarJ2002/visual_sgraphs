/*!
 * @file            findAnchorRoomPairs.cc
 *
 * @brief           Implements findAnchorRoomPairs(), declared in
 * private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

#include <algorithm>
#include <iterator>

namespace vs_graphs
{
namespace core
{
namespace semantic
{

std::vector<MergePair>
    findAnchorRoomPairs(const std::vector<Room *> &survivingRooms_in,
                        const std::vector<Room *> &absorbedRooms_in,
                        const Room                *p_survivingAnchorRoom_in,
                        const Room                *p_absorbedAnchorRoom_in)
{
    std::vector<MergePair> anchorPairs;
    if (p_survivingAnchorRoom_in == nullptr ||
        p_absorbedAnchorRoom_in == nullptr)
    {
        return anchorPairs;
    }
    const std::vector<Room *>::const_iterator survivingRoom =
        std::find(survivingRooms_in.begin(),
                  survivingRooms_in.end(),
                  p_survivingAnchorRoom_in);
    const std::vector<Room *>::const_iterator absorbedRoom =
        std::find(absorbedRooms_in.begin(),
                  absorbedRooms_in.end(),
                  p_absorbedAnchorRoom_in);
    if (survivingRoom != survivingRooms_in.end() &&
        absorbedRoom != absorbedRooms_in.end())
    {
        anchorPairs.push_back(
            {static_cast<std::size_t>(
                 std::distance(absorbedRooms_in.begin(), absorbedRoom)),
             static_cast<std::size_t>(
                 std::distance(survivingRooms_in.begin(), survivingRoom))});
    }
    return anchorPairs;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
