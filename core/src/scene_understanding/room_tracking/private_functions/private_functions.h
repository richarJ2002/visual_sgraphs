/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            private_functions.h
 *
 * @brief           Declares the helpers of the room tracking stage.
 *                  Package-internal: not included by any public header.
 */

#ifndef VS_GRAPHS_CORE_SRC_ROOM_TRACKING_PRIVATE_FUNCTIONS_H
#define VS_GRAPHS_CORE_SRC_ROOM_TRACKING_PRIVATE_FUNCTIONS_H

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "scene_understanding/scene_graph/passage/objects/PassageClass.h"
#include "scene_understanding/semantics_manager/objects/SemanticsManagerStatusEnum.h"

namespace vs_graphs
{
namespace core
{

namespace keyframe
{
class KeyFrame;
} // namespace keyframe

namespace map
{
class Map;
} // namespace map

/*!
 * @brief           Most recent live key frames the traversal replay walks each
 *                  cycle; the key frame room links are written only inside
 *                  this window.
 */
inline constexpr std::size_t MAXIMUM_TRAVERSAL_HISTORY_KEYFRAMES = 128U;

/*!
 * @brief           Returns the replay window of a map: its live key frames in
 *                  isKeyFrameEarlier order, the last
 *                  MAXIMUM_TRAVERSAL_HISTORY_KEYFRAMES of them.
 *
 * @param[in]       p_map_in
 *                  Map whose key frames are read; borrowed, may be null
 *                  (empty window).
 *
 * @param[out]      windowKeyFrames_out
 *                  Window key frames, oldest first; borrowed.
 */
void collectReplayWindowKeyFrames(
    map::Map                          *p_map_in,
    std::vector<keyframe::KeyFrame *> &windowKeyFrames_out);

/*!
 * @brief           Strict weak order of key frames by frame id, then key
 *                  frame id; used to replay the camera path in order.
 *
 * @param[in]       p_firstKeyFrame_in
 *                  First key frame; borrowed, not null.
 *
 * @param[in]       p_secondKeyFrame_in
 *                  Second key frame; borrowed, not null.
 *
 * @return          True when the first key frame comes before the second.
 */
[[nodiscard]] bool
    isKeyFrameEarlier(const keyframe::KeyFrame *p_firstKeyFrame_in,
                      const keyframe::KeyFrame *p_secondKeyFrame_in);

/*!
 * @brief           Reads a crossed passage's known side, records it from the
 *                  crossing start when the passage has none yet, and derives
 *                  the crossing direction from it.
 *
 * @param[in,out]   p_passage_inout
 *                  Crossed passage; its known-side direction may be set.
 *                  Borrowed.
 *
 * @param[in]       crossingStart_world_m_in
 *                  Start of the crossing segment in world coordinates, in
 *                  metres.
 *
 * @param[in]       minimumSideDistance_m_in
 *                  Smallest distance from the passage plane that resolves a
 *                  side, in metres.
 *
 * @param[out]      knownSide_out
 *                  Known-side provenance of the passage after this call.
 *
 * @param[out]      traversalDirection_out
 *                  KNOWN_TO_FAR or FAR_TO_KNOWN; UNKNOWN without a known
 *                  side.
 *
 * @return          SEMANTICS_MANAGER_STATUS_SUCCESS.
 */
[[nodiscard]] SemanticsManagerStatus resolveTraversalDirection(
    semantic::Passage                      *p_passage_inout,
    const Eigen::Vector3d                  &crossingStart_world_m_in,
    double                                  minimumSideDistance_m_in,
    semantic::Passage::KnownSideProvenance &knownSide_out,
    semantic::Passage::TraversalDirection  &traversalDirection_out);

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_ROOM_TRACKING_PRIVATE_FUNCTIONS_H */
