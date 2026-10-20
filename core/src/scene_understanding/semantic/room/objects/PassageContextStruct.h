/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors:  Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 *              and Holger Voos
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
 * @file            PassageContextStruct.h
 *
 * @brief           Declares PassageContext, a saved description of one passage
 * of a room.
 */

#ifndef VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_PASSAGE_CONTEXT_STRUCT_H
#define VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_PASSAGE_CONTEXT_STRUCT_H

#include "scene_understanding/semantic/room/objects/WallBoundsStruct.h"

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace vs_graphs
{
namespace core
{
namespace semantic
{



/*!
 * @brief           Saved description of one passage (door or opening) of a
 *                  room: its aperture, which rooms it joins and how often it
 *                  was walked through.
 */
struct PassageContext
{
    /*!
     * @brief           Passage identifier; stays the same for the same
     *                  doorway across maps.
     */
    int             id{0};
    /*!
     * @brief           True when the passage was marked passable (the UAV can
     *                  fly through it).
     */
    bool            isPassable{false};
    /*!
     * @brief           True when the passage had a far-side room recorded;
     *                  secondaryRoomId is meaningful only then.
     */
    bool            hasFarSideRoom{false};
    /*!
     * @brief           Id of the room on the far side of the passage, valid
     *                  when hasFarSideRoom is true.
     */
    int             secondaryRoomId{0};
    /*!
     * @brief           True when width_m and height_m are both finite and
     *                  positive.
     */
    bool            isApertureValid{false};
    /*!
     * @brief           Passage opening width, metres.
     */
    double          width_m{0.0};
    /*!
     * @brief           Passage opening height, metres.
     */
    double          height_m{0.0};
    /*!
     * @brief           Centre of the passage in the world frame, metres.
     */
    Eigen::Vector3d passageCentroid_world{Eigen::Vector3d::Zero()};
    /*!
     * @brief           True when knownSideDirection_world holds a finite unit
     *                  direction.
     */
    bool            hasKnownSideDirection{false};
    /*!
     * @brief           Unit direction in the world frame from the passage
     *                  toward the side the UAV observed it from; valid when
     *                  hasKnownSideDirection is true.
     */
    Eigen::Vector3d knownSideDirection_world{Eigen::Vector3d::Zero()};
    /*!
     * @brief           True when the observing-side room was recorded;
     *                  knownSideRoomId is meaningful only then.
     */
    bool            hasKnownSideRoom{false};
    /*!
     * @brief           Id of the room on the side the passage was observed
     *                  from; -1 when not recorded.
     */
    int             knownSideRoomId{-1};
    /*!
     * @brief           Number of recorded crossings from the known side to the
     *                  far side.
     */
    std::size_t     traversalKnownToFarCount{0U};
    /*!
     * @brief           Number of recorded crossings from the far side back to
     *                  the known side.
     */
    std::size_t     traversalFarToKnownCount{0U};
    /*!
     * @brief           Number of recorded crossings whose direction could not
     *                  be determined.
     */
    std::size_t     traversalUnknownCount{0U};
    /*!
     * @brief           Number of walls associated with this passage.
     */
    std::size_t     associatedWallCount{0U};
    /*!
     * @brief           True when the passage was crossed in both directions.
     */
    bool            hasBidirectionalTraversalEvidence{false};
    /*!
     * @brief           True when the passage is historical recovery topology
     *                  only, not a freshly observed one.
     */
    bool            isRecoveryProxy{false};
};

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_SEMANTIC_ROOM_OBJECTS_PASSAGE_CONTEXT_STRUCT_H */
