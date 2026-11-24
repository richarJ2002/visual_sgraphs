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
 * @file            PassageRoomLinkStateStruct.h
 *
 * @brief           Declares what associatePassagesToRooms() knows about one
 *                  passage while it links the passage to its rooms.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_PASSAGE_ROOM_LINK_STATE_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_PASSAGE_ROOM_LINK_STATE_STRUCT_H

#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "scene_understanding/scene_graph/passage/Passage.h"

namespace vs_graphs
{
namespace core
{

namespace geometric
{
class Plane;
} /* namespace geometric */

namespace semantic
{
class Room;
} /* namespace semantic */

/*!
 * @brief           Per-passage state of one associatePassagesToRooms() pass,
 *                  handed from step to step. A fresh value is made for every
 *                  passage. Room and plane pointers are borrowed from the
 *                  scene graph.
 */
struct PassageRoomLinkState
{
  public:
    /*!
     * @brief           True once readPassageLinkGeometry() found a usable,
     *                  non-proxy passage with a supporting wall and a finite
     *                  plane; the later steps run only then.
     */
    bool isPassageLinkable{false};

    /*!
     * @brief           Walls the passage is linked to.
     */
    std::vector<geometric::Plane *> supportingWalls{};

    /*!
     * @brief           Unit-normal passage plane, world frame.
     */
    Eigen::Vector4d passageEquation_world{Eigen::Vector4d::Zero()};

    /*!
     * @brief           Passage centroid, world frame, metres.
     */
    Eigen::Vector3d passageCentroid_world_m{Eigen::Vector3d::Zero()};

    /*!
     * @brief           Passage width, and how far from the passage centre a
     *                  wall still frames the opening, metres.
     */
    double passageWidth_m{0.0};
    double openingFrameReach_m{0.0};

    /*!
     * @brief           Side the passage was first seen from, refreshed when a
     *                  step sets it on the passage.
     */
    semantic::Passage::KnownSideProvenance knownSide{};

    /*!
     * @brief           Nearest room on each side of the passage plane, and the
     *                  room on each side that owns a supporting wall itself.
     */
    semantic::Room *p_negativeSideRoom{nullptr};
    semantic::Room *p_positiveSideRoom{nullptr};
    semantic::Room *p_negativeExactSupportingOwner{nullptr};
    semantic::Room *p_positiveExactSupportingOwner{nullptr};

    /*!
     * @brief           Distance from the passage centroid to the nearest room
     *                  centroid on each side, metres.
     */
    double negativeRoomDistance_m{std::numeric_limits<double>::max()};
    double positiveRoomDistance_m{std::numeric_limits<double>::max()};

    /*!
     * @brief           Rooms linked to the passage this pass: all of them,
     *                  the confirmed ones, and the last confirmed and the last
     *                  undefined (prospective) one.
     */
    std::size_t     associatedRoomCount{0U};
    std::size_t     confirmedAssociatedRoomCount{0U};
    semantic::Room *p_confirmedAssociatedRoom{nullptr};
    semantic::Room *p_undefinedAssociatedRoom{nullptr};

    /*!
     * @brief           Sign of the known-side direction along the passage
     *                  normal; 0 when no direction is known.
     */
    double knownSideSign{0.0};

    /*!
     * @brief           True when the passage held a far-side handle at the
     *                  start of the far-side step.
     */
    bool hadProspectiveHandle{false};

    /*!
     * @brief           True when the only confirmed room won the side opposite
     *                  the known side, so it is the far-side room.
     */
    bool isSoleRoomOnFarSide{false};
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_PASSAGE_ROOM_LINK_STATE_STRUCT_H */
