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
 * @file            FarSideProspectiveMatchStruct.h
 *
 * @brief           Declares the prospective room a free-space cluster matched
 *                  on the far side of a passage.
 */

#ifndef VS_GRAPHS_SEMANTICS_MANAGER_FAR_SIDE_PROSPECTIVE_MATCH_STRUCT_H
#define VS_GRAPHS_SEMANTICS_MANAGER_FAR_SIDE_PROSPECTIVE_MATCH_STRUCT_H

#include <vector>

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
class Passage;
class Room;
} /* namespace semantic */

/*!
 * @brief           Result of findFarSideProspectiveRoom() for one free-space
 *                  cluster. Room, passage and plane pointers are borrowed from
 *                  the scene graph.
 */
struct FarSideProspectiveMatch
{
  public:
    /*!
     * @brief           Undefined prospective room the cluster lies in front
     *                  of, or nullptr when no passage handle matched.
     */
    semantic::Room *p_prospectiveRoom{nullptr};

    /*!
     * @brief           Passage whose prospective room matched, or nullptr.
     */
    semantic::Passage *p_passage{nullptr};

    /*!
     * @brief           Walls of the matched prospective room taken before the
     *                  cluster admitted any wall, so admission by the cluster
     *                  cannot manufacture its own promotion evidence.
     */
    std::vector<geometric::Plane *> prospectiveWallsBeforeCluster{};
};

} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_SEMANTICS_MANAGER_FAR_SIDE_PROSPECTIVE_MATCH_STRUCT_H */
