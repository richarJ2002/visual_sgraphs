/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "Atlas.h"

#include "private_functions.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Room-prior seed: the old final room and the new starting
 *               room must carry the same non-empty tag. A silent mismatch
 *               means no prior link (e.g. loop closure between
 *               non-consecutive maps): not this path's job.
 */
bool consecutiveSeedTagsMatch(Map *p_oldMap_in, Map *p_currentMap_in)
{
    if (p_oldMap_in == nullptr || p_currentMap_in == nullptr)
    {
        return false;
    }
    semantic::Room *p_oldFinalRoom = p_oldMap_in->getFinalRoom();
    semantic::Room *p_newStartRoom = p_currentMap_in->getStartingRoom();
    return p_oldFinalRoom != nullptr && p_newStartRoom != nullptr &&
           p_oldFinalRoom->hasRoomTag() && p_newStartRoom->hasRoomTag() &&
           !p_oldFinalRoom->getRoomTag().empty() &&
           p_oldFinalRoom->getRoomTag() == p_newStartRoom->getRoomTag();
}

} // namespace core
} // namespace vs_graphs
