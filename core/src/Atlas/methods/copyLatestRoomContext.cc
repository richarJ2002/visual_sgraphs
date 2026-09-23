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

namespace vs_graphs
{
namespace core
{

std::optional<semantic::RoomContextSnapshot>
    Atlas::copyLatestRoomContext(const int roomId_in) const
{
    std::lock_guard<std::mutex>                  contextLock(mRoomContextMutex);
    std::optional<semantic::RoomContextSnapshot> latestSnapshot;

    for (const std::pair<const long unsigned int,
                         std::vector<semantic::RoomContextSnapshot>>
             &historyEntry : roomContextHistory)
    {
        static_cast<void>(historyEntry.first);
        for (const semantic::RoomContextSnapshot &snapshot :
             historyEntry.second)
        {
            if (snapshot.roomId != roomId_in ||
                (latestSnapshot.has_value() &&
                 snapshot.timestamp <= latestSnapshot->timestamp))
            {
                continue;
            }
            latestSnapshot = snapshot;
        }
    }

    return latestSnapshot;
}

} // namespace core
} // namespace vs_graphs
