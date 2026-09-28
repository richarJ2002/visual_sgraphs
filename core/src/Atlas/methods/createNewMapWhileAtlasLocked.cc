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

void Atlas::createNewMapWhileAtlasLocked()
{
    std::cout << "\n[Atlas]" << std::endl;
    std::cout << "- Creating a new map (MapId: " << Map::nextId
              << ", Init KeyFrame: " << lastInitKeyFrameId << ") ..."
              << std::endl;

    if (p_activeMap)
    {
        if (!maps.empty() &&
            lastInitKeyFrameId < p_activeMap->getMaxKeyFrameId())
            lastInitKeyFrameId = p_activeMap->getMaxKeyFrameId() + 1;

        /* Snapshot room geometry before the map is stranded so that
         * rooms in the new map can inherit identity tags.            */
        exportRoomContextFromCurrentMap();

        p_activeMap->setStoredMap();
        std::cout << "- The created map with MapId #" << p_activeMap->getId()
                  << " has been stored!" << std::endl;
    }

    Map *p_previousMap = p_activeMap;
    p_activeMap        = new Map(lastInitKeyFrameId);
    p_activeMap->setCurrentMap();
    maps.insert(p_activeMap);
    if (p_previousMap != nullptr)
    {
        /* Mission-chain trace link: the stranded map points at its
         * successor. Same-map clears never pass through here, so the link
         * stays null for them by construction. */
        p_previousMap->setFollowingMap(p_activeMap);
    }
    {
        std::lock_guard<std::mutex> contextLock(roomContextMutex);
        isNewMapCreatedPending = true;
    }
}

} // namespace core
} // namespace vs_graphs
