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

void Atlas::preSave()
{
    if (p_activeMap)
    {
        if (!maps.empty() &&
            lastInitKeyFrameId < p_activeMap->getMaxKeyFrameId())
            lastInitKeyFrameId =
                p_activeMap->getMaxKeyFrameId() +
                1; // The init KF is the next of current maximum
    }

    struct CompFunctor
    {
        inline bool operator()(Map *p_firstMap_in, Map *p_secondMap_in)
        {
            return p_firstMap_in->getId() < p_secondMap_in->getId();
        }
    };
    std::copy(maps.begin(), maps.end(), std::back_inserter(backupMaps));
    sort(backupMaps.begin(), backupMaps.end(), CompFunctor());

    std::set<camera_models::geometriccamera::GeometricCamera *> cameraSet(
        cameras.begin(),
        cameras.end());
    for (Map *p_map : backupMaps)
    {
        if (!p_map || p_map->isBad())
            continue;

        if (p_map->getAllKeyFrames().size() == 0)
        {
            // Empty map, erase before of save it.
            setMapBad(p_map);
            continue;
        }
        p_map->preSave(cameraSet);
    }
    removeBadMaps();
}

} // namespace core
} // namespace vs_graphs
