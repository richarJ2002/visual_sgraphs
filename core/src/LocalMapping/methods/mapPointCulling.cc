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

#include "LocalMapping.h"

namespace vs_graphs
{
namespace core
{

void LocalMapping::mapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint *>::iterator lit          = mlpRecentAddedMapPoints.begin();
    const unsigned long int    nCurrentKFid = p_currentKeyFrame->mnId;

    int nThObs;
    if (monocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    int borrar = mlpRecentAddedMapPoints.size();

    while (lit != mlpRecentAddedMapPoints.end())
    {
        MapPoint *pMP = *lit;

        if (pMP->isBad())
            lit = mlpRecentAddedMapPoints.erase(lit);
        else if (pMP->getFoundRatio() < 0.25f)
        {
            pMP->setBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if (((int)nCurrentKFid - (int)pMP->firstKeyFrameId) >= 2 &&
                 pMP->getObservationCount() <= cnThObs)
        {
            pMP->setBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if (((int)nCurrentKFid - (int)pMP->firstKeyFrameId) >= 3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            lit++;
            borrar--;
        }
    }
}

} // namespace core
} // namespace vs_graphs
