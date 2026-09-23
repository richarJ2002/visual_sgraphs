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

#include "LoopClosing.h"

namespace vs_graphs
{
namespace core
{

void LoopClosing::checkObservations(set<KeyFrame *> &spKFsMap1,
                                    set<KeyFrame *> &spKFsMap2)
{
    cout << "----------------------" << endl;
    for (KeyFrame *pKFi1 : spKFsMap1)
    {
        map<KeyFrame *, int> matchedMapPointCounts;
        set<MapPoint *>      spMPs = pKFi1->getMapPoints();

        for (MapPoint *pMPij : spMPs)
        {
            if (!pMPij || pMPij->isBad())
            {
                continue;
            }

            map<KeyFrame *, tuple<int, int>> mapPointObservations =
                pMPij->getObservations();
            for (KeyFrame *pKFi2 : spKFsMap2)
            {
                if (mapPointObservations.find(pKFi2) !=
                    mapPointObservations.end())
                {
                    if (matchedMapPointCounts.find(pKFi2) !=
                        matchedMapPointCounts.end())
                    {
                        matchedMapPointCounts[pKFi2] =
                            matchedMapPointCounts[pKFi2] + 1;
                    }
                    else
                    {
                        matchedMapPointCounts[pKFi2] = 1;
                    }
                }
            }
        }

        if (matchedMapPointCounts.size() == 0)
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId
                 << " has not any matched MP with the other map" << endl;
        }
        else
        {
            cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with "
                 << matchedMapPointCounts.size() << " KF from the other map"
                 << endl;
            for (pair<KeyFrame *, int> matchedKF : matchedMapPointCounts)
            {
                cout << "   -KF: " << matchedKF.first->mnId
                     << ", Number of matches: " << matchedKF.second << endl;
            }
        }
    }
    cout << "----------------------" << endl;
}

} // namespace core
} // namespace vs_graphs
