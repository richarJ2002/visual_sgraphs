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

void LoopClosing::checkObservations(set<KeyFrame *> &keyFramesMap1_in,
                                    set<KeyFrame *> &keyFramesMap2_in)
{
    cout << "----------------------" << endl;
    for (KeyFrame *p_keyFrameInMap1 : keyFramesMap1_in)
    {
        map<KeyFrame *, int> matchedMapPointCounts;
        set<MapPoint *>      mapPoints = p_keyFrameInMap1->getMapPoints();

        for (MapPoint *p_sharedMapPoint : mapPoints)
        {
            if (!p_sharedMapPoint || p_sharedMapPoint->isBad())
            {
                continue;
            }

            map<KeyFrame *, tuple<int, int>> mapPointObservations =
                p_sharedMapPoint->getObservations();
            for (KeyFrame *p_keyFrameInMap2 : keyFramesMap2_in)
            {
                if (mapPointObservations.find(p_keyFrameInMap2) !=
                    mapPointObservations.end())
                {
                    if (matchedMapPointCounts.find(p_keyFrameInMap2) !=
                        matchedMapPointCounts.end())
                    {
                        matchedMapPointCounts[p_keyFrameInMap2] =
                            matchedMapPointCounts[p_keyFrameInMap2] + 1;
                    }
                    else
                    {
                        matchedMapPointCounts[p_keyFrameInMap2] = 1;
                    }
                }
            }
        }

        if (matchedMapPointCounts.size() == 0)
        {
            cout << "CHECK-OBS: KF " << p_keyFrameInMap1->id
                 << " has not any matched MP with the other map" << endl;
        }
        else
        {
            cout << "CHECK-OBS: KF " << p_keyFrameInMap1->id
                 << " has matched MP with " << matchedMapPointCounts.size()
                 << " KF from the other map" << endl;
            for (pair<KeyFrame *, int> matchedKeyFrame : matchedMapPointCounts)
            {
                cout << "   -KF: " << matchedKeyFrame.first->id
                     << ", Number of matches: " << matchedKeyFrame.second
                     << endl;
            }
        }
    }
    cout << "----------------------" << endl;
}

} // namespace core
} // namespace vs_graphs
