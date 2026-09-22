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

#include "KeyFrame.h"

#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>

namespace vs_graphs
{
namespace core
{

void KeyFrame::updateBestCovisibles()
{
    unique_lock<mutex>            lock(mMutexConnections);
    vector<pair<int, KeyFrame *>> vPairs;
    vPairs.reserve(connectedKeyFrameWeights.size());
    for (map<KeyFrame *, int>::iterator mit  = connectedKeyFrameWeights.begin(),
                                        mend = connectedKeyFrameWeights.end();
         mit != mend;
         mit++)
        vPairs.push_back(make_pair(mit->second, mit->first));

    sort(vPairs.begin(), vPairs.end());
    list<KeyFrame *> lKFs;
    list<int>        lWs;
    for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
    {
        if (vPairs[i].second != nullptr)
        {
            if (!vPairs[i].second->isBad())
            {
                lKFs.push_front(vPairs[i].second);
                lWs.push_front(vPairs[i].first);
            }
        }
    }

    orderedConnectedKeyFrames = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
    orderedWeights            = vector<int>(lWs.begin(), lWs.end());
}

} // namespace core
} // namespace vs_graphs
