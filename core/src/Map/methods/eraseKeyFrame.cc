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

#include "Map.h"

#include <algorithm>
#include <iterator>
#include <mutex>

namespace vs_graphs
{
namespace core
{

void Map::eraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    keyFrames.erase(pKF);
    keyFrameIndex.erase(pKF->mnId);
    keyFrameOrigins.erase(
        std::remove(keyFrameOrigins.begin(), keyFrameOrigins.end(), pKF),
        keyFrameOrigins.end());

    if (p_firstRegionKeyFrame == pKF)
    {
        p_firstRegionKeyFrame = nullptr;
    }

    if (p_initialKeyFrame == pKF)
    {
        p_initialKeyFrame = nullptr;
    }

    if (keyFrames.size() > 0)
    {
        if (pKF->mnId == p_lowerIdKeyFrame->mnId)
        {
            vector<KeyFrame *> vpKFs =
                vector<KeyFrame *>(keyFrames.begin(), keyFrames.end());
            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
            p_lowerIdKeyFrame = vpKFs[0];
        }

        if (p_initialKeyFrame == nullptr)
        {
            p_initialKeyFrame = p_lowerIdKeyFrame;
        }
    }
    else
    {
        p_lowerIdKeyFrame = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

} // namespace core
} // namespace vs_graphs
