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

void Map::addKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);

    // First keyframe seeds the map (origin and lowest-id keyframe references);
    // later keyframes are inserted with no id-duplicate check.
    if (keyFrames.empty())
    {
        std::cout << "\n[Mapping] Map initialized with initial KeyFrame #"
                  << initKeyFrameId << "." << std::endl;
        initKeyFrameId    = pKF->mnId;
        p_initialKeyFrame = pKF;
        p_lowerIdKeyFrame = pKF;
    }

    // Add the KeyFrame to the map
    keyFrames.insert(pKF);

    // Update the maximum KeyFrame id
    if (pKF->mnId > maxKeyFrameId)
        maxKeyFrameId = pKF->mnId;

    if (pKF->mnId < p_lowerIdKeyFrame->mnId)
        p_lowerIdKeyFrame = pKF;

    keyFrameIndex[pKF->mnId] = pKF;
}

} // namespace core
} // namespace vs_graphs
