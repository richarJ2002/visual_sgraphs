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

#include <mutex>

namespace vs_graphs
{
namespace core
{

void LoopClosing::resetIfRequested()
{
    unique_lock<mutex> lock(resetMutex);
    if (isResetRequested)
    {
        cout << "Loop closer reset requested..." << endl;
        loopKeyFrameQueue.clear();
        lastLoopKeyFrameId =
            0; // TODO old variable, it is not use in the new algorithm
        isResetRequested          = false;
        isResetActiveMapRequested = false;
    }
    else if (isResetActiveMapRequested)
    {

        for (list<KeyFrame *>::const_iterator loopKeyFrameIt =
                 loopKeyFrameQueue.begin();
             loopKeyFrameIt != loopKeyFrameQueue.end();)
        {
            KeyFrame *p_keyFrame = *loopKeyFrameIt;
            if (p_keyFrame->getMap() == p_mapToReset)
            {
                loopKeyFrameIt = loopKeyFrameQueue.erase(loopKeyFrameIt);
            }
            else
                ++loopKeyFrameIt;
        }

        lastLoopKeyFrameId =
            p_atlas->getLastInitKeyFrameId(); // TODO old variable, it is not
                                              // use in the new algorithm
        isResetActiveMapRequested = false;
    }
}

} // namespace core
} // namespace vs_graphs
