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
#include "Map.h"

#include <algorithm>
#include <iterator>
#include <mutex>

namespace vs_graphs
{
namespace core
{

MapStatus Map::eraseKeyFrame(KeyFrame *p_keyFrame_inout)
{
    std::unique_lock<std::mutex> lock(mapMutex);
    keyFrames.erase(p_keyFrame_inout);
    keyFrameIndex.erase(p_keyFrame_inout->id);
    keyFrameOrigins.erase(std::remove(keyFrameOrigins.begin(),
                                      keyFrameOrigins.end(),
                                      p_keyFrame_inout),
                          keyFrameOrigins.end());

    if (p_firstRegionKeyFrame == p_keyFrame_inout)
    {
        p_firstRegionKeyFrame = nullptr;
    }

    if (p_initialKeyFrame == p_keyFrame_inout)
    {
        p_initialKeyFrame = nullptr;
    }

    if (keyFrames.size() > 0)
    {
        if (p_keyFrame_inout->id == p_lowerIdKeyFrame->id)
        {
            std::vector<KeyFrame *> remainingKeyFrames =
                std::vector<KeyFrame *>(keyFrames.begin(), keyFrames.end());
            std::sort(remainingKeyFrames.begin(),
                      remainingKeyFrames.end(),
                      KeyFrame::lId);
            p_lowerIdKeyFrame = remainingKeyFrames[0];
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

    return MapStatus::MAP_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
