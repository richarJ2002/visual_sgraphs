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

MapStatus Map::addKeyFrame(KeyFrame *p_keyFrame_inout)
{
    unique_lock<mutex> lock(mapMutex);

    // First keyframe seeds the map (origin and lowest-id keyframe references);
    // later keyframes are inserted with no id-duplicate check.
    if (keyFrames.empty())
    {
        std::cout << "\n[Mapping] Map initialized with initial KeyFrame #"
                  << initKeyFrameId << "." << std::endl;
        initKeyFrameId    = p_keyFrame_inout->id;
        p_initialKeyFrame = p_keyFrame_inout;
        p_lowerIdKeyFrame = p_keyFrame_inout;
    }

    // Add the KeyFrame to the map
    keyFrames.insert(p_keyFrame_inout);

    // Update the maximum KeyFrame id
    if (p_keyFrame_inout->id > maxKeyFrameId)
        maxKeyFrameId = p_keyFrame_inout->id;

    if (p_keyFrame_inout->id < p_lowerIdKeyFrame->id)
        p_lowerIdKeyFrame = p_keyFrame_inout;

    keyFrameIndex[p_keyFrame_inout->id] = p_keyFrame_inout;

    return MapStatus::MAP_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
