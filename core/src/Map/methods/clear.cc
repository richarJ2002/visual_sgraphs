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

/*!
 * @file            clear.cc
 *
 * @brief           Implements Map::clear(), declared in Map.h.
 */

#include "KeyFrame.h"
#include "Map.h"

#include <algorithm>
#include <iterator>
#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapStatus Map::clear()
{
    for (std::set<KeyFrame *>::iterator sit  = keyFrames.begin(),
                                        send = keyFrames.end();
         sit != send;
         sit++)
    {
        KeyFrame *p_keyFrame = *sit;
        if (p_keyFrame->updateMap(static_cast<Map *>(nullptr)) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateMap returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    planes.clear();
    markers.clear();
    passages.clear();
    floors.clear();
    doors.clear();
    mapPoints.clear();
    keyFrames.clear();

    planeIndex.clear();
    markerIndex.clear();
    passageIndex.clear();
    floorIndex.clear();
    doorIndex.clear();
    keyFrameIndex.clear();
    roomWallPlaneIndex.clear();

    skeletonClusterPoints.clear();
    skeletonEdges.clear();

    maxKeyFrameId        = initKeyFrameId;
    hasImuInitialization = false;
    detectedRooms.clear();
    markerBasedRooms.clear();
    referenceMapPoints.clear();
    keyFrameOrigins.clear();
    hasInertialBA1 = false;
    hasInertialBA2 = false;

    return MapStatus::MAP_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
