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

#include "Tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

TrackingStatus Tracking::updateLocalPoints()
{
    localMapPoints.clear();

    int pointCount = 0;

    for (std::vector<KeyFrame *>::const_reverse_iterator
             itKeyFrame    = localKeyFrames.rbegin(),
             itEndKeyFrame = localKeyFrames.rend();
         itKeyFrame != itEndKeyFrame;
         ++itKeyFrame)
    {
        KeyFrame               *p_keyFrame = *itKeyFrame;
        std::vector<MapPoint *> mapPoints{};
        if (p_keyFrame->getMapPointMatches(mapPoints) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getMapPointMatches returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        for (std::vector<MapPoint *>::const_iterator
                 itMapPoint    = mapPoints.begin(),
                 itEndMapPoint = mapPoints.end();
             itMapPoint != itEndMapPoint;
             itMapPoint++)
        {

            MapPoint *p_mapPoint = *itMapPoint;
            if (!p_mapPoint)
                continue;
            if (p_mapPoint->trackReferenceFrameId == currentFrame.id)
                continue;
            bool mapPointIsBad{};
            if (p_mapPoint->isBad(mapPointIsBad) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: isBad returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (!mapPointIsBad)
            {
                pointCount++;
                localMapPoints.push_back(p_mapPoint);
                p_mapPoint->trackReferenceFrameId = currentFrame.id;
            }
        }
    }

    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
