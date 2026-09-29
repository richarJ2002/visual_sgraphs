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

#include "MapPoint.h"

#include "ORBmatcher.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MapPointStatus MapPoint::preSave(std::set<KeyFrame *> &keyFrames_in,
                                 std::set<MapPoint *> &mapPoints_in)
{
    backupReplacedId = -1;

    backupObservationIds1.clear();
    backupObservationIds2.clear();

    // Snapshot the observation map and replaced pointer under the feature lock.
    // Dropped keyframes are erased below, after the lock is released, because
    // EraseObservation() takes featuresMutex again.
    std::map<KeyFrame *, std::tuple<int, int>> savedObservations;
    {
        unique_lock<mutex> lock(featuresMutex);
        if (p_replaced && mapPoints_in.find(p_replaced) != mapPoints_in.end())
            backupReplacedId = p_replaced->id;

        savedObservations.insert(observations.begin(), observations.end());
    }

    for (std::map<KeyFrame *, std::tuple<int, int>>::const_iterator
             observationIt = savedObservations.begin(),
             end           = savedObservations.end();
         observationIt != end;
         ++observationIt)
    {
        KeyFrame *p_keyFrame = observationIt->first;
        if (keyFrames_in.find(p_keyFrame) != keyFrames_in.end())
        {
            backupObservationIds1[observationIt->first->id] =
                get<0>(observationIt->second);
            backupObservationIds2[observationIt->first->id] =
                get<1>(observationIt->second);
        }
        else
        {
            if (eraseObservation(p_keyFrame) !=
                MapPointStatus::MAP_POINT_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: eraseObservation returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    // Save the id of the reference KF
    unique_lock<mutex> lock(featuresMutex);
    if (keyFrames_in.find(p_referenceKeyFrame) != keyFrames_in.end())
    {
        backupRefKeyFrameId = p_referenceKeyFrame->id;
    }

    return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
