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

MapPointStatus MapPoint::eraseObservation(KeyFrame *p_keyFrame_in)
{
    bool bBad = false;
    {
        std::unique_lock<std::mutex> lock(featuresMutex);
        if (observations.count(p_keyFrame_in))
        {
            std::tuple<int, int> indexes   = observations[p_keyFrame_in];
            int                  leftIndex = std::get<0>(indexes),
                rightIndex                 = std::get<1>(indexes);

            if (leftIndex != -1)
            {
                if (!p_keyFrame_in->p_camera2 &&
                    p_keyFrame_in->uRight[leftIndex] >= 0)
                    observationCount -= 2;
                else
                    observationCount--;
            }
            if (rightIndex != -1)
            {
                observationCount--;
            }

            observations.erase(p_keyFrame_in);

            if (p_referenceKeyFrame == p_keyFrame_in)
            {
                p_referenceKeyFrame = nullptr;

                for (const auto &[p_candidateKeyFrame, featureIndexes] :
                     observations)
                {
                    (void)featureIndexes;

                    bool candidateKeyFrameIsBad{};
                    if (!(p_candidateKeyFrame == nullptr) &&
                        p_candidateKeyFrame->isBad(candidateKeyFrameIsBad) !=
                            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: isBad returned a failure status although it "
                            "cannot fail; continuing as before.",
                            __func__);
                    }
                    if (p_candidateKeyFrame == nullptr ||
                        candidateKeyFrameIsBad)
                    {
                        continue;
                    }

                    if (p_referenceKeyFrame == nullptr ||
                        p_candidateKeyFrame->id < p_referenceKeyFrame->id)
                    {
                        p_referenceKeyFrame = p_candidateKeyFrame;
                    }
                }
            }

            // If only 2 observations or less, discard point
            if (observationCount <= 2)
                bBad = true;
        }
    }

    if (bBad)
    {
        if (setBadFlag() != MapPointStatus::MAP_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setBadFlag returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }

    return MapPointStatus::MAP_POINT_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
