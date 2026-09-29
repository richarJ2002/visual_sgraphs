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
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

void LoopClosing::recordLoopCorrectionEvent(bool               accepted_in,
                                            const std::string &reason_in)
{
    {
        std::lock_guard<std::mutex> lock(loopCorrectionStatusMutex);
        ++loopCorrectionStatus.sequence;
        loopCorrectionStatus.hasEvent        = true;
        loopCorrectionStatus.wasLastAccepted = accepted_in;
        loopCorrectionStatus.lastReason      = reason_in;

        if (accepted_in)
        {
            ++loopCorrectionStatus.acceptedCount;
        }
        else
        {
            ++loopCorrectionStatus.rejectedCount;
        }

        if (p_currentKF != nullptr)
        {
            loopCorrectionStatus.lastCurrentKeyFrameId = p_currentKF->id;
            loopCorrectionStatus.lastCurrentTimestamp  = p_currentKF->timeStamp;
            Map *p_currentKFMap                        = nullptr;
            if (p_currentKF->getMap(p_currentKFMap) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getMap returned a failure status although it "
                             "cannot fail; continuing as before.",
                             __func__);
            }
            if (p_currentKFMap != nullptr)
            {
                Map *p_currentKFMap2 = nullptr;
                if (p_currentKF->getMap(p_currentKFMap2) !=
                    KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: getMap returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }
                unsigned long id2{};
                if (p_currentKFMap2->getId(id2) !=
                    MapStatus::MAP_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                 "%s: getId returned a failure status although "
                                 "it cannot fail; continuing as before.",
                                 __func__);
                }
                loopCorrectionStatus.lastMapId = id2;
            }
        }
        if (p_loopMatchedKF != nullptr)
        {
            loopCorrectionStatus.lastMatchedKeyFrameId = p_loopMatchedKF->id;
            loopCorrectionStatus.lastMatchedTimestamp =
                p_loopMatchedKF->timeStamp;
        }
    }
}

} // namespace core
} // namespace vs_graphs
