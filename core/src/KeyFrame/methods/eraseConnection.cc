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
 * @file            eraseConnection.cc
 *
 * @brief           Implements KeyFrame::eraseConnection(), declared in
 *                  KeyFrame.h.
 */

#include "KeyFrame.h"

#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

#include <mutex>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

KeyFrameStatus KeyFrame::eraseConnection(KeyFrame *p_keyFrame_in)
{
    bool shouldUpdate = false;
    {
        std::unique_lock<std::mutex> lock(connectionsMutex);
        if (connectedKeyFrameWeights.count(p_keyFrame_in))
        {
            connectedKeyFrameWeights.erase(p_keyFrame_in);
            shouldUpdate = true;
        }
    }

    if (shouldUpdate)
    {
        if (updateBestCovisibles() != KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateBestCovisibles returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
