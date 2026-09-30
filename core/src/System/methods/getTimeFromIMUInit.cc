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
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            getTimeFromIMUInit.cc
 *
 * @brief           Implements System::getTimeFromIMUInit(), declared in
 *                  System.h.
 */

#include "LocalMapping.h"
#include "System.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SystemStatus System::getTimeFromIMUInit(double &timeFromIMUInit_out)
{
    double localMapperCurrentKeyFrameTime{};
    if (p_localMapper->getCurrentKeyFrameTime(localMapperCurrentKeyFrameTime) !=
        LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getCurrentKeyFrameTime returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    double aux = localMapperCurrentKeyFrameTime - p_localMapper->firstTimestamp;
    bool   atlasIsImuInitialized{};
    if (((aux > 0.)) && p_atlas->isImuInitialized(atlasIsImuInitialized) !=
                            AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isImuInitialized returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if ((aux > 0.) && atlasIsImuInitialized)
    {
        double localMapperCurrentKeyFrameTime2{};
        if (p_localMapper->getCurrentKeyFrameTime(
                localMapperCurrentKeyFrameTime2) !=
            LocalMappingStatus::LOCAL_MAPPING_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getCurrentKeyFrameTime returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        timeFromIMUInit_out =
            localMapperCurrentKeyFrameTime2 - p_localMapper->firstTimestamp;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
    else
    {
        timeFromIMUInit_out = 0.f;
        return SystemStatus::SYSTEM_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
