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
 * @file            write.cc
 *
 * @brief           Implements VertexSim3Expmap::write(), declared in
 *                  OptimizableTypes.h.
 */

#include "OptimizableTypes.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool VertexSim3Expmap::write(std::ostream &outputStream_inout) const
{
    g2o::Sim3     cam2world(estimate().inverse());
    g2o::Vector7d logVector = cam2world.log();
    for (int parameterIndex = 0; parameterIndex < 7; parameterIndex++)
    {
        outputStream_inout << logVector[parameterIndex] << " ";
    }
    size_t firstCameraSize{};
    if (p_firstCamera->size(firstCameraSize) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    for (size_t parameterIndex = 0; parameterIndex < firstCameraSize;
         parameterIndex++)
    {
        float firstCameraParameter{};
        if (p_firstCamera->getParameter(parameterIndex, firstCameraParameter) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        outputStream_inout << firstCameraParameter << " ";
    }

    size_t secondCameraSize{};
    if (p_secondCamera->size(secondCameraSize) !=
        camera_models::geometriccamera::GeometricCameraStatus::
            GEOMETRIC_CAMERA_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: size returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    for (size_t parameterIndex = 0; parameterIndex < secondCameraSize;
         parameterIndex++)
    {
        float secondCameraParameter{};
        if (p_secondCamera->getParameter(parameterIndex,
                                         secondCameraParameter) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getParameter returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        outputStream_inout << secondCameraParameter << " ";
    }
    return outputStream_inout.good();
}

} // namespace core
} // namespace vs_graphs
