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
 * @brief           Implements VertexPose::write(), declared in G2oTypes.h.
 */

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool VertexPose::write(std::ostream &outputStream_out) const
{
    std::vector<Eigen::Matrix<double, 3, 3>> cameraRotation_worldToCamera =
        _estimate.Rcw;
    std::vector<Eigen::Matrix<double, 3, 1>> cameraTranslation_worldToCamera =
        _estimate.tcw;

    std::vector<Eigen::Matrix<double, 3, 3>> extrinsicRotation_cameraToBody =
        _estimate.Rbc;
    std::vector<Eigen::Matrix<double, 3, 1>> extrinsicTranslation_cameraToBody =
        _estimate.tbc;

    const int cameraCount = cameraTranslation_worldToCamera.size();

    for (int cameraIndex = 0; cameraIndex < cameraCount; cameraIndex++)
    {
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
                outputStream_out
                    << cameraRotation_worldToCamera[cameraIndex](componentIndex,
                                                                 columnIndex)
                    << " ";
        }
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            outputStream_out
                << cameraTranslation_worldToCamera[cameraIndex](componentIndex)
                << " ";
        }

        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
                outputStream_out << extrinsicRotation_cameraToBody[cameraIndex](
                                        componentIndex,
                                        columnIndex)
                                 << " ";
        }
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            outputStream_out << extrinsicTranslation_cameraToBody[cameraIndex](
                                    componentIndex)
                             << " ";
        }

        size_t size2{};
        if (_estimate.pCamera[cameraIndex]->size(size2) !=
            camera_models::geometriccamera::GeometricCameraStatus::
                GEOMETRIC_CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: size returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        for (size_t componentIndex = 0; componentIndex < size2;
             componentIndex++)
        {
            float parameter{};
            if (_estimate.pCamera[cameraIndex]->getParameter(componentIndex,
                                                             parameter) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            outputStream_out << parameter << " ";
        }
    }

    outputStream_out << _estimate.bf << " ";

    return outputStream_out.good();
}

} // namespace core
} // namespace vs_graphs
