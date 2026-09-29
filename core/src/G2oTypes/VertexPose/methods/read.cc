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

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

bool VertexPose::read(std::istream &inputStream_inout)
{
    std::vector<Eigen::Matrix<double, 3, 3>> Rcw;
    std::vector<Eigen::Matrix<double, 3, 1>> tcw;
    std::vector<Eigen::Matrix<double, 3, 3>> Rbc;
    std::vector<Eigen::Matrix<double, 3, 1>> tbc;

    const int cameraCount = _estimate.Rbc.size();
    for (int cameraIndex = 0; cameraIndex < cameraCount; cameraIndex++)
    {
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
                inputStream_inout >>
                    Rcw[cameraIndex](componentIndex, columnIndex);
        }
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            inputStream_inout >> tcw[cameraIndex](componentIndex);
        }

        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
                inputStream_inout >>
                    Rbc[cameraIndex](componentIndex, columnIndex);
        }
        for (int componentIndex = 0; componentIndex < 3; componentIndex++)
        {
            inputStream_inout >> tbc[cameraIndex](componentIndex);
        }

        float  nextParam;
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
            inputStream_inout >> nextParam;
            if (_estimate.pCamera[cameraIndex]->setParameter(nextParam,
                                                             componentIndex) !=
                camera_models::geometriccamera::GeometricCameraStatus::
                    GEOMETRIC_CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: setParameter returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
        }
    }

    double baselineFocalProduct;
    inputStream_inout >> baselineFocalProduct;
    _estimate.setParam(Rcw, tcw, Rbc, tbc, baselineFocalProduct);
    updateCache();

    return true;
}

} // namespace core
} // namespace vs_graphs
