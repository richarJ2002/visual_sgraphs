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
 * @file            fromCvMat.cc
 *
 * @brief           Implements the Converter::toSophus() overload taking
 *                  a cv pose matrix, declared in
 *                  Utils/Converter/objects/Converter.h.
 */

#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace converter
{

ConverterStatus Converter::toSophus(const cv::Mat      &transform_in,
                                    Sophus::SE3<float> &sophus_out)
{
    Eigen::Matrix<double, 3, 3> eigenMatrix{};
    if (toMatrix3d(transform_in.rowRange(0, 3).colRange(0, 3), eigenMatrix) !=
        ConverterStatus::CONVERTER_STATUS_SUCCESS)
    {
        // toMatrix3d cannot fail; continue as before.
    }
    Eigen::Quaternionf quaternion(eigenMatrix.cast<float>());

    Eigen::Matrix<double, 3, 1> vector3d{};
    if (toVector3d(transform_in.rowRange(0, 3).col(3), vector3d) !=
        ConverterStatus::CONVERTER_STATUS_SUCCESS)
    {
        // toVector3d cannot fail; continue as before.
    }
    Eigen::Matrix<float, 3, 1> translation = vector3d.cast<float>();

    sophus_out = Sophus::SE3<float>(quaternion, translation);
    return ConverterStatus::CONVERTER_STATUS_SUCCESS;
}

} // namespace converter
} // namespace utils
} // namespace core
} // namespace vs_graphs
