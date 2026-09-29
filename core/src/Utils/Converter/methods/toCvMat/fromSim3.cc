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
 * @file            fromSim3.cc
 *
 * @brief           Implements the Converter::toCvMat() overload taking
 *                  a similarity transform, declared in
 *                  Utils/Converter/objects/Converter.h.
 */

#include "Utils/Converter/objects/Converter.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace converter
{

ConverterStatus Converter::toCvMat(const g2o::Sim3 &similarity_in,
                                   cv::Mat         &cvMat_out)
{
    Eigen::Matrix3d eigenRotation = similarity_in.rotation().toRotationMatrix();
    Eigen::Vector3d eigenTranslation = similarity_in.translation();
    double          scale            = similarity_in.scale();
    /* Fold the similarity scale into the rotation part. */
    cv::Mat         cvSE3{};
    if (toCvSE3(scale * eigenRotation, eigenTranslation, cvSE3) !=
        ConverterStatus::CONVERTER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: toCvSE3 returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    cvMat_out = cvSE3;
    return ConverterStatus::CONVERTER_STATUS_SUCCESS;
}

} // namespace converter
} // namespace utils
} // namespace core
} // namespace vs_graphs
