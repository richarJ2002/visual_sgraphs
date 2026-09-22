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
 * @file            toMatrix4f.cc
 *
 * @brief           Implements Converter::toMatrix4f(), declared in
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

Eigen::Matrix<float, 4, 4> Converter::toMatrix4f(const cv::Mat &matrix_in)
{
    Eigen::Matrix<float, 4, 4> eigenMatrix;

    eigenMatrix << matrix_in.at<float>(0, 0), matrix_in.at<float>(0, 1),
        matrix_in.at<float>(0, 2), matrix_in.at<float>(0, 3),
        matrix_in.at<float>(1, 0), matrix_in.at<float>(1, 1),
        matrix_in.at<float>(1, 2), matrix_in.at<float>(1, 3),
        matrix_in.at<float>(2, 0), matrix_in.at<float>(2, 1),
        matrix_in.at<float>(2, 2), matrix_in.at<float>(2, 3),
        matrix_in.at<float>(3, 0), matrix_in.at<float>(3, 1),
        matrix_in.at<float>(3, 2), matrix_in.at<float>(3, 3);
    return eigenMatrix;
}

} // namespace converter
} // namespace utils
} // namespace core
} // namespace vs_graphs
