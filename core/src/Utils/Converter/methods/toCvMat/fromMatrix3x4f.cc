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
 * @file            fromMatrix3x4f.cc
 *
 * @brief           Implements the Converter::toCvMat() overload taking
 *                  a float 3x4 matrix, declared in
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

cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 3, 4> &matrix_in)
{
    cv::Mat cvMatrix(3, 4, CV_32F);
    for (int rowIndex = 0; rowIndex < 3; rowIndex++)
        for (int columnIndex = 0; columnIndex < 4; columnIndex++)
            cvMatrix.at<float>(rowIndex, columnIndex) =
                matrix_in(rowIndex, columnIndex);

    return cvMatrix.clone();
}

} // namespace converter
} // namespace utils
} // namespace core
} // namespace vs_graphs
