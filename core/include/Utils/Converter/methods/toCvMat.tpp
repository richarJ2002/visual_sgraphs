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
 * @file            toCvMat.tpp
 *
 * @brief           Implements the Converter::toCvMat() template taking an
 *                  Eigen matrix or vector, declared in
 *                  Utils/Converter/objects/Converter.h.
 */

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace converter
{

template <typename Derived>
ConverterStatus Converter::toCvMat(const Eigen::MatrixBase<Derived> &matrix_in,
                                   cv::Mat                          &cvMat_out)
{
    cv::Mat cvMatrix(static_cast<int>(matrix_in.rows()),
                     static_cast<int>(matrix_in.cols()),
                     CV_32F);
    for (int rowIndex = 0; rowIndex < cvMatrix.rows; rowIndex++)
        for (int columnIndex = 0; columnIndex < cvMatrix.cols; columnIndex++)
            cvMatrix.at<float>(rowIndex, columnIndex) =
                static_cast<float>(matrix_in(rowIndex, columnIndex));

    /* cvMatrix owns a fresh buffer, so the result never aliases a matrix the
     * caller passed in as cvMat_out. */
    cvMat_out = cvMatrix;
    return ConverterStatus::CONVERTER_STATUS_SUCCESS;
}

} // namespace converter
} // namespace utils
} // namespace core
} // namespace vs_graphs
