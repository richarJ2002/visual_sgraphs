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
 * @file            isRotationMatrix.cc
 *
 * @brief           Implements Converter::isRotationMatrix(), declared
 *                  in Utils/Converter/objects/Converter.h.
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

ConverterStatus Converter::isRotationMatrix(const cv::Mat &rotationMatrix_in,
                                            bool          &isRotationMatrix_out)
{
    cv::Mat rotationTranspose;
    cv::transpose(rotationMatrix_in, rotationTranspose);
    cv::Mat shouldBeIdentity = rotationTranspose * rotationMatrix_in;
    cv::Mat identity         = cv::Mat::eye(3, 3, shouldBeIdentity.type());

    /* Accept small numerical drift around exact orthonormality. */
    isRotationMatrix_out = cv::norm(identity, shouldBeIdentity) < 1e-6;
    return ConverterStatus::CONVERTER_STATUS_SUCCESS;
}

} // namespace converter
} // namespace utils
} // namespace core
} // namespace vs_graphs
