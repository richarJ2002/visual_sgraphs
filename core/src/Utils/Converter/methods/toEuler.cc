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
 * @file            toEuler.cc
 *
 * @brief           Implements Converter::toEuler(), declared in
 *                  Utils/Converter/objects/Converter.h.
 */

#include "Utils/Converter/objects/Converter.h"

#include <cassert>
#include <cmath>

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace converter
{

ConverterStatus Converter::toEuler(const cv::Mat      &rotationMatrix_in,
                                   std::vector<float> &euler_out)
{
    bool isRotation = false;
    if (isRotationMatrix(rotationMatrix_in, isRotation) !=
        ConverterStatus::CONVERTER_STATUS_SUCCESS)
    {
        // isRotationMatrix cannot fail; continue as before.
    }
    assert(isRotation);
    float symmetricSum = sqrt(
        rotationMatrix_in.at<float>(0, 0) * rotationMatrix_in.at<float>(0, 0) +
        rotationMatrix_in.at<float>(1, 0) * rotationMatrix_in.at<float>(1, 0));

    bool singular = symmetricSum < 1e-6; // If

    float xAngle, yAngle, zAngle;
    if (!singular)
    {
        xAngle = atan2(rotationMatrix_in.at<float>(2, 1),
                       rotationMatrix_in.at<float>(2, 2));
        yAngle = atan2(-rotationMatrix_in.at<float>(2, 0), symmetricSum);
        zAngle = atan2(rotationMatrix_in.at<float>(1, 0),
                       rotationMatrix_in.at<float>(0, 0));
    }
    else
    {
        /* Fall back to the gimbal-lock form near the singularity. */
        xAngle = atan2(-rotationMatrix_in.at<float>(1, 2),
                       rotationMatrix_in.at<float>(1, 1));
        yAngle = atan2(-rotationMatrix_in.at<float>(2, 0), symmetricSum);
        zAngle = 0;
    }

    std::vector<float> eulerAngles(3);
    eulerAngles[0] = xAngle;
    eulerAngles[1] = yAngle;
    eulerAngles[2] = zAngle;

    euler_out = eulerAngles;
    return ConverterStatus::CONVERTER_STATUS_SUCCESS;
}

} // namespace converter
} // namespace utils
} // namespace core
} // namespace vs_graphs
