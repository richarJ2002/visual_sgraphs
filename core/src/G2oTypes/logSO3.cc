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
 * @file            logSO3.cc
 *
 * @brief           Implements logSO3(), declared in G2oTypes.h.
 */

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{

G2oTypesStatus logSO3(const Eigen::Matrix3d &rotationMatrix_in,
                      Eigen::Vector3d       &rotationVector_out)
{
    const double trace = rotationMatrix_in(0, 0) + rotationMatrix_in(1, 1) +
                         rotationMatrix_in(2, 2);
    Eigen::Vector3d rotationVector;
    rotationVector << (rotationMatrix_in(2, 1) - rotationMatrix_in(1, 2)) / 2,
        (rotationMatrix_in(0, 2) - rotationMatrix_in(2, 0)) / 2,
        (rotationMatrix_in(1, 0) - rotationMatrix_in(0, 1)) / 2;
    const double cosAngle = (trace - 1.0) * 0.5f;
    if (cosAngle > 1 || cosAngle < -1)
    {
        rotationVector_out = rotationVector;
        return G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS;
    }
    const double angle    = acos(cosAngle);
    const double sinAngle = sin(angle);
    if (fabs(sinAngle) < 1e-5)
    {
        rotationVector_out = rotationVector;
        return G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS;
    }
    else
    {
        rotationVector_out = angle * rotationVector / sinAngle;
        return G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS;
    }
}

} // namespace core
} // namespace vs_graphs
