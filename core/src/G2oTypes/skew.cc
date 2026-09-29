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

namespace vs_graphs
{
namespace core
{

G2oTypesStatus computeSkewMatrix(const Eigen::Vector3d &angularVelocity_in,
                                 Eigen::Matrix3d       &skewMatrix_out)
{
    Eigen::Matrix3d skewMatrix;
    skewMatrix << 0.0, -angularVelocity_in[2], angularVelocity_in[1],
        angularVelocity_in[2], 0.0, -angularVelocity_in[0],
        -angularVelocity_in[1], angularVelocity_in[0], 0.0;
    skewMatrix_out = skewMatrix;
    return G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
