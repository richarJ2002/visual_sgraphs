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

void EdgePriorPoseImu::computeError()
{
    const VertexPose *p_poseVertex =
        static_cast<const VertexPose *>(_vertices[0]);
    const VertexVelocity *p_velocityVertex =
        static_cast<const VertexVelocity *>(_vertices[1]);
    const VertexGyroBias *p_gyroBiasVertex =
        static_cast<const VertexGyroBias *>(_vertices[2]);
    const VertexAccBias *p_accBiasVertex =
        static_cast<const VertexAccBias *>(_vertices[3]);

    const Eigen::Vector3d rotationError =
        logSO3(Rwb.transpose() * p_poseVertex->estimate().Rwb);
    const Eigen::Vector3d translationError =
        Rwb.transpose() * (p_poseVertex->estimate().twb - twb);
    const Eigen::Vector3d velocityError = p_velocityVertex->estimate() - vwb;
    const Eigen::Vector3d gyroBiasError = p_gyroBiasVertex->estimate() - bg;
    const Eigen::Vector3d accBiasError  = p_accBiasVertex->estimate() - ba;

    _error << rotationError, translationError, velocityError, gyroBiasError,
        accBiasError;
}

} // namespace core
} // namespace vs_graphs
