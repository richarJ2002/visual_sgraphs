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

void EdgeInertial::computeError()
{
    // TODO Maybe Reintegrate inertial measurments when difference between
    // linearization point and current estimate is too big
    const VertexPose     *VP1 = static_cast<const VertexPose *>(_vertices[0]);
    const VertexVelocity *VV1 =
        static_cast<const VertexVelocity *>(_vertices[1]);
    const VertexGyroBias *VG1 =
        static_cast<const VertexGyroBias *>(_vertices[2]);
    const VertexAccBias *VA1 = static_cast<const VertexAccBias *>(_vertices[3]);
    const VertexPose    *VP2 = static_cast<const VertexPose *>(_vertices[4]);
    const VertexVelocity *VV2 =
        static_cast<const VertexVelocity *>(_vertices[5]);
    const IMU::Bias       b1(VA1->estimate()[0],
                       VA1->estimate()[1],
                       VA1->estimate()[2],
                       VG1->estimate()[0],
                       VG1->estimate()[1],
                       VG1->estimate()[2]);
    const Eigen::Matrix3d dR =
        p_preintegrated->getDeltaRotation(b1).cast<double>();
    const Eigen::Vector3d dV =
        p_preintegrated->getDeltaVelocity(b1).cast<double>();
    const Eigen::Vector3d dP =
        p_preintegrated->getDeltaPosition(b1).cast<double>();

    const Eigen::Vector3d er = LogSO3(
        dR.transpose() * VP1->estimate().Rwb.transpose() * VP2->estimate().Rwb);
    const Eigen::Vector3d ev =
        VP1->estimate().Rwb.transpose() *
            (VV2->estimate() - VV1->estimate() - g * dt) -
        dV;
    const Eigen::Vector3d ep = VP1->estimate().Rwb.transpose() *
                                   (VP2->estimate().twb - VP1->estimate().twb -
                                    VV1->estimate() * dt - g * dt * dt / 2) -
                               dP;

    _error << er, ev, ep;
}

} // namespace core
} // namespace vs_graphs
