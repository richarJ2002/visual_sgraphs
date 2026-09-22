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
    const VertexPose     *VP = static_cast<const VertexPose *>(_vertices[0]);
    const VertexVelocity *VV =
        static_cast<const VertexVelocity *>(_vertices[1]);
    const VertexGyroBias *VG =
        static_cast<const VertexGyroBias *>(_vertices[2]);
    const VertexAccBias *VA = static_cast<const VertexAccBias *>(_vertices[3]);

    const Eigen::Vector3d er  = LogSO3(Rwb.transpose() * VP->estimate().Rwb);
    const Eigen::Vector3d et  = Rwb.transpose() * (VP->estimate().twb - twb);
    const Eigen::Vector3d ev  = VV->estimate() - vwb;
    const Eigen::Vector3d ebg = VG->estimate() - bg;
    const Eigen::Vector3d eba = VA->estimate() - ba;

    _error << er, et, ev, ebg, eba;
}

} // namespace core
} // namespace vs_graphs
