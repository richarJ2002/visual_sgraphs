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

#include "OptimizableTypes.h"

namespace vs_graphs
{
namespace core
{

void EdgeSE3ProjectXYZDepth::linearizeOplus()
{
    g2o::VertexSE3Expmap *vi =
        static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
    Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

    double x = xyz_trans[0];
    double y = xyz_trans[1];

    // Derivative of depth (z-coordinate in camera frame) w.r.t SE3 pose
    Eigen::Matrix<double, 1, 6> SE3deriv_z;
    SE3deriv_z << y, -x, 0, 0, 0, 1; // d(z)/d(xi) where xi = [rot, trans]

    _jacobianOplusXi = -SE3deriv_z;
}

} // namespace core
} // namespace vs_graphs
