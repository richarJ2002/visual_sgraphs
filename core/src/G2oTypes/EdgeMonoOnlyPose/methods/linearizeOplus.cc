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

void EdgeMonoOnlyPose::linearizeOplus()
{
    const VertexPose *VPose = static_cast<const VertexPose *>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = VPose->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = VPose->estimate().tcw[cam_idx];
    const Eigen::Vector3d  Xc  = Rcw * Xw + tcw;
    const Eigen::Vector3d  Xb =
        VPose->estimate().Rbc[cam_idx] * Xc + VPose->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = VPose->estimate().Rcb[cam_idx];

    Eigen::Matrix<double, 2, 3> proj_jac =
        VPose->estimate().pCamera[cam_idx]->computeProjectionJacobian(Xc);

    Eigen::Matrix<double, 3, 6> SE3deriv;
    double                      x = Xb(0);
    double                      y = Xb(1);
    double                      z = Xb(2);
    SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0, -z, 0.0, x, 0.0, 1.0, 0.0, y, -x,
        0.0, 0.0, 0.0, 1.0;
    _jacobianOplusXi =
        proj_jac * Rcb * SE3deriv; // symbol different becasue of update mode
}

} // namespace core
} // namespace vs_graphs
