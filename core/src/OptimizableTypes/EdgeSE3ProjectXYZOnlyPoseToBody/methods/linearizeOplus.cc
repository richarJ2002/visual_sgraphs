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

void EdgeSE3ProjectXYZOnlyPoseToBody::linearizeOplus()
{
    g2o::VertexSE3Expmap *vi =
        static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
    g2o::SE3Quat    T_lw(vi->estimate());
    Eigen::Vector3d X_l = T_lw.map(Xw);
    Eigen::Vector3d X_r = mTrl.map(T_lw.map(Xw));

    double x_w = X_l[0];
    double y_w = X_l[1];
    double z_w = X_l[2];

    Eigen::Matrix<double, 3, 6> SE3deriv;
    SE3deriv << 0.f, z_w, -y_w, 1.f, 0.f, 0.f, -z_w, 0.f, x_w, 0.f, 1.f, 0.f,
        y_w, -x_w, 0.f, 0.f, 0.f, 1.f;

    _jacobianOplusXi = -pCamera->computeProjectionJacobian(X_r) *
                       mTrl.rotation().toRotationMatrix() * SE3deriv;
}

} // namespace core
} // namespace vs_graphs
