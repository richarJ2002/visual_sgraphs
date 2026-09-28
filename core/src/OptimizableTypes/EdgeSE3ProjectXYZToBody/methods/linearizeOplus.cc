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

void EdgeSE3ProjectXYZToBody::linearizeOplus()
{
    g2o::VertexSE3Expmap *p_poseVertex =
        static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat            T_lw(p_poseVertex->estimate());
    g2o::SE3Quat            T_rw = mTrl * T_lw;
    g2o::VertexSBAPointXYZ *p_pointVertex =
        static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);
    Eigen::Vector3d pointPosition_w = p_pointVertex->estimate();
    Eigen::Vector3d pointPosition_l = T_lw.map(pointPosition_w);
    Eigen::Vector3d pointPosition_r = mTrl.map(T_lw.map(pointPosition_w));

    _jacobianOplusXi = -p_camera->computeProjectionJacobian(pointPosition_r) *
                       T_rw.rotation().toRotationMatrix();

    double transformedX = pointPosition_l[0];
    double transformedY = pointPosition_l[1];
    double transformedZ = pointPosition_l[2];

    Eigen::Matrix<double, 3, 6> se3Derivative;
    se3Derivative << 0.f, transformedZ, -transformedY, 1.f, 0.f, 0.f,
        -transformedZ, 0.f, transformedX, 0.f, 1.f, 0.f, transformedY,
        -transformedX, 0.f, 0.f, 0.f, 1.f;

    _jacobianOplusXj = -p_camera->computeProjectionJacobian(pointPosition_r) *
                       mTrl.rotation().toRotationMatrix() * se3Derivative;
}

} // namespace core
} // namespace vs_graphs
