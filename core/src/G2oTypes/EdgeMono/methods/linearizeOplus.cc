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

void EdgeMono::linearizeOplus()
{
    const VertexPose *p_poseVertex =
        static_cast<const VertexPose *>(_vertices[1]);
    const g2o::VertexSBAPointXYZ *p_mapPointVertex =
        static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

    const Eigen::Matrix3d &Rcw = p_poseVertex->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &tcw = p_poseVertex->estimate().tcw[cam_idx];
    const Eigen::Vector3d  Xc  = Rcw * p_mapPointVertex->estimate() + tcw;
    const Eigen::Vector3d  Xb  = p_poseVertex->estimate().Rbc[cam_idx] * Xc +
                               p_poseVertex->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &Rcb = p_poseVertex->estimate().Rcb[cam_idx];

    const Eigen::Matrix<double, 2, 3> projectionJacobian =
        p_poseVertex->estimate().pCamera[cam_idx]->computeProjectionJacobian(
            Xc);
    _jacobianOplusXi = -projectionJacobian * Rcw;

    Eigen::Matrix<double, 3, 6> se3Derivative;
    double                      bodyPointX = Xb(0);
    double                      bodyPointY = Xb(1);
    double                      bodyPointZ = Xb(2);

    se3Derivative << 0.0, bodyPointZ, -bodyPointY, 1.0, 0.0, 0.0, -bodyPointZ,
        0.0, bodyPointX, 0.0, 1.0, 0.0, bodyPointY, -bodyPointX, 0.0, 0.0, 0.0,
        1.0;

    _jacobianOplusXj =
        projectionJacobian * Rcb * se3Derivative; // TODO optimize this product
}

} // namespace core
} // namespace vs_graphs
