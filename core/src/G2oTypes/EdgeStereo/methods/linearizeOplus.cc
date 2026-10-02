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
 * @file            linearizeOplus.cc
 *
 * @brief           Implements EdgeStereo::linearizeOplus(), declared in
 *                  G2oTypes.h.
 */

#include "G2oTypes.h"
#include "ImuTypes.h"
#include "Utils/Converter/objects/Converter.h"

namespace vs_graphs
{
namespace core
{

void EdgeStereo::linearizeOplus()
{
    const VertexPose *p_poseVertex =
        static_cast<const VertexPose *>(_vertices[1]);
    const g2o::VertexSBAPointXYZ *p_mapPointVertex =
        static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

    const Eigen::Matrix3d &rotation_worldToCamera =
        p_poseVertex->estimate().Rcw[cam_idx];
    const Eigen::Vector3d &translation_worldToCamera =
        p_poseVertex->estimate().tcw[cam_idx];
    const Eigen::Vector3d Xc =
        rotation_worldToCamera * p_mapPointVertex->estimate() +
        translation_worldToCamera;
    const Eigen::Vector3d Xb = p_poseVertex->estimate().Rbc[cam_idx] * Xc +
                               p_poseVertex->estimate().tbc[cam_idx];
    const Eigen::Matrix3d &rotation_bodyToCamera =
        p_poseVertex->estimate().Rcb[cam_idx];
    const double baselineFocalProduct = p_poseVertex->estimate().bf;
    const double inverseDepthSquared  = 1.0 / (Xc(2) * Xc(2));

    Eigen::Matrix<double, 3, 3> projectionJacobian;
    projectionJacobian.block<2, 3>(0, 0) =
        p_poseVertex->estimate().pCamera[cam_idx]->computeProjectionJacobian(
            Xc);
    projectionJacobian.block<1, 3>(2, 0) = projectionJacobian.block<1, 3>(0, 0);
    projectionJacobian(2, 2) += baselineFocalProduct * inverseDepthSquared;

    _jacobianOplusXi = -projectionJacobian * rotation_worldToCamera;

    Eigen::Matrix<double, 3, 6> se3Derivative;
    double                      bodyPointX = Xb(0);
    double                      bodyPointY = Xb(1);
    double                      bodyPointZ = Xb(2);

    se3Derivative << 0.0, bodyPointZ, -bodyPointY, 1.0, 0.0, 0.0, -bodyPointZ,
        0.0, bodyPointX, 0.0, 1.0, 0.0, bodyPointY, -bodyPointX, 0.0, 0.0, 0.0,
        1.0;

    _jacobianOplusXj =
        projectionJacobian * rotation_bodyToCamera * se3Derivative;
}

} // namespace core
} // namespace vs_graphs
