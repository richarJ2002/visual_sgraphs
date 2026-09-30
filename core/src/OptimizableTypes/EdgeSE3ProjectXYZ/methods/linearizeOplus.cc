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
 * @brief           Implements EdgeSE3ProjectXYZ::linearizeOplus(), declared in
 *                  OptimizableTypes.h.
 */

#include "OptimizableTypes.h"

namespace vs_graphs
{
namespace core
{

void EdgeSE3ProjectXYZ::linearizeOplus()
{
    g2o::VertexSE3Expmap *p_poseVertex =
        static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat            poseTransform(p_poseVertex->estimate());
    g2o::VertexSBAPointXYZ *p_pointVertex =
        static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);
    Eigen::Vector3d pointPosition            = p_pointVertex->estimate();
    Eigen::Vector3d transformedPointPosition = poseTransform.map(pointPosition);

    double transformedX = transformedPointPosition[0];
    double transformedY = transformedPointPosition[1];
    double transformedZ = transformedPointPosition[2];

    // Materialize eagerly: computeProjectionJacobian() returns by value,
    // so `auto` would capture a lazy expression referencing a dead
    // temporary (stack-use-after-scope under vectorized evaluation).
    const Eigen::Matrix<double, 2, 3> projectionJacobian =
        -p_camera->computeProjectionJacobian(transformedPointPosition);

    _jacobianOplusXi =
        projectionJacobian * poseTransform.rotation().toRotationMatrix();

    Eigen::Matrix<double, 3, 6> se3Derivative;
    se3Derivative << 0.f, transformedZ, -transformedY, 1.f, 0.f, 0.f,
        -transformedZ, 0.f, transformedX, 0.f, 1.f, 0.f, transformedY,
        -transformedX, 0.f, 0.f, 0.f, 1.f;

    _jacobianOplusXj = projectionJacobian * se3Derivative;
}

} // namespace core
} // namespace vs_graphs
