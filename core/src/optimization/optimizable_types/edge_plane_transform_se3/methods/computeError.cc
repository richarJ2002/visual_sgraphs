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
 * @file            computeError.cc
 *
 * @brief           Implements EdgePlaneTransformSE3::computeError().
 */

#include "optimization/optimizable_types/edge_plane_transform_se3/edge_plane_transform_se3.h"

namespace vs_graphs::core::optimizable_types::edge_plane_transform_se3
{

void EdgePlaneTransformSE3::computeError()
{
        const g2o::VertexSE3Expmap *v =
            static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::SE3Quat    estimate = v->estimate();
        const Eigen::Matrix3d R        = estimate.rotation().toRotationMatrix();
        const Eigen::Vector3d t        = estimate.translation();

        /*!
         * Plane.cc:transformPlaneEquation law (scale fixed at 1 here):
         * n' = R n ; d' = sigma*d - n'^T t
         */
        const Eigen::Vector3d predCount =
            static_cast<double>(_measurement.sigma) * (R * _measurement.n_A);
        const double predictedDistance =
            static_cast<double>(_measurement.sigma) * _measurement.d_A -
            predCount.dot(t);

        /*!
         * Minimal rotation vector (axis-angle) that rotates n_pred onto
         * n_B, i.e. Log_SO3(n_pred, n_B).
         */
        const Eigen::Vector3d &n_B      = _measurement.n_B;
        const Eigen::Vector3d  axis     = predCount.cross(n_B);
        const double           sinAngle = axis.norm();
        const double           cosAngle =
            std::max(-1.0, std::min(1.0, predCount.dot(n_B)));
        const double    angle     = std::atan2(sinAngle, cosAngle);
        Eigen::Vector3d logVector = Eigen::Vector3d::Zero();
        if (sinAngle > 1e-12)
        {
            logVector = (axis / sinAngle) * angle;
        }
        else if (cosAngle < 0.0)
        {
            /*!
             * Antipodal singularity: axis/sinAngle is 0/0 exactly where
             * the true orientation error is at its maximum (angle ~ pi),
             * not zero -- leaving logVec at zero here would silently
             * mask a 180 degree misalignment as a perfect fit. Any unit
             * vector orthogonal to n_pred is a valid rotation axis at
             * this isolated point; unitOrthogonal() picks one
             * deterministically.
             */
            logVector = predCount.unitOrthogonal() * angle;
        }

        /* Orthonormal 2D basis for the tangent plane at n_B (B_B). */
        const Eigen::Vector3d referenceAxis = (std::abs(n_B.x()) <= 0.9)
                                                  ? Eigen::Vector3d::UnitX()
                                                  : Eigen::Vector3d::UnitY();
        const Eigen::Vector3d axisU = n_B.cross(referenceAxis).normalized();
        const Eigen::Vector3d axisV = n_B.cross(axisU).normalized();

        _error[0] = axisU.dot(logVector);
        _error[1] = axisV.dot(logVector);
        _error[2] = predictedDistance - _measurement.d_B;
    }

} /* namespace vs_graphs::core::optimizable_types::edge_plane_transform_se3 */
