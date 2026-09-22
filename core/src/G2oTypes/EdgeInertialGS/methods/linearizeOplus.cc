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

void EdgeInertialGS::linearizeOplus()
{
    const VertexPose     *VP1 = static_cast<const VertexPose *>(_vertices[0]);
    const VertexVelocity *VV1 =
        static_cast<const VertexVelocity *>(_vertices[1]);
    const VertexGyroBias *VG =
        static_cast<const VertexGyroBias *>(_vertices[2]);
    const VertexAccBias  *VA = static_cast<const VertexAccBias *>(_vertices[3]);
    const VertexPose     *VP2 = static_cast<const VertexPose *>(_vertices[4]);
    const VertexVelocity *VV2 =
        static_cast<const VertexVelocity *>(_vertices[5]);
    const VertexGDir  *VGDir = static_cast<const VertexGDir *>(_vertices[6]);
    const VertexScale *VS    = static_cast<const VertexScale *>(_vertices[7]);
    const IMU::Bias    b(VA->estimate()[0],
                      VA->estimate()[1],
                      VA->estimate()[2],
                      VG->estimate()[0],
                      VG->estimate()[1],
                      VG->estimate()[2]);
    const IMU::Bias    db = p_preintegrated->getDeltaBias(b);

    Eigen::Vector3d dbg;
    dbg << db.bwx, db.bwy, db.bwz;

    const Eigen::Matrix3d Rwb1     = VP1->estimate().Rwb;
    const Eigen::Matrix3d Rbw1     = Rwb1.transpose();
    const Eigen::Matrix3d Rwb2     = VP2->estimate().Rwb;
    const Eigen::Matrix3d Rwg      = VGDir->estimate().Rwg;
    Eigen::MatrixXd       Gm       = Eigen::MatrixXd::Zero(3, 2);
    Gm(0, 1)                       = -IMU::GRAVITY_VALUE;
    Gm(1, 0)                       = IMU::GRAVITY_VALUE;
    const double          s        = VS->estimate();
    const Eigen::MatrixXd dGdTheta = Rwg * Gm;
    const Eigen::Matrix3d dR =
        p_preintegrated->getDeltaRotation(b).cast<double>();
    const Eigen::Matrix3d eR    = dR.transpose() * Rbw1 * Rwb2;
    const Eigen::Vector3d er    = LogSO3(eR);
    const Eigen::Matrix3d invJr = InverseRightJacobianSO3(er);

    // Jacobians wrt Pose 1
    _jacobianOplus[0].setZero();
    // rotation
    _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * Rwb2.transpose() * Rwb1;
    _jacobianOplus[0].block<3, 3>(3, 0) = Sophus::SO3d::hat(
        Rbw1 * (s * (VV2->estimate() - VV1->estimate()) - g * dt));
    _jacobianOplus[0].block<3, 3>(6, 0) = Sophus::SO3d::hat(
        Rbw1 * (s * (VP2->estimate().twb - VP1->estimate().twb -
                     VV1->estimate() * dt) -
                0.5 * g * dt * dt));
    // translation
    _jacobianOplus[0].block<3, 3>(6, 3) =
        Eigen::DiagonalMatrix<double, 3>(-s, -s, -s);

    // Jacobians wrt Velocity 1
    _jacobianOplus[1].setZero();
    _jacobianOplus[1].block<3, 3>(3, 0) = -s * Rbw1;
    _jacobianOplus[1].block<3, 3>(6, 0) = -s * Rbw1 * dt;

    // Jacobians wrt Gyro bias
    _jacobianOplus[2].setZero();
    _jacobianOplus[2].block<3, 3>(0, 0) =
        -invJr * eR.transpose() * RightJacobianSO3(JRg * dbg) * JRg;
    _jacobianOplus[2].block<3, 3>(3, 0) = -JVg;
    _jacobianOplus[2].block<3, 3>(6, 0) = -JPg;

    // Jacobians wrt Accelerometer bias
    _jacobianOplus[3].setZero();
    _jacobianOplus[3].block<3, 3>(3, 0) = -JVa;
    _jacobianOplus[3].block<3, 3>(6, 0) = -JPa;

    // Jacobians wrt Pose 2
    _jacobianOplus[4].setZero();
    // rotation
    _jacobianOplus[4].block<3, 3>(0, 0) = invJr;
    // translation
    _jacobianOplus[4].block<3, 3>(6, 3) = s * Rbw1 * Rwb2;

    // Jacobians wrt Velocity 2
    _jacobianOplus[5].setZero();
    _jacobianOplus[5].block<3, 3>(3, 0) = s * Rbw1;

    // Jacobians wrt Gravity direction
    _jacobianOplus[6].setZero();
    _jacobianOplus[6].block<3, 2>(3, 0) = -Rbw1 * dGdTheta * dt;
    _jacobianOplus[6].block<3, 2>(6, 0) = -0.5 * Rbw1 * dGdTheta * dt * dt;

    // Jacobians wrt scale factor
    _jacobianOplus[7].setZero();
    _jacobianOplus[7].block<3, 1>(3, 0) =
        Rbw1 * (VV2->estimate() - VV1->estimate());
    _jacobianOplus[7].block<3, 1>(6, 0) =
        Rbw1 *
        (VP2->estimate().twb - VP1->estimate().twb - VV1->estimate() * dt);
}

} // namespace core
} // namespace vs_graphs
