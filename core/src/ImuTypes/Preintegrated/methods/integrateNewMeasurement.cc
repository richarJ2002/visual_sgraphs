/*!
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details:
 * https://www.gnu.org/licenses/
 */

#include "ImuTypes.h"

namespace vs_graphs
{
namespace core
{
namespace IMU
{

void Preintegrated::integrateNewMeasurement(const Eigen::Vector3f &acceleration,
                                            const Eigen::Vector3f &angVel,
                                            const float           &dt)
{
    mvMeasurements.push_back(Integrable(acceleration, angVel, dt));

    // Position is updated firstly, as it depends on previously computed
    // velocity and rotation. Velocity is updated secondly, as it depends on
    // previously computed rotation. Rotation is the last to be updated.

    // Matrices to compute covariance
    Eigen::Matrix<float, 9, 9> A;
    A.setIdentity();
    Eigen::Matrix<float, 9, 6> B;
    B.setZero();

    Eigen::Vector3f acc, accW;
    acc << acceleration(0) - b.bax, acceleration(1) - b.bay,
        acceleration(2) - b.baz;
    accW << angVel(0) - b.bwx, angVel(1) - b.bwy, angVel(2) - b.bwz;

    avgA = (dT * avgA + dR * acc * dt) / (dT + dt);
    avgW = (dT * avgW + accW * dt) / (dT + dt);

    // Update delta position dP and velocity dV (rely on no-updated delta
    // rotation)
    dP = dP + dV * dt + 0.5f * dR * acc * dt * dt;
    dV = dV + dR * acc * dt;

    // Compute velocity and position parts of matrices A and B (rely on
    // non-updated delta rotation)
    Eigen::Matrix<float, 3, 3> Wacc = Sophus::SO3f::hat(acc);

    A.block<3, 3>(3, 0) = -dR * dt * Wacc;
    A.block<3, 3>(6, 0) = -0.5f * dR * dt * dt * Wacc;
    A.block<3, 3>(6, 3) = Eigen::DiagonalMatrix<float, 3>(dt, dt, dt);
    B.block<3, 3>(3, 3) = dR * dt;
    B.block<3, 3>(6, 3) = 0.5f * dR * dt * dt;

    // Update position and velocity jacobians wrt bias correction
    JPa = JPa + JVa * dt - 0.5f * dR * dt * dt;
    JPg = JPg + JVg * dt - 0.5f * dR * dt * dt * Wacc * JRg;
    JVa = JVa - dR * dt;
    JVg = JVg - dR * dt * Wacc * JRg;

    // Update delta rotation
    IntegratedRotation dRi(angVel, b, dt);
    dR = NormalizeRotation(dR * dRi.deltaR);

    // Compute rotation parts of matrices A and B
    A.block<3, 3>(0, 0) = dRi.deltaR.transpose();
    B.block<3, 3>(0, 0) = dRi.rightJ * dt;

    // Update covariance
    C.block<9, 9>(0, 0) =
        A * C.block<9, 9>(0, 0) * A.transpose() + B * Nga * B.transpose();
    C.block<6, 6>(9, 9) += NgaWalk;

    // Update rotation jacobian wrt bias correction
    JRg = dRi.deltaR.transpose() * JRg - dRi.rightJ * dt;

    // Total integrated time
    dT += dt;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
