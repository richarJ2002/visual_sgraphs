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

void Preintegrated::integrateNewMeasurement(
    const Eigen::Vector3f &acceleration_in,
    const Eigen::Vector3f &angularVelocity_in,
    const float           &deltaTime_in)
{
    measurements.push_back(
        Integrable(acceleration_in, angularVelocity_in, deltaTime_in));

    // Position is updated firstly, as it depends on previously computed
    // velocity and rotation. Velocity is updated secondly, as it depends on
    // previously computed rotation. Rotation is the last to be updated.

    // Matrices to compute covariance
    Eigen::Matrix<float, 9, 9> stateTransitionMatrix;
    stateTransitionMatrix.setIdentity();
    Eigen::Matrix<float, 9, 6> noiseJacobianMatrix;
    noiseJacobianMatrix.setZero();

    Eigen::Vector3f debiasedAcceleration, debiasedAngularVelocity;
    debiasedAcceleration << acceleration_in(0) - b.bax,
        acceleration_in(1) - b.bay, acceleration_in(2) - b.baz;
    debiasedAngularVelocity << angularVelocity_in(0) - b.bwx,
        angularVelocity_in(1) - b.bwy, angularVelocity_in(2) - b.bwz;

    avgA = (dT * avgA + dR * debiasedAcceleration * deltaTime_in) /
           (dT + deltaTime_in);
    avgW = (dT * avgW + debiasedAngularVelocity * deltaTime_in) /
           (dT + deltaTime_in);

    // Update delta position dP and velocity dV (rely on no-updated delta
    // rotation)
    dP = dP + dV * deltaTime_in +
         0.5f * dR * debiasedAcceleration * deltaTime_in * deltaTime_in;
    dV = dV + dR * debiasedAcceleration * deltaTime_in;

    // Compute velocity and position parts of matrices A and B (rely on
    // non-updated delta rotation)
    Eigen::Matrix<float, 3, 3> accelerationSkewMatrix =
        Sophus::SO3f::hat(debiasedAcceleration);

    stateTransitionMatrix.block<3, 3>(3, 0) =
        -dR * deltaTime_in * accelerationSkewMatrix;
    stateTransitionMatrix.block<3, 3>(6, 0) =
        -0.5f * dR * deltaTime_in * deltaTime_in * accelerationSkewMatrix;
    stateTransitionMatrix.block<3, 3>(6, 3) =
        Eigen::DiagonalMatrix<float, 3>(deltaTime_in,
                                        deltaTime_in,
                                        deltaTime_in);
    noiseJacobianMatrix.block<3, 3>(3, 3) = dR * deltaTime_in;
    noiseJacobianMatrix.block<3, 3>(6, 3) =
        0.5f * dR * deltaTime_in * deltaTime_in;

    // Update position and velocity jacobians wrt bias correction
    JPa = JPa + JVa * deltaTime_in - 0.5f * dR * deltaTime_in * deltaTime_in;
    JPg =
        JPg + JVg * deltaTime_in -
        0.5f * dR * deltaTime_in * deltaTime_in * accelerationSkewMatrix * JRg;
    JVa = JVa - dR * deltaTime_in;
    JVg = JVg - dR * deltaTime_in * accelerationSkewMatrix * JRg;

    // Update delta rotation
    IntegratedRotation integratedRotation(angularVelocity_in, b, deltaTime_in);
    dR = normalizeRotation(dR * integratedRotation.deltaR);

    // Compute rotation parts of matrices A and B
    stateTransitionMatrix.block<3, 3>(0, 0) =
        integratedRotation.deltaR.transpose();
    noiseJacobianMatrix.block<3, 3>(0, 0) =
        integratedRotation.rightJ * deltaTime_in;

    // Update covariance
    C.block<9, 9>(0, 0) =
        stateTransitionMatrix * C.block<9, 9>(0, 0) *
            stateTransitionMatrix.transpose() +
        noiseJacobianMatrix * Nga * noiseJacobianMatrix.transpose();
    C.block<6, 6>(9, 9) += NgaWalk;

    // Update rotation jacobian wrt bias correction
    JRg = integratedRotation.deltaR.transpose() * JRg -
          integratedRotation.rightJ * deltaTime_in;

    // Total integrated time
    dT += deltaTime_in;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
