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

#include "Sim3Solver.h"

#include <cmath>
#include <opencv2/core/core.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

#include "KeyFrame.h"
#include "ORBmatcher.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace vs_graphs
{
namespace core
{

Sim3SolverStatus Sim3Solver::computeSim3(Eigen::Matrix3f &P1_inout,
                                         Eigen::Matrix3f &P2_inout)
{
    // Custom implementation of:
    // Horn 1987, Closed-form solution of absolute orientataion using unit
    // quaternions

    // Step 1: Centroid and relative coordinates

    Eigen::Matrix3f Pr1; // Relative coordinates to centroid (set 1)
    Eigen::Matrix3f Pr2; // Relative coordinates to centroid (set 2)
    Eigen::Vector3f O1;  // Centroid of P1
    Eigen::Vector3f O2;  // Centroid of P2

    if (computeCentroid(P1_inout, Pr1, O1) !=
        Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeCentroid returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    if (computeCentroid(P2_inout, Pr2, O2) !=
        Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeCentroid returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    // Step 2: Compute M matrix

    Eigen::Matrix3f M = Pr2 * Pr1.transpose();

    // Step 3: Compute N matrix
    double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

    Eigen::Matrix4f N;

    N11 = M(0, 0) + M(1, 1) + M(2, 2);
    N12 = M(1, 2) - M(2, 1);
    N13 = M(2, 0) - M(0, 2);
    N14 = M(0, 1) - M(1, 0);
    N22 = M(0, 0) - M(1, 1) - M(2, 2);
    N23 = M(0, 1) + M(1, 0);
    N24 = M(2, 0) + M(0, 2);
    N33 = -M(0, 0) + M(1, 1) - M(2, 2);
    N34 = M(1, 2) + M(2, 1);
    N44 = -M(0, 0) - M(1, 1) + M(2, 2);

    N << N11, N12, N13, N14, N12, N22, N23, N24, N13, N23, N33, N34, N14, N24,
        N34, N44;

    // Step 4: Eigenvector of the highest eigenvalue
    Eigen::EigenSolver<Eigen::Matrix4f> eigSolver;
    eigSolver.compute(N);

    Eigen::Vector4f eval = eigSolver.eigenvalues().real();
    Eigen::Matrix4f evec =
        eigSolver.eigenvectors()
            .real(); // evec[0] is the quaternion of the desired rotation

    int maximumIndex; // should be zero
    eval.maxCoeff(&maximumIndex);

    Eigen::Vector3f vector = evec.block<3, 1>(
        1,
        maximumIndex); // extract imaginary part of the quaternion (sin*axis)

    // Rotation angle. sin is the norm of the imaginary part, cos is the real
    // part
    double angle = atan2(vector.norm(), evec(0, maximumIndex));

    vector =
        2 * angle * vector /
        vector
            .norm(); // Angle-axis representation. quaternion angle is the half
    mR12i = Sophus::SO3f::exp(vector).matrix();

    // Step 5: Rotate set 2
    Eigen::Matrix3f P3 = mR12i * Pr2;

    // Step 6: Scale

    if (!isScaleFixed)
    {
        cv::Mat cvMat{};
        if (utils::converter::Converter::toCvMat(Pr1, cvMat) !=
            utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: toCvMat returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        cv::Mat cvMat2{};
        if (utils::converter::Converter::toCvMat(P3, cvMat2) !=
            utils::converter::ConverterStatus::CONVERTER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: toCvMat returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        double cvnom = cvMat.dot(cvMat2);
        double nom   = (Pr1.array() * P3.array()).sum();
        if (abs(nom - cvnom) > 1e-3)
            std::cout << "sim3 solver: " << abs(nom - cvnom) << std::endl
                      << nom << std::endl;
        Eigen::Array<float, 3, 3> aux_P3;
        aux_P3     = P3.array() * P3.array();
        double den = aux_P3.sum();

        ms12i = nom / den;
    }
    else
        ms12i = 1.0f;

    // Step 7: Translation
    mt12i = O1 - ms12i * mR12i * O2;

    // Step 8: Transformation

    // Step 8.1 T12
    mT12i.setIdentity();

    Eigen::Matrix3f sR      = ms12i * mR12i;
    mT12i.block<3, 3>(0, 0) = sR;
    mT12i.block<3, 1>(0, 3) = mt12i;

    // Step 8.2 T21
    mT21i.setIdentity();
    Eigen::Matrix3f rinv = (1.0 / ms12i) * mR12i.transpose();

    // sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
    mT21i.block<3, 3>(0, 0) = rinv;

    Eigen::Vector3f tinv    = -rinv * mt12i;
    mT21i.block<3, 1>(0, 3) = tinv;

    return Sim3SolverStatus::SIM3_SOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
