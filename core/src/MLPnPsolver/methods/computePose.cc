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

/*!
 * @file            computePose.cc
 *
 * @brief           Implements MLPnPsolver::computePose(), declared in
 *                  MLPnPsolver.h.
 */

/*!****************************************************************************
 * Author:   Steffen Urban                                              *
 * Contact:  urbste@gmail.com                                          *
 * License:  Copyright (c) 2016 Steffen Urban, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

#include "MLPnPsolver.h"

#include <Eigen/Sparse>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

MLPnPsolverStatus
    MLPnPsolver::computePose(const BearingVectors      &f_in,
                             const Points3             &p_in,
                             const Covariance3Matrices &covMats_in,
                             const std::vector<int>    &indices_in,
                             TransformationMatrix      &result_inout)
{
    size_t numberCorrespondences = indices_in.size();
    assert(numberCorrespondences > 5);

    bool                         planar = false;
    // compute the nullspace of all vectors
    std::vector<Eigen::MatrixXd> nullspaces(numberCorrespondences);
    Eigen::MatrixXd              points3(3, numberCorrespondences);
    Points3                      points3v(numberCorrespondences);
    Points4                      points4v(numberCorrespondences);
    for (size_t correspondenceIndex = 0;
         correspondenceIndex < numberCorrespondences;
         correspondenceIndex++)
    {
        BearingVector currentBearing = f_in[indices_in[correspondenceIndex]];
        points3.col(correspondenceIndex) =
            p_in[indices_in[correspondenceIndex]];
        // nullspace of right vector
        Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner>
            svd_f(currentBearing.transpose(), Eigen::ComputeFullV);
        nullspaces[correspondenceIndex] = svd_f.matrixV().block(0, 1, 3, 2);
        points3v[correspondenceIndex]   = p_in[indices_in[correspondenceIndex]];
    }

    //////////////////////////////////////
    // 1. test if we have a planar scene
    //////////////////////////////////////

    Eigen::Matrix3d planarTest = points3 * points3.transpose();
    Eigen::FullPivHouseholderQR<Eigen::Matrix3d> rankTest(planarTest);
    Eigen::Matrix3d                              eigenRot;
    eigenRot.setIdentity();

    // if yes -> transform points to new eigen frame
    // if (minEigenVal < 1e-3 || minEigenVal == 0.0)
    // rankTest.setThreshold(1e-10);
    if (rankTest.rank() == 2)
    {
        planar = true;
        // self adjoint is faster and more accurate than general eigen solvers
        // also has closed form solution for 3x3 self-adjoint matrices
        // in addition this solver sorts the eigenvalues in increasing order
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(planarTest);
        eigenRot = eigenSolver.eigenvectors().real();
        eigenRot.transposeInPlace();
        for (size_t correspondenceIndex = 0;
             correspondenceIndex < numberCorrespondences;
             correspondenceIndex++)
            points3.col(correspondenceIndex) =
                eigenRot * points3.col(correspondenceIndex);
    }
    //////////////////////////////////////
    // 2. stochastic model
    //////////////////////////////////////
    Eigen::SparseMatrix<double> P(2 * numberCorrespondences,
                                  2 * numberCorrespondences);
    bool                        shouldUseCovariance = false;
    P.setIdentity(); // standard

    // if we do have covariance information
    // -> fill covariance matrix
    if (covMats_in.size() == numberCorrespondences)
    {
        shouldUseCovariance = true;
        int l               = 0;
        for (size_t correspondenceIndex = 0;
             correspondenceIndex < numberCorrespondences;
             ++correspondenceIndex)
        {
            // invert matrix
            Covariance2Matrix temp =
                nullspaces[correspondenceIndex].transpose() *
                covMats_in[correspondenceIndex] *
                nullspaces[correspondenceIndex];
            temp                     = temp.inverse().eval();
            P.coeffRef(l, l)         = temp(0, 0);
            P.coeffRef(l, l + 1)     = temp(0, 1);
            P.coeffRef(l + 1, l)     = temp(1, 0);
            P.coeffRef(l + 1, l + 1) = temp(1, 1);
            l += 2;
        }
    }

    //////////////////////////////////////
    // 3. fill the design matrix A
    //////////////////////////////////////
    const int       rowsA = 2 * numberCorrespondences;
    int             colsA = 12;
    Eigen::MatrixXd A;
    if (planar)
    {
        colsA = 9;
        A     = Eigen::MatrixXd(rowsA, 9);
    }
    else
        A = Eigen::MatrixXd(rowsA, 12);
    A.setZero();

    // fill design matrix
    if (planar)
    {
        for (size_t correspondenceIndex = 0;
             correspondenceIndex < numberCorrespondences;
             ++correspondenceIndex)
        {
            Point3 point3Current = points3.col(correspondenceIndex);

            // r12
            A(2 * correspondenceIndex, 0) =
                nullspaces[correspondenceIndex](0, 0) * point3Current[1];
            A(2 * correspondenceIndex + 1, 0) =
                nullspaces[correspondenceIndex](0, 1) * point3Current[1];
            // r13
            A(2 * correspondenceIndex, 1) =
                nullspaces[correspondenceIndex](0, 0) * point3Current[2];
            A(2 * correspondenceIndex + 1, 1) =
                nullspaces[correspondenceIndex](0, 1) * point3Current[2];
            // r22
            A(2 * correspondenceIndex, 2) =
                nullspaces[correspondenceIndex](1, 0) * point3Current[1];
            A(2 * correspondenceIndex + 1, 2) =
                nullspaces[correspondenceIndex](1, 1) * point3Current[1];
            // r23
            A(2 * correspondenceIndex, 3) =
                nullspaces[correspondenceIndex](1, 0) * point3Current[2];
            A(2 * correspondenceIndex + 1, 3) =
                nullspaces[correspondenceIndex](1, 1) * point3Current[2];
            // r32
            A(2 * correspondenceIndex, 4) =
                nullspaces[correspondenceIndex](2, 0) * point3Current[1];
            A(2 * correspondenceIndex + 1, 4) =
                nullspaces[correspondenceIndex](2, 1) * point3Current[1];
            // r33
            A(2 * correspondenceIndex, 5) =
                nullspaces[correspondenceIndex](2, 0) * point3Current[2];
            A(2 * correspondenceIndex + 1, 5) =
                nullspaces[correspondenceIndex](2, 1) * point3Current[2];
            // t1
            A(2 * correspondenceIndex, 6) =
                nullspaces[correspondenceIndex](0, 0);
            A(2 * correspondenceIndex + 1, 6) =
                nullspaces[correspondenceIndex](0, 1);
            // t2
            A(2 * correspondenceIndex, 7) =
                nullspaces[correspondenceIndex](1, 0);
            A(2 * correspondenceIndex + 1, 7) =
                nullspaces[correspondenceIndex](1, 1);
            // t3
            A(2 * correspondenceIndex, 8) =
                nullspaces[correspondenceIndex](2, 0);
            A(2 * correspondenceIndex + 1, 8) =
                nullspaces[correspondenceIndex](2, 1);
        }
    }
    else
    {
        for (size_t correspondenceIndex = 0;
             correspondenceIndex < numberCorrespondences;
             ++correspondenceIndex)
        {
            Point3 point3Current = points3.col(correspondenceIndex);

            // r11
            A(2 * correspondenceIndex, 0) =
                nullspaces[correspondenceIndex](0, 0) * point3Current[0];
            A(2 * correspondenceIndex + 1, 0) =
                nullspaces[correspondenceIndex](0, 1) * point3Current[0];
            // r12
            A(2 * correspondenceIndex, 1) =
                nullspaces[correspondenceIndex](0, 0) * point3Current[1];
            A(2 * correspondenceIndex + 1, 1) =
                nullspaces[correspondenceIndex](0, 1) * point3Current[1];
            // r13
            A(2 * correspondenceIndex, 2) =
                nullspaces[correspondenceIndex](0, 0) * point3Current[2];
            A(2 * correspondenceIndex + 1, 2) =
                nullspaces[correspondenceIndex](0, 1) * point3Current[2];
            // r21
            A(2 * correspondenceIndex, 3) =
                nullspaces[correspondenceIndex](1, 0) * point3Current[0];
            A(2 * correspondenceIndex + 1, 3) =
                nullspaces[correspondenceIndex](1, 1) * point3Current[0];
            // r22
            A(2 * correspondenceIndex, 4) =
                nullspaces[correspondenceIndex](1, 0) * point3Current[1];
            A(2 * correspondenceIndex + 1, 4) =
                nullspaces[correspondenceIndex](1, 1) * point3Current[1];
            // r23
            A(2 * correspondenceIndex, 5) =
                nullspaces[correspondenceIndex](1, 0) * point3Current[2];
            A(2 * correspondenceIndex + 1, 5) =
                nullspaces[correspondenceIndex](1, 1) * point3Current[2];
            // r31
            A(2 * correspondenceIndex, 6) =
                nullspaces[correspondenceIndex](2, 0) * point3Current[0];
            A(2 * correspondenceIndex + 1, 6) =
                nullspaces[correspondenceIndex](2, 1) * point3Current[0];
            // r32
            A(2 * correspondenceIndex, 7) =
                nullspaces[correspondenceIndex](2, 0) * point3Current[1];
            A(2 * correspondenceIndex + 1, 7) =
                nullspaces[correspondenceIndex](2, 1) * point3Current[1];
            // r33
            A(2 * correspondenceIndex, 8) =
                nullspaces[correspondenceIndex](2, 0) * point3Current[2];
            A(2 * correspondenceIndex + 1, 8) =
                nullspaces[correspondenceIndex](2, 1) * point3Current[2];
            // t1
            A(2 * correspondenceIndex, 9) =
                nullspaces[correspondenceIndex](0, 0);
            A(2 * correspondenceIndex + 1, 9) =
                nullspaces[correspondenceIndex](0, 1);
            // t2
            A(2 * correspondenceIndex, 10) =
                nullspaces[correspondenceIndex](1, 0);
            A(2 * correspondenceIndex + 1, 10) =
                nullspaces[correspondenceIndex](1, 1);
            // t3
            A(2 * correspondenceIndex, 11) =
                nullspaces[correspondenceIndex](2, 0);
            A(2 * correspondenceIndex + 1, 11) =
                nullspaces[correspondenceIndex](2, 1);
        }
    }

    //////////////////////////////////////
    // 4. solve least squares
    //////////////////////////////////////
    Eigen::MatrixXd AtPA;
    if (shouldUseCovariance)
        AtPA = A.transpose() * P *
               A; // setting up the full normal equations seems to be unstable
    else
        AtPA = A.transpose() * A;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(AtPA, Eigen::ComputeFullV);
    Eigen::MatrixXd                   result1 = svd_A.matrixV().col(colsA - 1);

    ////////////////////////////////
    // now we treat the results differently,
    // depending on the scene (planar or not)
    ////////////////////////////////
    RotationMatrix    Rout;
    TranslationVector tout;
    if (planar) // planar case
    {
        RotationMatrix temporary;
        // until now, we only estimated
        // row one and two of the transposed rotation matrix
        temporary << 0.0, result1(0, 0), result1(1, 0), 0.0, result1(2, 0),
            result1(3, 0), 0.0, result1(4, 0), result1(5, 0);
        // row 3
        temporary.col(0) = temporary.col(1).cross(temporary.col(2));
        temporary.transposeInPlace();

        double scale = 1.0 / std::sqrt(std::abs(temporary.col(1).norm() *
                                                temporary.col(2).norm()));
        // find best rotation matrix in frobenius sense
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(temporary,
                                                     Eigen::ComputeFullU |
                                                         Eigen::ComputeFullV);
        RotationMatrix                    rout1 =
            svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();
        // test if we found a good rotation matrix
        if (rout1.determinant() < 0)
            rout1 *= -1.0;
        // rotate this matrix back using the eigen frame
        rout1 = eigenRot.transpose() * rout1;

        TranslationVector t =
            scale *
            TranslationVector(result1(6, 0), result1(7, 0), result1(8, 0));
        rout1.transposeInPlace();
        rout1 *= -1;
        if (rout1.determinant() < 0.0)
            rout1.col(2) *= -1;
        // now we have to find the best out of 4 combinations
        RotationMatrix R1, R2;
        R1.col(0) = rout1.col(0);
        R1.col(1) = rout1.col(1);
        R1.col(2) = rout1.col(2);
        R2.col(0) = -rout1.col(0);
        R2.col(1) = -rout1.col(1);
        R2.col(2) = rout1.col(2);

        std::vector<TransformationMatrix,
                    Eigen::aligned_allocator<TransformationMatrix>>
            Ts(4);
        Ts[0].block<3, 3>(0, 0) = R1;
        Ts[0].block<3, 1>(0, 3) = t;
        Ts[1].block<3, 3>(0, 0) = R1;
        Ts[1].block<3, 1>(0, 3) = -t;
        Ts[2].block<3, 3>(0, 0) = R2;
        Ts[2].block<3, 1>(0, 3) = t;
        Ts[3].block<3, 3>(0, 0) = R2;
        Ts[3].block<3, 1>(0, 3) = -t;

        std::vector<double> normValue(4);
        for (int correspondenceIndex = 0; correspondenceIndex < 4;
             ++correspondenceIndex)
        {
            Point3 reproPoint;
            double norms = 0.0;
            for (int p = 0; p < 6; ++p)
            {
                reproPoint =
                    Ts[correspondenceIndex].block<3, 3>(0, 0) * points3v[p] +
                    Ts[correspondenceIndex].block<3, 1>(0, 3);
                reproPoint = reproPoint / reproPoint.norm();
                norms += (1.0 - reproPoint.transpose() * f_in[indices_in[p]]);
            }
            normValue[correspondenceIndex] = norms;
        }
        std::vector<double>::iterator findMinimumRepro =
            std::min_element(std::begin(normValue), std::end(normValue));
        int minimumReprojectionIndex =
            std::distance(std::begin(normValue), findMinimumRepro);
        Rout = Ts[minimumReprojectionIndex].block<3, 3>(0, 0);
        tout = Ts[minimumReprojectionIndex].block<3, 1>(0, 3);
    }
    else // non-planar
    {
        RotationMatrix temporary;
        temporary << result1(0, 0), result1(3, 0), result1(6, 0), result1(1, 0),
            result1(4, 0), result1(7, 0), result1(2, 0), result1(5, 0),
            result1(8, 0);
        // get the scale
        double scale = 1.0 / std::pow(std::abs(temporary.col(0).norm() *
                                               temporary.col(1).norm() *
                                               temporary.col(2).norm()),
                                      1.0 / 3.0);
        // double scale = 1.0 / std::sqrt(std::abs(tmp.col(0).norm() *
        // tmp.col(1).norm()));
        //  find best rotation matrix in frobenius sense
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(temporary,
                                                     Eigen::ComputeFullU |
                                                         Eigen::ComputeFullV);
        Rout = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();
        // test if we found a good rotation matrix
        if (Rout.determinant() < 0)
            Rout *= -1.0;
        // scale translation
        tout = Rout * (scale * TranslationVector(result1(9, 0),
                                                 result1(10, 0),
                                                 result1(11, 0)));

        // find correct direction in terms of reprojection error, just take the
        // first 6 correspondences
        std::vector<double> error(2);
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
            Ts(2);
        for (int s = 0; s < 2; ++s)
        {
            error[s]                = 0.0;
            Ts[s]                   = Eigen::Matrix4d::Identity();
            Ts[s].block<3, 3>(0, 0) = Rout;
            if (s == 0)
                Ts[s].block<3, 1>(0, 3) = tout;
            else
                Ts[s].block<3, 1>(0, 3) = -tout;
            Ts[s] = Ts[s].inverse().eval();
            for (int p = 0; p < 6; ++p)
            {
                BearingVector v = Ts[s].block<3, 3>(0, 0) * points3v[p] +
                                  Ts[s].block<3, 1>(0, 3);
                v = v / v.norm();
                error[s] += (1.0 - v.transpose() * f_in[indices_in[p]]);
            }
        }
        if (error[0] < error[1])
            tout = Ts[0].block<3, 1>(0, 3);
        else
            tout = Ts[1].block<3, 1>(0, 3);
        Rout = Ts[0].block<3, 3>(0, 0);
    }

    //////////////////////////////////////
    // 5. gauss newton
    //////////////////////////////////////
    RodriguesVector omega{};
    if (rot2rodrigues(Rout, omega) !=
        MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rot2rodrigues returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::VectorXd minx(6);
    minx[0] = omega[0];
    minx[1] = omega[1];
    minx[2] = omega[2];
    minx[3] = tout[0];
    minx[4] = tout[1];
    minx[5] = tout[2];

    if (mlpnp_gn(minx, points3v, nullspaces, P, shouldUseCovariance) !=
        MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: mlpnp_gn returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    Eigen::Matrix3d rotation2{};
    if (rodrigues2rot(RodriguesVector(minx[0], minx[1], minx[2]), rotation2) !=
        MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rodrigues2rot returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    Rout = rotation2;
    tout = TranslationVector(minx[3], minx[4], minx[5]);
    // result inverse as opengv uses this convention
    result_inout.block<3, 3>(0, 0) = Rout;
    result_inout.block<3, 1>(0, 3) = tout;

    return MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
