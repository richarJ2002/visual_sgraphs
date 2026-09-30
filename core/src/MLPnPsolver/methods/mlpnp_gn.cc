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
 * @file            mlpnp_gn.cc
 *
 * @brief           Implements MLPnPsolver::mlpnp_gn(), declared in
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
    MLPnPsolver::mlpnp_gn(Eigen::VectorXd                    &x_inout,
                          const Points3                      &points_in,
                          const std::vector<Eigen::MatrixXd> &nullspaces_in,
                          const Eigen::SparseMatrix<double>   Kll_in,
                          bool shouldUseCovariance_in)
{
    const int observationCount = points_in.size();
    const int unknownCount     = 6;
    // check redundancy
    assert((2 * observationCount - unknownCount) > 0);

    // =============
    // set all matrices up
    // =============

    Eigen::VectorXd r(2 * observationCount);
    Eigen::VectorXd rd(2 * observationCount);
    Eigen::MatrixXd Jac(2 * observationCount, unknownCount);
    Eigen::VectorXd g(unknownCount, 1);
    Eigen::VectorXd dx(unknownCount, 1); // result vector

    Jac.setZero();
    r.setZero();
    dx.setZero();
    g.setZero();

    int       it_cnt    = 0;
    bool      stop      = false;
    const int maximumIt = 5;
    double    epsP      = 1e-5;

    Eigen::MatrixXd jacTsKll;
    Eigen::MatrixXd A;
    // solve simple gradient descent
    while (it_cnt < maximumIt && !stop)
    {
        if (mlpnp_residuals_and_jacs(x_inout,
                                     points_in,
                                     nullspaces_in,
                                     r,
                                     Jac,
                                     true) !=
            MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("vs_graphs"),
                "%s: mlpnp_residuals_and_jacs returned a failure status "
                "although it cannot fail; continuing as before.",
                __func__);
        }

        if (shouldUseCovariance_in)
            jacTsKll = Jac.transpose() * Kll_in;
        else
            jacTsKll = Jac.transpose();

        A = jacTsKll * Jac;

        // get system matrix
        g = jacTsKll * r;

        // solve
        Eigen::LDLT<Eigen::MatrixXd> chol(A);
        dx = chol.solve(g);
        // this is to prevent the solution from falling into a wrong minimum
        // if the linear estimate is spurious
        if (dx.array().abs().maxCoeff() > 5.0 ||
            dx.array().abs().minCoeff() > 1.0)
            break;
        // observation update
        Eigen::MatrixXd dl = Jac * dx;
        if (dl.array().abs().maxCoeff() < epsP)
        {
            stop    = true;
            x_inout = x_inout - dx;
            break;
        }
        else
            x_inout = x_inout - dx;

        ++it_cnt;
    } // while
    // result

    return MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
