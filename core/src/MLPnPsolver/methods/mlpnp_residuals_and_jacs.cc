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
 * @file            mlpnp_residuals_and_jacs.cc
 *
 * @brief           Implements MLPnPsolver::mlpnp_residuals_and_jacs(), declared
 *                  in MLPnPsolver.h.
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

MLPnPsolverStatus MLPnPsolver::mlpnp_residuals_and_jacs(
    const Eigen::VectorXd              &x_in,
    const Points3                      &points_in,
    const std::vector<Eigen::MatrixXd> &nullspaces_in,
    Eigen::VectorXd                    &r_inout,
    Eigen::MatrixXd                    &fjac_in,
    bool                                getJacs_in)
{
    RodriguesVector   w(x_in[0], x_in[1], x_in[2]);
    TranslationVector T(x_in[3], x_in[4], x_in[5]);

    RotationMatrix R{};
    if (rodrigues2rot(w, R) != MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: rodrigues2rot returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    int ii = 0;

    Eigen::MatrixXd jacs(2, 6);

    for (size_t pointIndex = 0; pointIndex < points_in.size(); ++pointIndex)
    {
        Eigen::Vector3d pointCamera = R * points_in[pointIndex] + T;
        pointCamera /= pointCamera.norm();

        r_inout[ii] =
            nullspaces_in[pointIndex].col(0).transpose() * pointCamera;
        r_inout[ii + 1] =
            nullspaces_in[pointIndex].col(1).transpose() * pointCamera;
        if (getJacs_in)
        {
            // jacs
            if (mlpnpJacs(points_in[pointIndex],
                          nullspaces_in[pointIndex].col(0),
                          nullspaces_in[pointIndex].col(1),
                          w,
                          T,
                          jacs) !=
                MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: mlpnpJacs returned a failure status although "
                             "it cannot fail; continuing as before.",
                             __func__);
            }

            // r
            fjac_in(ii, 0) = jacs(0, 0);
            fjac_in(ii, 1) = jacs(0, 1);
            fjac_in(ii, 2) = jacs(0, 2);

            fjac_in(ii, 3) = jacs(0, 3);
            fjac_in(ii, 4) = jacs(0, 4);
            fjac_in(ii, 5) = jacs(0, 5);
            // s
            fjac_in(ii + 1, 0) = jacs(1, 0);
            fjac_in(ii + 1, 1) = jacs(1, 1);
            fjac_in(ii + 1, 2) = jacs(1, 2);

            fjac_in(ii + 1, 3) = jacs(1, 3);
            fjac_in(ii + 1, 4) = jacs(1, 4);
            fjac_in(ii + 1, 5) = jacs(1, 5);
        }
        ii += 2;
    }

    return MLPnPsolverStatus::MLPN_PSOLVER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
