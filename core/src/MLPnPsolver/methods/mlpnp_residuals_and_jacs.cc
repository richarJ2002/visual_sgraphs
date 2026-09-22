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

namespace vs_graphs
{
namespace core
{

void MLPnPsolver::mlpnp_residuals_and_jacs(
    const Eigen::VectorXd              &x,
    const points_t                     &pts,
    const std::vector<Eigen::MatrixXd> &nullspaces,
    Eigen::VectorXd                    &r,
    Eigen::MatrixXd                    &fjac,
    bool                                getJacs)
{
    rodrigues_t   w(x[0], x[1], x[2]);
    translation_t T(x[3], x[4], x[5]);

    rotation_t R  = rodrigues2rot(w);
    int        ii = 0;

    Eigen::MatrixXd jacs(2, 6);

    for (size_t i = 0; i < pts.size(); ++i)
    {
        Eigen::Vector3d ptCam = R * pts[i] + T;
        ptCam /= ptCam.norm();

        r[ii]     = nullspaces[i].col(0).transpose() * ptCam;
        r[ii + 1] = nullspaces[i].col(1).transpose() * ptCam;
        if (getJacs)
        {
            // jacs
            mlpnpJacs(pts[i],
                      nullspaces[i].col(0),
                      nullspaces[i].col(1),
                      w,
                      T,
                      jacs);

            // r
            fjac(ii, 0) = jacs(0, 0);
            fjac(ii, 1) = jacs(0, 1);
            fjac(ii, 2) = jacs(0, 2);

            fjac(ii, 3) = jacs(0, 3);
            fjac(ii, 4) = jacs(0, 4);
            fjac(ii, 5) = jacs(0, 5);
            // s
            fjac(ii + 1, 0) = jacs(1, 0);
            fjac(ii + 1, 1) = jacs(1, 1);
            fjac(ii + 1, 2) = jacs(1, 2);

            fjac(ii + 1, 3) = jacs(1, 3);
            fjac(ii + 1, 4) = jacs(1, 4);
            fjac(ii + 1, 5) = jacs(1, 5);
        }
        ii += 2;
    }
}

} // namespace core
} // namespace vs_graphs
