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

void MLPnPsolver::checkInliers()
{
    inlierCount = 0;

    for (int i = 0; i < N; i++)
    {
        point_t     p = points3Dw[i];
        cv::Point3f P3Dw(p(0), p(1), p(2));
        cv::Point2f P2D = points2D[i];

        float xc = mRi[0][0] * P3Dw.x + mRi[0][1] * P3Dw.y +
                   mRi[0][2] * P3Dw.z + mti[0];
        float yc = mRi[1][0] * P3Dw.x + mRi[1][1] * P3Dw.y +
                   mRi[1][2] * P3Dw.z + mti[1];
        float zc = mRi[2][0] * P3Dw.x + mRi[2][1] * P3Dw.y +
                   mRi[2][2] * P3Dw.z + mti[2];

        cv::Point3f P3Dc(xc, yc, zc);
        cv::Point2f uv = p_camera->project(P3Dc);

        float distX = P2D.x - uv.x;
        float distY = P2D.y - uv.y;

        float error2 = distX * distX + distY * distY;

        if (error2 < maxError[i])
        {
            inlierFlags[i] = true;
            inlierCount++;
        }
        else
        {
            inlierFlags[i] = false;
        }
    }
}

} // namespace core
} // namespace vs_graphs
