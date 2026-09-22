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

void Calib::setCalibration(const Sophus::SE3<float> &sophTbc,
                           const float              &ng,
                           const float              &na,
                           const float              &ngw,
                           const float              &naw)
{
    mbIsSet          = true;
    const float ng2  = ng * ng;
    const float na2  = na * na;
    const float ngw2 = ngw * ngw;
    const float naw2 = naw * naw;

    // Sophus/Eigen
    mTbc = sophTbc;
    mTcb = mTbc.inverse();
    Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
    CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
