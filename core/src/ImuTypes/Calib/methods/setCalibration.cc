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
 * @file            setCalibration.cc
 *
 * @brief           Implements Calib::setCalibration(), declared in ImuTypes.h.
 */

#include "ImuTypes.h"

namespace vs_graphs
{
namespace core
{
namespace IMU
{

CalibStatus Calib::setCalibration(
    const Sophus::SE3<float> &extrinsicPose_cameraToBody_in,
    const float              &ng_in,
    const float              &na_in,
    const float              &ngw_in,
    const float              &naw_in)
{
    isCalibrationSet = true;
    const float ng2  = ng_in * ng_in;
    const float na2  = na_in * na_in;
    const float ngw2 = ngw_in * ngw_in;
    const float naw2 = naw_in * naw_in;

    // Sophus/Eigen
    mTbc = extrinsicPose_cameraToBody_in;
    mTcb = mTbc.inverse();
    Cov.diagonal() << ng2, ng2, ng2, na2, na2, na2;
    CovWalk.diagonal() << ngw2, ngw2, ngw2, naw2, naw2, naw2;

    return CalibStatus::CALIB_STATUS_SUCCESS;
}

} // namespace IMU
} // namespace core
} // namespace vs_graphs
