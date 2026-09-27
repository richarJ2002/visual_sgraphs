/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*!
 * @file            precomputeRectificationMaps.cc
 *
 * @brief           Implements Settings::precomputeRectificationMaps(),
 *                  declared in Utils/Settings/objects/Settings.h.
 */

#include "Utils/Settings/objects/Settings.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>

#include "System.h"

#include "CameraModels/Pinhole/objects/Pinhole.h"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace settings
{

void Settings::precomputeRectificationMaps()
{
    // Precompute rectification maps, new calibrations, ...
    cv::Mat K1 =
        static_cast<camera_models::pinhole::Pinhole *>(calibration1)->toK();
    K1.convertTo(K1, CV_64F);
    cv::Mat K2 =
        static_cast<camera_models::pinhole::Pinhole *>(calibration2)->toK();
    K2.convertTo(K2, CV_64F);

    cv::Mat cvTlr;
    cv::eigen2cv(stereoTransform.inverse().matrix3x4(), cvTlr);
    cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
    R12.convertTo(R12, CV_64F);
    cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
    t12.convertTo(t12, CV_64F);

    cv::Mat R_r1_u1, R_r2_u2;
    cv::Mat P1, P2, Q;

    cv::stereoRectify(K1,
                      camera1DistortionCoef(),
                      K2,
                      camera2DistortionCoef(),
                      newImageSize,
                      R12,
                      t12,
                      R_r1_u1,
                      R_r2_u2,
                      P1,
                      P2,
                      Q,
                      cv::CALIB_ZERO_DISPARITY,
                      -1,
                      newImageSize);
    cv::initUndistortRectifyMap(K1,
                                camera1DistortionCoef(),
                                R_r1_u1,
                                P1.rowRange(0, 3).colRange(0, 3),
                                newImageSize,
                                CV_32F,
                                rectifyMap1Left,
                                rectifyMap2Left);
    cv::initUndistortRectifyMap(K2,
                                camera2DistortionCoef(),
                                R_r2_u2,
                                P2.rowRange(0, 3).colRange(0, 3),
                                newImageSize,
                                CV_32F,
                                rectifyMap1Right,
                                rectifyMap2Right);

    // Update calibration
    calibration1->setParameter(P1.at<double>(0, 0), 0);
    calibration1->setParameter(P1.at<double>(1, 1), 1);
    calibration1->setParameter(P1.at<double>(0, 2), 2);
    calibration1->setParameter(P1.at<double>(1, 2), 3);

    calibration2->setParameter(P2.at<double>(0, 0), 0);
    calibration2->setParameter(P2.at<double>(1, 1), 1);
    calibration2->setParameter(P2.at<double>(0, 2), 2);
    calibration2->setParameter(P2.at<double>(1, 2), 3);

    // Update bf
    baselineFocal = stereoBaseline * P1.at<double>(0, 0);

    // Update relative pose between camera 1 and IMU if necessary
    if (sensor == System::IMU_STEREO)
    {
        Eigen::Matrix3f eigenR_r1_u1;
        cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
        Sophus::SE3f T_r1_u1(eigenR_r1_u1, Eigen::Vector3f::Zero());
        bodyToCamera = bodyToCamera * T_r1_u1.inverse();
    }
}

} // namespace settings
} // namespace utils
} // namespace core
} // namespace vs_graphs
