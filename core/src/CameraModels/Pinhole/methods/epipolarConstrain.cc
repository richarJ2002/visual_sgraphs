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
 * @file            epipolarConstrain.cc
 *
 * @brief           Implements Pinhole::epipolarConstrain(), declared in
 *                  CameraModels/Pinhole/objects/Pinhole.h.
 */

#include "CameraModels/Pinhole/objects/Pinhole.h"

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <sophus/so3.hpp>

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{
bool Pinhole::epipolarConstrain(
    geometriccamera::GeometricCamera *p_otherCamera_inout,
    const cv::KeyPoint               &keypoint1_in,
    const cv::KeyPoint               &keypoint2_in,
    const Eigen::Matrix3f            &rotation12_in,
    const Eigen::Vector3f            &translation12_in,
    [[maybe_unused]] const float      sigmaLevel_in,
    const float                       uncertainty_in)
{
    // Compute Fundamental Matrix
    Eigen::Matrix3f t12x          = Sophus::SO3f::hat(translation12_in);
    Eigen::Matrix3f cameraMatrix1 = this->toK_();
    Eigen::Matrix3f cameraMatrix2 = p_otherCamera_inout->toK_();
    Eigen::Matrix3f F12           = cameraMatrix1.transpose().inverse() * t12x *
                          rotation12_in * cameraMatrix2.inverse();

    // Epipolar line in second image l = x1'F12 = [a b c]
    const float epipolarLineA = keypoint1_in.pt.x * F12(0, 0) +
                                keypoint1_in.pt.y * F12(1, 0) + F12(2, 0);
    const float epipolarLineB = keypoint1_in.pt.x * F12(0, 1) +
                                keypoint1_in.pt.y * F12(1, 1) + F12(2, 1);
    const float epipolarLineC = keypoint1_in.pt.x * F12(0, 2) +
                                keypoint1_in.pt.y * F12(1, 2) + F12(2, 2);

    const float numerator = epipolarLineA * keypoint2_in.pt.x +
                            epipolarLineB * keypoint2_in.pt.y + epipolarLineC;

    const float denominator =
        epipolarLineA * epipolarLineA + epipolarLineB * epipolarLineB;

    if (denominator == 0)
        return false;

    const float squaredDistance = numerator * numerator / denominator;

    return squaredDistance < 3.84 * uncertainty_in;
}
} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
