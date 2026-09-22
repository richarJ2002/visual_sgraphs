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
 * @file            triangulateMatches.cc
 *
 * @brief           Implements KannalaBrandt8::triangulateMatches(),
 *                  declared in
 * CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
float KannalaBrandt8::triangulateMatches(
    geometriccamera::GeometricCamera *p_otherCamera_in,
    const cv::KeyPoint               &keypoint1_in,
    const cv::KeyPoint               &keypoint2_in,
    const Eigen::Matrix3f            &rotation12_in,
    const Eigen::Vector3f            &translation12_in,
    const float                       sigmaLevel_in,
    const float                       uncertainty_in,
    Eigen::Vector3f                  &point3D_out)
{

    Eigen::Vector3f r1 = this->unprojectEig(keypoint1_in.pt);
    Eigen::Vector3f r2 = p_otherCamera_in->unprojectEig(keypoint2_in.pt);

    // Check parallax
    Eigen::Vector3f r21 = rotation12_in * r2;

    const float cosParallaxRays = r1.dot(r21) / (r1.norm() * r21.norm());

    if (cosParallaxRays > 0.9998)
    {
        return -1;
    }

    // Parallax is good, so we try to triangulate
    cv::Point2f p11, p22;

    p11.x = r1[0];
    p11.y = r1[1];

    p22.x = r2[0];
    p22.y = r2[1];

    Eigen::Vector3f            x3D;
    Eigen::Matrix<float, 3, 4> pose1_in;
    pose1_in << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

    Eigen::Matrix<float, 3, 4> pose2_in;

    Eigen::Matrix3f R21 = rotation12_in.transpose();
    pose2_in << R21, -R21 * translation12_in;

    triangulate(p11, p22, pose1_in, pose2_in, x3D);
    // cv::Mat x3Dt = x3D.t();

    float z1 = x3D(2);
    if (z1 <= 0)
    {
        return -2;
    }

    float z2 = R21.row(2).dot(x3D) + pose2_in(2, 3);
    if (z2 <= 0)
    {
        return -3;
    }

    // Check reprojection error
    Eigen::Vector2f uv1 = this->project(x3D);

    float errX1 = uv1(0) - keypoint1_in.pt.x;
    float errY1 = uv1(1) - keypoint1_in.pt.y;

    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel_in)
    { // Reprojection error is high
        return -4;
    }

    Eigen::Vector3f x3D2 = R21 * x3D + pose2_in.col(3);
    Eigen::Vector2f uv2  = p_otherCamera_in->project(x3D2);

    float errX2 = uv2(0) - keypoint2_in.pt.x;
    float errY2 = uv2(1) - keypoint2_in.pt.y;

    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * uncertainty_in)
    { // Reprojection error is high
        return -5;
    }

    point3D_out = x3D;

    return z1;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
