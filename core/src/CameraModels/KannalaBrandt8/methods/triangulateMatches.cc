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
 *                                   declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <rclcpp/logging.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
KannalaBrandt8Status KannalaBrandt8::triangulateMatches(
    geometriccamera::GeometricCamera *p_otherCamera_inout,
    const cv::KeyPoint               &keypoint1_in,
    const cv::KeyPoint               &keypoint2_in,
    const Eigen::Matrix3f            &rotation12_in,
    const Eigen::Vector3f            &translation12_in,
    const float                       sigmaLevel_in,
    const float                       uncertainty_in,
    Eigen::Vector3f                  &point3d_out,
    float                            &parallax_out)
{

    Eigen::Vector3f cameraRay1 = this->unprojectEig(keypoint1_in.pt);
    Eigen::Vector3f cameraRay2 =
        p_otherCamera_inout->unprojectEig(keypoint2_in.pt);

    // Check parallax
    Eigen::Vector3f ray2InFrame1 = rotation12_in * cameraRay2;

    const float parallaxCosine = cameraRay1.dot(ray2InFrame1) /
                                 (cameraRay1.norm() * ray2InFrame1.norm());

    if (parallaxCosine > 0.9998)
    {
        parallax_out = -1;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    // Parallax is good, so we try to triangulate
    cv::Point2f imagePoint1, imagePoint2;

    imagePoint1.x = cameraRay1[0];
    imagePoint1.y = cameraRay1[1];

    imagePoint2.x = cameraRay2[0];
    imagePoint2.y = cameraRay2[1];

    Eigen::Vector3f            triangulatedPoint3D;
    Eigen::Matrix<float, 3, 4> projectionMatrix1;
    projectionMatrix1 << Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero();

    Eigen::Matrix<float, 3, 4> projectionMatrix2;

    Eigen::Matrix3f R21 = rotation12_in.transpose();
    projectionMatrix2 << R21, -R21 * translation12_in;

    if (triangulate(imagePoint1,
                    imagePoint2,
                    projectionMatrix1,
                    projectionMatrix2,
                    triangulatedPoint3D) !=
        KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: triangulate returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    // cv::Mat x3Dt = x3D.t();

    float cameraDepth1 = triangulatedPoint3D(2);
    if (cameraDepth1 <= 0)
    {
        parallax_out = -2;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    float cameraDepth2 =
        R21.row(2).dot(triangulatedPoint3D) + projectionMatrix2(2, 3);
    if (cameraDepth2 <= 0)
    {
        parallax_out = -3;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    // Check reprojection error
    Eigen::Vector2f projectedPoint1 = this->project(triangulatedPoint3D);

    float reprojectionErrorX1 = projectedPoint1(0) - keypoint1_in.pt.x;
    float reprojectionErrorY1 = projectedPoint1(1) - keypoint1_in.pt.y;

    if ((reprojectionErrorX1 * reprojectionErrorX1 +
         reprojectionErrorY1 * reprojectionErrorY1) > 5.991 * sigmaLevel_in)
    { // Reprojection error is high
        parallax_out = -4;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    Eigen::Vector3f pointInCamera2 =
        R21 * triangulatedPoint3D + projectionMatrix2.col(3);
    Eigen::Vector2f projectedPoint2 =
        p_otherCamera_inout->project(pointInCamera2);

    float reprojectionErrorX2 = projectedPoint2(0) - keypoint2_in.pt.x;
    float reprojectionErrorY2 = projectedPoint2(1) - keypoint2_in.pt.y;

    if ((reprojectionErrorX2 * reprojectionErrorX2 +
         reprojectionErrorY2 * reprojectionErrorY2) > 5.991 * uncertainty_in)
    { // Reprojection error is high
        parallax_out = -5;
        return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
    }

    point3d_out = triangulatedPoint3D;

    parallax_out = cameraDepth1;
    return KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
