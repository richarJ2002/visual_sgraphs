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
 * @file            matchAndTriangulate.cc
 *
 * @brief           Implements KannalaBrandt8::matchAndTriangulate(),
 *                                   declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <rclcpp/logging.hpp>
#include <sophus/se3.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
bool KannalaBrandt8::matchAndTriangulate(
    const cv::KeyPoint               &keypoint1_in,
    const cv::KeyPoint               &keypoint2_in,
    geometriccamera::GeometricCamera *p_otherCamera_inout,
    Sophus::SE3f                     &pose1_in,
    Sophus::SE3f                     &pose2_in,
    const float                       sigmaLevel1_in,
    const float                       sigmaLevel2_in,
    Eigen::Vector3f                  &point3d_inout)
{
    /* Declare and init local variables */
    Eigen::Matrix<float, 3, 4> cameraPose_worldToCamera1 = pose1_in.matrix3x4();
    Eigen::Matrix3f            cameraRotation_worldToCamera1 =
        cameraPose_worldToCamera1.block<3, 3>(0, 0);
    Eigen::Matrix3f cameraRotation_camera1ToWorld =
        cameraRotation_worldToCamera1.transpose();
    Eigen::Matrix<float, 3, 4> cameraPose_worldToCamera2 = pose2_in.matrix3x4();
    Eigen::Matrix3f            cameraRotation_worldToCamera2 =
        cameraPose_worldToCamera2.block<3, 3>(0, 0);
    Eigen::Matrix3f cameraRotation_camera2ToWorld =
        cameraRotation_worldToCamera2.transpose();

    cv::Point3f unprojectedPoint1 = this->unproject(keypoint1_in.pt);
    cv::Point3f unprojectedPoint2 =
        p_otherCamera_inout->unproject(keypoint2_in.pt);

    Eigen::Vector3f cameraRay1(unprojectedPoint1.x,
                               unprojectedPoint1.y,
                               unprojectedPoint1.z);
    Eigen::Vector3f cameraRay2(unprojectedPoint2.x,
                               unprojectedPoint2.y,
                               unprojectedPoint2.z);

    /* Check parallax between rays */
    Eigen::Vector3f worldRay1 = cameraRotation_camera1ToWorld * cameraRay1;
    Eigen::Vector3f worldRay2 = cameraRotation_camera2ToWorld * cameraRay2;

    const float parallaxCosine =
        worldRay1.dot(worldRay2) / (worldRay1.norm() * worldRay2.norm());

    /* If parallax is lower than 0.9998, reject this match */
    if (parallaxCosine > 0.9998)
    {
        return false;
    }

    /* Parallax is good, so we try to triangulate */
    cv::Point2f imagePoint1, imagePoint2;

    imagePoint1.x = unprojectedPoint1.x;
    imagePoint1.y = unprojectedPoint1.y;

    imagePoint2.x = unprojectedPoint2.x;
    imagePoint2.y = unprojectedPoint2.y;

    Eigen::Vector3f triangulatedPoint3D;

    if (triangulate(imagePoint1,
                    imagePoint2,
                    cameraPose_worldToCamera1,
                    cameraPose_worldToCamera2,
                    triangulatedPoint3D) !=
        KannalaBrandt8Status::KANNALA_BRANDT8_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: triangulate returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    /* Check triangulation in front of cameras */
    float cameraDepth1 =
        cameraRotation_worldToCamera1.row(2).dot(triangulatedPoint3D) +
        pose1_in.translation()(2);
    if (cameraDepth1 <= 0)
    {
        /* Point is not in front of the first camera */
        return false;
    }

    float cameraDepth2 =
        cameraRotation_worldToCamera2.row(2).dot(triangulatedPoint3D) +
        pose2_in.translation()(2);
    if (cameraDepth2 <= 0)
    {
        /* Point is not in front of the first camera */
        return false;
    }

    /*!
     * Check reprojection error in first keyframe: Transform point into camera
     * reference system.
     */
    Eigen::Vector3f pointInCamera1 =
        cameraRotation_worldToCamera1 * triangulatedPoint3D +
        pose1_in.translation();
    Eigen::Vector2f projectedPoint1 = this->project(pointInCamera1);

    float reprojectionErrorX1 = projectedPoint1(0) - keypoint1_in.pt.x;
    float reprojectionErrorY1 = projectedPoint1(1) - keypoint1_in.pt.y;

    if ((reprojectionErrorX1 * reprojectionErrorX1 +
         reprojectionErrorY1 * reprojectionErrorY1) > 5.991 * sigmaLevel1_in)
    {
        /* Reprojection error is high, hence reject */
        return false;
    }

    /*!
     * Check reprojection error in second keyframe: Transform point into camera
     * reference system.
     */
    Eigen::Vector3f pointInCamera2 =
        cameraRotation_worldToCamera2 * triangulatedPoint3D +
        pose2_in.translation();
    Eigen::Vector2f projectedPoint2 =
        p_otherCamera_inout->project(pointInCamera2);

    float reprojectionErrorX2 = projectedPoint2(0) - keypoint2_in.pt.x;
    float reprojectionErrorY2 = projectedPoint2(1) - keypoint2_in.pt.y;

    if ((reprojectionErrorX2 * reprojectionErrorX2 +
         reprojectionErrorY2 * reprojectionErrorY2) > 5.991 * sigmaLevel2_in)
    {
        /* Reprojection error is high */
        return false;
    }

    /*!
     * Since parallax is big enough and reprojection errors are low, this pair
     * of points can be considered as a match.
     */
    point3d_inout = triangulatedPoint3D;

    return true;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
