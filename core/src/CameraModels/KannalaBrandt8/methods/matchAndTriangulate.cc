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
 *                  declared in
 * CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <sophus/se3.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
bool KannalaBrandt8::matchAndTriangulate(
    const cv::KeyPoint               &keypoint1_in,
    const cv::KeyPoint               &keypoint2_in,
    geometriccamera::GeometricCamera *p_otherCamera_in,
    Sophus::SE3f                     &pose1_in,
    Sophus::SE3f                     &pose2_in,
    const float                       sigmaLevel1_in,
    const float                       sigmaLevel2_in,
    Eigen::Vector3f                  &point3D_out)
{
    /* Declare and init local variables */
    Eigen::Matrix<float, 3, 4> eigTcw1 = pose1_in.matrix3x4();
    Eigen::Matrix3f            Rcw1    = eigTcw1.block<3, 3>(0, 0);
    Eigen::Matrix3f            Rwc1    = Rcw1.transpose();
    Eigen::Matrix<float, 3, 4> eigTcw2 = pose2_in.matrix3x4();
    Eigen::Matrix3f            Rcw2    = eigTcw2.block<3, 3>(0, 0);
    Eigen::Matrix3f            Rwc2    = Rcw2.transpose();

    cv::Point3f ray1c = this->unproject(keypoint1_in.pt);
    cv::Point3f ray2c = p_otherCamera_in->unproject(keypoint2_in.pt);

    Eigen::Vector3f r1(ray1c.x, ray1c.y, ray1c.z);
    Eigen::Vector3f r2(ray2c.x, ray2c.y, ray2c.z);

    /* Check parallax between rays */
    Eigen::Vector3f ray1 = Rwc1 * r1;
    Eigen::Vector3f ray2 = Rwc2 * r2;

    const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

    /* If parallax is lower than 0.9998, reject this match */
    if (cosParallaxRays > 0.9998)
    {
        return false;
    }

    /* Parallax is good, so we try to triangulate */
    cv::Point2f p11, p22;

    p11.x = ray1c.x;
    p11.y = ray1c.y;

    p22.x = ray2c.x;
    p22.y = ray2c.y;

    Eigen::Vector3f x3D;

    triangulate(p11, p22, eigTcw1, eigTcw2, x3D);

    /* Check triangulation in front of cameras */
    float z1 = Rcw1.row(2).dot(x3D) + pose1_in.translation()(2);
    if (z1 <= 0)
    {
        /* Point is not in front of the first camera */
        return false;
    }

    float z2 = Rcw2.row(2).dot(x3D) + pose2_in.translation()(2);
    if (z2 <= 0)
    {
        /* Point is not in front of the first camera */
        return false;
    }

    /*!
     * Check reprojection error in first keyframe: Transform point into camera
     * reference system.
     */
    Eigen::Vector3f x3D1 = Rcw1 * x3D + pose1_in.translation();
    Eigen::Vector2f uv1  = this->project(x3D1);

    float errX1 = uv1(0) - keypoint1_in.pt.x;
    float errY1 = uv1(1) - keypoint1_in.pt.y;

    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaLevel1_in)
    {
        /* Reprojection error is high, hence reject */
        return false;
    }

    /*!
     * Check reprojection error in second keyframe: Transform point into camera
     * reference system.
     */
    Eigen::Vector3f x3D2 = Rcw2 * x3D + pose2_in.translation();
    Eigen::Vector2f uv2  = p_otherCamera_in->project(x3D2);

    float errX2 = uv2(0) - keypoint2_in.pt.x;
    float errY2 = uv2(1) - keypoint2_in.pt.y;

    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaLevel2_in)
    {
        /* Reprojection error is high */
        return false;
    }

    /*!
     * Since parallax is big enough and reprojection errors are low, this pair
     * of points can be considered as a match.
     */
    point3D_out = x3D;

    return true;
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
