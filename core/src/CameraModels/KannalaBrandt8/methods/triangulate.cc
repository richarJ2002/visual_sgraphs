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
 * @file            triangulate.cc
 *
 * @brief           Implements the private KannalaBrandt8::triangulate()
 *                  helper, declared in
 *                  CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

namespace vs_graphs::core::camera_models::kannalabrandt8
{
void KannalaBrandt8::triangulate(const cv::Point2f                &point1_in,
                                 const cv::Point2f                &point2_in,
                                 const Eigen::Matrix<float, 3, 4> &pose1_in,
                                 const Eigen::Matrix<float, 3, 4> &pose2_in,
                                 Eigen::Vector3f                  &point3d_out)
{
    Eigen::Matrix<float, 4, 4> designMatrix;
    designMatrix.row(0) = point1_in.x * pose1_in.row(2) - pose1_in.row(0);
    designMatrix.row(1) = point1_in.y * pose1_in.row(2) - pose1_in.row(1);
    designMatrix.row(2) = point2_in.x * pose2_in.row(2) - pose2_in.row(0);
    designMatrix.row(3) = point2_in.y * pose2_in.row(2) - pose2_in.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4f> svd(designMatrix, Eigen::ComputeFullV);
    Eigen::Vector4f                   homogeneousPoint3D = svd.matrixV().col(3);
    point3d_out = homogeneousPoint3D.head(3) / homogeneousPoint3D(3);
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
