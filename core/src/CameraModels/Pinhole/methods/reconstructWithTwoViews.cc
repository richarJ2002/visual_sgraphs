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
 * @file            reconstructWithTwoViews.cc
 *
 * @brief           Implements Pinhole::reconstructWithTwoViews(),
 *                  declared in CameraModels/Pinhole/objects/Pinhole.h.
 */

#include "CameraModels/Pinhole/objects/Pinhole.h"

#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <sophus/se3.hpp>

#include "TwoViewReconstruction.h"

namespace vs_graphs
{
namespace core
{
namespace camera_models
{
namespace pinhole
{
bool Pinhole::reconstructWithTwoViews(const std::vector<cv::KeyPoint> &keys1_in,
                                      const std::vector<cv::KeyPoint> &keys2_in,
                                      const std::vector<int>   &matches12_in,
                                      Sophus::SE3f             &pose21_inout,
                                      std::vector<cv::Point3f> &points3d_inout,
                                      std::vector<bool> &triangulated_inout)
{
    if (!p_twoViewReconstruction)
    {
        Eigen::Matrix3f cameraMatrix = this->toK_();
        p_twoViewReconstruction      = new TwoViewReconstruction(cameraMatrix);
    }

    return p_twoViewReconstruction->reconstruct(keys1_in,
                                                keys2_in,
                                                matches12_in,
                                                pose21_inout,
                                                points3d_inout,
                                                triangulated_inout);
}
} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
