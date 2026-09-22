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
                                      Sophus::SE3f             &pose21_out,
                                      std::vector<cv::Point3f> &points3D_out,
                                      std::vector<bool> &triangulated_out)
{
    if (!p_twoViewReconstruction)
    {
        Eigen::Matrix3f K       = this->toK_();
        p_twoViewReconstruction = new TwoViewReconstruction(K);
    }

    return p_twoViewReconstruction->Reconstruct(keys1_in,
                                                keys2_in,
                                                matches12_in,
                                                pose21_out,
                                                points3D_out,
                                                triangulated_out);
}
} // namespace pinhole
} // namespace camera_models
} // namespace core
} // namespace vs_graphs
