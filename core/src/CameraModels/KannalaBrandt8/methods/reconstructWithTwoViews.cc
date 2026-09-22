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
 * @brief           Implements KannalaBrandt8::reconstructWithTwoViews(),
 *                  declared in
 * CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h.
 */

#include "CameraModels/KannalaBrandt8/objects/KannalaBrandt8.h"

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <sophus/se3.hpp>

#include "TwoViewReconstruction.h"

namespace vs_graphs::core::camera_models::kannalabrandt8
{
bool KannalaBrandt8::reconstructWithTwoViews(
    const std::vector<cv::KeyPoint> &keys1_in,
    const std::vector<cv::KeyPoint> &keys2_in,
    const std::vector<int>          &matches12_in,
    Sophus::SE3f                    &pose21_out,
    std::vector<cv::Point3f>        &points3D_out,
    std::vector<bool>               &triangulated_out)
{
    /*!
     * If address for two view reconstruction does not exist then init. This is
     * only done on the first reconstruction.
     */
    if (!p_twoViewReconstruction)
    {
        /* Extract calibration matrix */
        Eigen::Matrix3f K = this->toK_();

        /* Init two view reconstruction */
        p_twoViewReconstruction = new TwoViewReconstruction(K);
    }

    /* Correct FishEye distortion */
    std::vector<cv::KeyPoint> vKeysUn1 = keys1_in, vKeysUn2 = keys2_in;
    std::vector<cv::Point2f>  vPts1(keys1_in.size());
    std::vector<cv::Point2f>  vPts2(keys2_in.size());

    /* Extract points from key 1 */
    for (size_t i = 0; i < keys1_in.size(); i++)
    {
        vPts1[i] = keys1_in[i].pt;
    }

    /* Extract points from key 2 */
    for (size_t i = 0; i < keys2_in.size(); i++)
    {
        vPts2[i] = keys2_in[i].pt;
    }

    cv::Mat D = (cv::Mat_<float>(4, 1) << parameters[4],
                 parameters[5],
                 parameters[6],
                 parameters[7]);

    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat K = this->toK();
    cv::fisheye::undistortPoints(vPts1, vPts1, K, D, R, K);
    cv::fisheye::undistortPoints(vPts2, vPts2, K, D, R, K);

    for (size_t i = 0; i < keys1_in.size(); i++)
    {
        vKeysUn1[i].pt = vPts1[i];
    }

    for (size_t i = 0; i < keys2_in.size(); i++)
    {
        vKeysUn2[i].pt = vPts2[i];
    }

    return p_twoViewReconstruction->Reconstruct(vKeysUn1,
                                                vKeysUn2,
                                                matches12_in,
                                                pose21_out,
                                                points3D_out,
                                                triangulated_out);
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
