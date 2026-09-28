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
    Sophus::SE3f                    &pose21_inout,
    std::vector<cv::Point3f>        &points3d_inout,
    std::vector<bool>               &triangulated_inout)
{
    /*!
     * If address for two view reconstruction does not exist then init. This is
     * only done on the first reconstruction.
     */
    if (!p_twoViewReconstruction)
    {
        /* Extract calibration matrix */
        Eigen::Matrix3f calibrationMatrix = this->toK_();

        /* Init two view reconstruction */
        p_twoViewReconstruction = new TwoViewReconstruction(calibrationMatrix);
    }

    /* Correct FishEye distortion */
    std::vector<cv::KeyPoint> undistortedKeys1 = keys1_in,
                              undistortedKeys2 = keys2_in;
    std::vector<cv::Point2f> undistortedPoints1(keys1_in.size());
    std::vector<cv::Point2f> undistortedPoints2(keys2_in.size());

    /* Extract points from key 1 */
    for (size_t featureIndex = 0; featureIndex < keys1_in.size();
         featureIndex++)
    {
        undistortedPoints1[featureIndex] = keys1_in[featureIndex].pt;
    }

    /* Extract points from key 2 */
    for (size_t featureIndex = 0; featureIndex < keys2_in.size();
         featureIndex++)
    {
        undistortedPoints2[featureIndex] = keys2_in[featureIndex].pt;
    }

    cv::Mat distortionCoefficients = (cv::Mat_<float>(4, 1) << parameters[4],
                                      parameters[5],
                                      parameters[6],
                                      parameters[7]);

    cv::Mat rectificationMatrix = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat calibrationMatrix   = this->toK();
    cv::fisheye::undistortPoints(undistortedPoints1,
                                 undistortedPoints1,
                                 calibrationMatrix,
                                 distortionCoefficients,
                                 rectificationMatrix,
                                 calibrationMatrix);
    cv::fisheye::undistortPoints(undistortedPoints2,
                                 undistortedPoints2,
                                 calibrationMatrix,
                                 distortionCoefficients,
                                 rectificationMatrix,
                                 calibrationMatrix);

    for (size_t featureIndex = 0; featureIndex < keys1_in.size();
         featureIndex++)
    {
        undistortedKeys1[featureIndex].pt = undistortedPoints1[featureIndex];
    }

    for (size_t featureIndex = 0; featureIndex < keys2_in.size();
         featureIndex++)
    {
        undistortedKeys2[featureIndex].pt = undistortedPoints2[featureIndex];
    }

    return p_twoViewReconstruction->reconstruct(undistortedKeys1,
                                                undistortedKeys2,
                                                matches12_in,
                                                pose21_inout,
                                                points3d_inout,
                                                triangulated_inout);
}
} // namespace vs_graphs::core::camera_models::kannalabrandt8
