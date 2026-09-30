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
 * @file            withImage.cc
 *
 * @brief           Implements ORBextractor::operator()(), declared in
 *                  ORBextractor.h.
 */

/*!
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ORBextractor.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

#include "../../private_functions.h"

namespace vs_graphs
{
namespace core
{

int ORBextractor::operator()(cv::InputArray                  image_in,
                             [[maybe_unused]] cv::InputArray mask_in,
                             std::vector<cv::KeyPoint>      &keypoints_inout,
                             cv::OutputArray                 descriptors_in,
                             std::vector<int>               &lappingArea_in)
{
    // cout << "[ORBextractor]: Max Features: " << featureCount << endl;
    if (image_in.empty())
        return -1;

    cv::Mat image = image_in.getMat();
    assert(image.type() == CV_8UC1);

    // Pre-compute the scale pyramid
    if (computePyramid(image) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computePyramid returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }

    std::vector<std::vector<cv::KeyPoint>> allKeypoints;
    if (computeKeyPointsOctTree(allKeypoints) !=
        ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: computeKeyPointsOctTree returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    // computeKeyPointsOld(allKeypoints);

    cv::Mat descriptors;

    int nkeypoints = 0;
    for (int level = 0; level < levelCount; ++level)
        nkeypoints += static_cast<int>(allKeypoints[level].size());
    if (nkeypoints == 0)
        descriptors_in.release();
    else
    {
        descriptors_in.create(nkeypoints, 32, CV_8U);
        descriptors = descriptors_in.getMat();
    }

    // keypoints_out.clear();
    // keypoints_out.reserve(nkeypoints);
    keypoints_inout = std::vector<cv::KeyPoint>(nkeypoints);

    int offset = 0;
    // Modified for speeding up stereo fisheye matching
    int monoIndex = 0, stereoIndex = nkeypoints - 1;
    for (int level = 0; level < levelCount; ++level)
    {
        std::vector<cv::KeyPoint> &keypoints = allKeypoints[level];
        int nkeypointsLevel = static_cast<int>(keypoints.size());

        if (nkeypointsLevel == 0)
            continue;

        // preprocess the resized image
        cv::Mat workingMatrix = imagePyramid[level].clone();
        cv::GaussianBlur(workingMatrix,
                         workingMatrix,
                         cv::Size(7, 7),
                         2,
                         2,
                         cv::BORDER_REFLECT_101);

        // Compute the descriptors
        // Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
        cv::Mat descriptor = cv::Mat(nkeypointsLevel, 32, CV_8U);
        if (computeDescriptors(workingMatrix,
                               keypoints,
                               descriptor,
                               briefPattern) !=
            ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeDescriptors returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }

        offset += nkeypointsLevel;

        float scale =
            scaleFactors[level]; // getScale(level, firstLevel, scaleFactor);
        int i = 0;
        for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                                                 keypointEnd = keypoints.end();
             keypoint != keypointEnd;
             ++keypoint)
        {

            // Scale keypoint coordinates
            if (level != 0)
            {
                keypoint->pt *= scale;
            }

            if (keypoint->pt.x >= lappingArea_in[0] &&
                keypoint->pt.x <= lappingArea_in[1])
            {
                keypoints_inout.at(stereoIndex) = (*keypoint);
                descriptor.row(i).copyTo(descriptors.row(stereoIndex));
                stereoIndex--;
            }
            else
            {
                keypoints_inout.at(monoIndex) = (*keypoint);
                descriptor.row(i).copyTo(descriptors.row(monoIndex));
                monoIndex++;
            }
            i++;
        }
    }
    // cout << "[ORBextractor]: extracted " << keypoints_out.size() << "
    // KeyPoints" << endl;
    return monoIndex;
}

} // namespace core
} // namespace vs_graphs
