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
 * @file            computeOrbDescriptor.cc
 *
 * @brief           Implements computeOrbDescriptor(), declared in
 *                  ORBextractor/private_functions.h.
 */

/*
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
#include <vector>

/*!
 * @brief           Conversion factor from degrees to radians (pi / 180).
 */
const float factorPI = static_cast<float>(CV_PI / 180.f);

namespace vs_graphs
{
namespace core
{

ORBextractorStatus computeOrbDescriptor(const cv::KeyPoint &kpt_in,
                                        const cv::Mat      &image_in,
                                        const cv::Point    *p_briefPattern_in,
                                        unsigned char      *p_descriptor_inout)
{
    float angle = kpt_in.angle * factorPI;
    float a = std::cos(angle), b = std::sin(angle);

    const uchar *p_center =
        &image_in.at<uchar>(cvRound(kpt_in.pt.y), cvRound(kpt_in.pt.x));
    const int step = static_cast<int>(image_in.step);

    /*
     * Brightness at the patternIndex_in-th point of the current 16-point
     * slice of the sampling pattern, after rotating that point by the
     * keypoint orientation (a = cos, b = sin of the angle).
     */
    const auto readRotatedPatternValue = [&](int patternIndex_in) -> int
    {
        return p_center[cvRound(p_briefPattern_in[patternIndex_in].x * b +
                                p_briefPattern_in[patternIndex_in].y * a) *
                            step +
                        cvRound(p_briefPattern_in[patternIndex_in].x * a -
                                p_briefPattern_in[patternIndex_in].y * b)];
    };

    for (int descriptorByteIndex = 0; descriptorByteIndex < 32;
         ++descriptorByteIndex, p_briefPattern_in += 16)
    {
        int t0, t1, value;
        t0    = readRotatedPatternValue(0);
        t1    = readRotatedPatternValue(1);
        value = t0 < t1;
        t0    = readRotatedPatternValue(2);
        t1    = readRotatedPatternValue(3);
        value |= (t0 < t1) << 1;
        t0 = readRotatedPatternValue(4);
        t1 = readRotatedPatternValue(5);
        value |= (t0 < t1) << 2;
        t0 = readRotatedPatternValue(6);
        t1 = readRotatedPatternValue(7);
        value |= (t0 < t1) << 3;
        t0 = readRotatedPatternValue(8);
        t1 = readRotatedPatternValue(9);
        value |= (t0 < t1) << 4;
        t0 = readRotatedPatternValue(10);
        t1 = readRotatedPatternValue(11);
        value |= (t0 < t1) << 5;
        t0 = readRotatedPatternValue(12);
        t1 = readRotatedPatternValue(13);
        value |= (t0 < t1) << 6;
        t0 = readRotatedPatternValue(14);
        t1 = readRotatedPatternValue(15);
        value |= (t0 < t1) << 7;

        p_descriptor_inout[descriptorByteIndex] = static_cast<uchar>(value);
    }

    return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
