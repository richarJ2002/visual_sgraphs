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

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{

ORBextractorStatus ORBextractor::computePyramid(cv::Mat image_in)
{
    for (int level = 0; level < levelCount; ++level)
    {
        float    scale = inverseScaleFactors[level];
        cv::Size size(cvRound(static_cast<float>(image_in.cols) * scale),
                      cvRound(static_cast<float>(image_in.rows) * scale));
        cv::Size wholeSize(size.width + EDGE_THRESHOLD * 2,
                           size.height + EDGE_THRESHOLD * 2);
        cv::Mat  temp(wholeSize, image_in.type()), masktemp;
        imagePyramid[level] = temp(
            cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, size.width, size.height));

        // Compute the resized image_in
        if (level != 0)
        {
            cv::resize(imagePyramid[level - 1],
                       imagePyramid[level],
                       size,
                       0,
                       0,
                       cv::INTER_LINEAR);

            cv::copyMakeBorder(imagePyramid[level],
                               temp,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);
        }
        else
        {
            cv::copyMakeBorder(image_in,
                               temp,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               EDGE_THRESHOLD,
                               cv::BORDER_REFLECT_101);
        }
    }

    return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
