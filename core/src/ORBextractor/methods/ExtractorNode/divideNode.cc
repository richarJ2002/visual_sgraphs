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

using namespace cv;
using namespace std;

namespace vs_graphs
{
namespace core
{

void ExtractorNode::divideNode(ExtractorNode &node1_out,
                               ExtractorNode &node2_out,
                               ExtractorNode &node3_out,
                               ExtractorNode &node4_out)
{
    const int halfX = ceil(static_cast<float>(topRight.x - topLeft.x) / 2);
    const int halfY = ceil(static_cast<float>(bottomRight.y - topLeft.y) / 2);

    // Define boundaries of childs
    node1_out.topLeft     = topLeft;
    node1_out.topRight    = cv::Point2i(topLeft.x + halfX, topLeft.y);
    node1_out.bottomLeft  = cv::Point2i(topLeft.x, topLeft.y + halfY);
    node1_out.bottomRight = cv::Point2i(topLeft.x + halfX, topLeft.y + halfY);
    node1_out.keys.reserve(keys.size());

    node2_out.topLeft     = node1_out.topRight;
    node2_out.topRight    = topRight;
    node2_out.bottomLeft  = node1_out.bottomRight;
    node2_out.bottomRight = cv::Point2i(topRight.x, topLeft.y + halfY);
    node2_out.keys.reserve(keys.size());

    node3_out.topLeft     = node1_out.bottomLeft;
    node3_out.topRight    = node1_out.bottomRight;
    node3_out.bottomLeft  = bottomLeft;
    node3_out.bottomRight = cv::Point2i(node1_out.bottomRight.x, bottomLeft.y);
    node3_out.keys.reserve(keys.size());

    node4_out.topLeft     = node3_out.topRight;
    node4_out.topRight    = node2_out.bottomRight;
    node4_out.bottomLeft  = node3_out.bottomRight;
    node4_out.bottomRight = bottomRight;
    node4_out.keys.reserve(keys.size());

    // Associate points to childs
    for (size_t i = 0; i < keys.size(); i++)
    {
        const cv::KeyPoint &kp = keys[i];
        if (kp.pt.x < node1_out.topRight.x)
        {
            if (kp.pt.y < node1_out.bottomRight.y)
                node1_out.keys.push_back(kp);
            else
                node3_out.keys.push_back(kp);
        }
        else if (kp.pt.y < node1_out.bottomRight.y)
            node2_out.keys.push_back(kp);
        else
            node4_out.keys.push_back(kp);
    }

    if (node1_out.keys.size() == 1)
        node1_out.isExhausted = true;
    if (node2_out.keys.size() == 1)
        node2_out.isExhausted = true;
    if (node3_out.keys.size() == 1)
        node3_out.isExhausted = true;
    if (node4_out.keys.size() == 1)
        node4_out.isExhausted = true;
}

} // namespace core
} // namespace vs_graphs
