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
 * @file            divideNode.cc
 *
 * @brief           Implements ExtractorNode::divideNode(), declared in
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
#include <vector>

namespace vs_graphs
{
namespace core
{

ExtractorNodeStatus ExtractorNode::divideNode(ExtractorNode &node1_inout,
                                              ExtractorNode &node2_inout,
                                              ExtractorNode &node3_inout,
                                              ExtractorNode &node4_inout)
{
    const int halfX = std::ceil(static_cast<float>(topRight.x - topLeft.x) / 2);
    const int halfY =
        std::ceil(static_cast<float>(bottomRight.y - topLeft.y) / 2);

    // Define boundaries of childs
    node1_inout.topLeft     = topLeft;
    node1_inout.topRight    = cv::Point2i(topLeft.x + halfX, topLeft.y);
    node1_inout.bottomLeft  = cv::Point2i(topLeft.x, topLeft.y + halfY);
    node1_inout.bottomRight = cv::Point2i(topLeft.x + halfX, topLeft.y + halfY);
    node1_inout.keys.reserve(keys.size());

    node2_inout.topLeft     = node1_inout.topRight;
    node2_inout.topRight    = topRight;
    node2_inout.bottomLeft  = node1_inout.bottomRight;
    node2_inout.bottomRight = cv::Point2i(topRight.x, topLeft.y + halfY);
    node2_inout.keys.reserve(keys.size());

    node3_inout.topLeft    = node1_inout.bottomLeft;
    node3_inout.topRight   = node1_inout.bottomRight;
    node3_inout.bottomLeft = bottomLeft;
    node3_inout.bottomRight =
        cv::Point2i(node1_inout.bottomRight.x, bottomLeft.y);
    node3_inout.keys.reserve(keys.size());

    node4_inout.topLeft     = node3_inout.topRight;
    node4_inout.topRight    = node2_inout.bottomRight;
    node4_inout.bottomLeft  = node3_inout.bottomRight;
    node4_inout.bottomRight = bottomRight;
    node4_inout.keys.reserve(keys.size());

    // Associate points to childs
    for (size_t keyIndex = 0; keyIndex < keys.size(); keyIndex++)
    {
        const cv::KeyPoint &keyPoint = keys[keyIndex];
        if (keyPoint.pt.x < node1_inout.topRight.x)
        {
            if (keyPoint.pt.y < node1_inout.bottomRight.y)
                node1_inout.keys.push_back(keyPoint);
            else
                node3_inout.keys.push_back(keyPoint);
        }
        else if (keyPoint.pt.y < node1_inout.bottomRight.y)
            node2_inout.keys.push_back(keyPoint);
        else
            node4_inout.keys.push_back(keyPoint);
    }

    if (node1_inout.keys.size() == 1)
        node1_inout.isExhausted = true;
    if (node2_inout.keys.size() == 1)
        node2_inout.isExhausted = true;
    if (node3_inout.keys.size() == 1)
        node3_inout.isExhausted = true;
    if (node4_inout.keys.size() == 1)
        node4_inout.isExhausted = true;

    return ExtractorNodeStatus::EXTRACTOR_NODE_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
