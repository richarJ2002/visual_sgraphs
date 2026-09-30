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
 * @file            distributeOctTree.cc
 *
 * @brief           Implements ORBextractor::distributeOctTree(), declared in
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

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{

ORBextractorStatus ORBextractor::distributeOctTree(
    const std::vector<cv::KeyPoint> &keysToDistribute_in,
    const int                       &minimumX_in,
    const int                       &maximumX_in,
    const int                       &minimumY_in,
    const int                       &maximumY_in,
    const int                       &featureCount_in,
    [[maybe_unused]] const int      &level_in,
    std::vector<cv::KeyPoint>       &keyPoints_out) const
{
    // Compute how many initial nodes
    const int initialNodeCount =
        std::round(static_cast<float>(maximumX_in - minimumX_in) /
                   (maximumY_in - minimumY_in));

    const float hX =
        static_cast<float>(maximumX_in - minimumX_in) / initialNodeCount;

    std::list<ExtractorNode> nodes;

    std::vector<ExtractorNode *> initialNodes;
    initialNodes.resize(initialNodeCount);

    for (int keyPointIndex = 0; keyPointIndex < initialNodeCount;
         keyPointIndex++)
    {
        ExtractorNode ni;
        ni.topLeft = cv::Point2i(hX * static_cast<float>(keyPointIndex), 0);
        ni.topRight =
            cv::Point2i(hX * static_cast<float>(keyPointIndex + 1), 0);
        ni.bottomLeft  = cv::Point2i(ni.topLeft.x, maximumY_in - minimumY_in);
        ni.bottomRight = cv::Point2i(ni.topRight.x, maximumY_in - minimumY_in);
        ni.keys.reserve(keysToDistribute_in.size());

        nodes.push_back(ni);
        initialNodes[keyPointIndex] = &nodes.back();
    }

    // Associate points to childs
    for (size_t keyPointIndex = 0; keyPointIndex < keysToDistribute_in.size();
         keyPointIndex++)
    {
        const cv::KeyPoint &keyPoint = keysToDistribute_in[keyPointIndex];
        initialNodes[keyPoint.pt.x / hX]->keys.push_back(keyPoint);
    }

    std::list<ExtractorNode>::iterator nodeIterator = nodes.begin();

    while (nodeIterator != nodes.end())
    {
        if (nodeIterator->keys.size() == 1)
        {
            nodeIterator->isExhausted = true;
            nodeIterator++;
        }
        else if (nodeIterator->keys.empty())
            nodeIterator = nodes.erase(nodeIterator);
        else
            nodeIterator++;
    }

    bool isFinished = false;

    int iteration = 0;

    std::vector<std::pair<int, ExtractorNode *>> vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(nodes.size() * 4);

    while (!isFinished)
    {
        iteration++;

        int previousSize = nodes.size();

        nodeIterator = nodes.begin();

        int toExpandCount = 0;

        vSizeAndPointerToNode.clear();

        while (nodeIterator != nodes.end())
        {
            if (nodeIterator->isExhausted)
            {
                // If node only contains one point do not subdivide and continue
                nodeIterator++;
                continue;
            }
            else
            {
                // If more than one point, subdivide
                ExtractorNode node1_out, node2_out, node3_out, node4_out;
                if (nodeIterator->divideNode(node1_out,
                                             node2_out,
                                             node3_out,
                                             node4_out) !=
                    ExtractorNodeStatus::EXTRACTOR_NODE_STATUS_SUCCESS)
                {
                    RCLCPP_ERROR(
                        rclcpp::get_logger("vs_graphs"),
                        "%s: divideNode returned a failure status although it "
                        "cannot fail; continuing as before.",
                        __func__);
                }

                // Add childs if they contain points
                if (node1_out.keys.size() > 0)
                {
                    nodes.push_front(node1_out);
                    if (node1_out.keys.size() > 1)
                    {
                        toExpandCount++;
                        vSizeAndPointerToNode.push_back(
                            std::make_pair(node1_out.keys.size(),
                                           &nodes.front()));
                        nodes.front().nodeIterator = nodes.begin();
                    }
                }
                if (node2_out.keys.size() > 0)
                {
                    nodes.push_front(node2_out);
                    if (node2_out.keys.size() > 1)
                    {
                        toExpandCount++;
                        vSizeAndPointerToNode.push_back(
                            std::make_pair(node2_out.keys.size(),
                                           &nodes.front()));
                        nodes.front().nodeIterator = nodes.begin();
                    }
                }
                if (node3_out.keys.size() > 0)
                {
                    nodes.push_front(node3_out);
                    if (node3_out.keys.size() > 1)
                    {
                        toExpandCount++;
                        vSizeAndPointerToNode.push_back(
                            std::make_pair(node3_out.keys.size(),
                                           &nodes.front()));
                        nodes.front().nodeIterator = nodes.begin();
                    }
                }
                if (node4_out.keys.size() > 0)
                {
                    nodes.push_front(node4_out);
                    if (node4_out.keys.size() > 1)
                    {
                        toExpandCount++;
                        vSizeAndPointerToNode.push_back(
                            std::make_pair(node4_out.keys.size(),
                                           &nodes.front()));
                        nodes.front().nodeIterator = nodes.begin();
                    }
                }

                nodeIterator = nodes.erase(nodeIterator);
                continue;
            }
        }

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        if (static_cast<int>(nodes.size()) >= featureCount_in ||
            static_cast<int>(nodes.size()) == previousSize)
        {
            isFinished = true;
        }
        else if ((static_cast<int>(nodes.size()) + toExpandCount * 3) >
                 featureCount_in)
        {

            while (!isFinished)
            {

                previousSize = nodes.size();

                std::vector<std::pair<int, ExtractorNode *>>
                    vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                std::sort(vPrevSizeAndPointerToNode.begin(),
                          vPrevSizeAndPointerToNode.end(),
                          compareNodes);
                for (int nodeIndex = vPrevSizeAndPointerToNode.size() - 1;
                     nodeIndex >= 0;
                     nodeIndex--)
                {
                    ExtractorNode node1_out, node2_out, node3_out, node4_out;
                    if (vPrevSizeAndPointerToNode[nodeIndex].second->divideNode(
                            node1_out,
                            node2_out,
                            node3_out,
                            node4_out) !=
                        ExtractorNodeStatus::EXTRACTOR_NODE_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("vs_graphs"),
                            "%s: divideNode returned a failure status although "
                            "it cannot fail; continuing as before.",
                            __func__);
                    }

                    // Add childs if they contain points
                    if (node1_out.keys.size() > 0)
                    {
                        nodes.push_front(node1_out);
                        if (node1_out.keys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                std::make_pair(node1_out.keys.size(),
                                               &nodes.front()));
                            nodes.front().nodeIterator = nodes.begin();
                        }
                    }
                    if (node2_out.keys.size() > 0)
                    {
                        nodes.push_front(node2_out);
                        if (node2_out.keys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                std::make_pair(node2_out.keys.size(),
                                               &nodes.front()));
                            nodes.front().nodeIterator = nodes.begin();
                        }
                    }
                    if (node3_out.keys.size() > 0)
                    {
                        nodes.push_front(node3_out);
                        if (node3_out.keys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                std::make_pair(node3_out.keys.size(),
                                               &nodes.front()));
                            nodes.front().nodeIterator = nodes.begin();
                        }
                    }
                    if (node4_out.keys.size() > 0)
                    {
                        nodes.push_front(node4_out);
                        if (node4_out.keys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                std::make_pair(node4_out.keys.size(),
                                               &nodes.front()));
                            nodes.front().nodeIterator = nodes.begin();
                        }
                    }

                    nodes.erase(vPrevSizeAndPointerToNode[nodeIndex]
                                    .second->nodeIterator);

                    if (static_cast<int>(nodes.size()) >= featureCount_in)
                        break;
                }

                if (static_cast<int>(nodes.size()) >= featureCount_in ||
                    static_cast<int>(nodes.size()) == previousSize)
                    isFinished = true;
            }
        }
    }

    // Retain the best point in each node
    std::vector<cv::KeyPoint> resultKeys;
    resultKeys.reserve(featureCount);
    for (std::list<ExtractorNode>::iterator finalNodeIterator = nodes.begin();
         finalNodeIterator != nodes.end();
         finalNodeIterator++)
    {
        std::vector<cv::KeyPoint> &nodeKeys        = finalNodeIterator->keys;
        cv::KeyPoint              *p_keyPoint      = &nodeKeys[0];
        float                      maximumResponse = p_keyPoint->response;

        for (size_t nodeKeyIndex = 1; nodeKeyIndex < nodeKeys.size();
             nodeKeyIndex++)
        {
            if (nodeKeys[nodeKeyIndex].response > maximumResponse)
            {
                p_keyPoint      = &nodeKeys[nodeKeyIndex];
                maximumResponse = nodeKeys[nodeKeyIndex].response;
            }
        }

        resultKeys.push_back(*p_keyPoint);
    }

    keyPoints_out = resultKeys;
    return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
