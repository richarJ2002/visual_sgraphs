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

using namespace cv;
using namespace std;

namespace vs_graphs
{
namespace core
{

vector<cv::KeyPoint> ORBextractor::distributeOctTree(
    const vector<cv::KeyPoint> &keysToDistribute_in,
    const int                  &minX_in,
    const int                  &maxX_in,
    const int                  &minY_in,
    const int                  &maxY_in,
    const int                  &featureCount_in,
    [[maybe_unused]] const int &level_in)
{
    // Compute how many initial nodes
    const int nIni =
        round(static_cast<float>(maxX_in - minX_in) / (maxY_in - minY_in));

    const float hX = static_cast<float>(maxX_in - minX_in) / nIni;

    list<ExtractorNode> lNodes;

    vector<ExtractorNode *> vpIniNodes;
    vpIniNodes.resize(nIni);

    for (int i = 0; i < nIni; i++)
    {
        ExtractorNode ni;
        ni.topLeft     = cv::Point2i(hX * static_cast<float>(i), 0);
        ni.topRight    = cv::Point2i(hX * static_cast<float>(i + 1), 0);
        ni.bottomLeft  = cv::Point2i(ni.topLeft.x, maxY_in - minY_in);
        ni.bottomRight = cv::Point2i(ni.topRight.x, maxY_in - minY_in);
        ni.keys.reserve(keysToDistribute_in.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    // Associate points to childs
    for (size_t i = 0; i < keysToDistribute_in.size(); i++)
    {
        const cv::KeyPoint &kp = keysToDistribute_in[i];
        vpIniNodes[kp.pt.x / hX]->keys.push_back(kp);
    }

    list<ExtractorNode>::iterator nodeIterator = lNodes.begin();

    while (nodeIterator != lNodes.end())
    {
        if (nodeIterator->keys.size() == 1)
        {
            nodeIterator->isExhausted = true;
            nodeIterator++;
        }
        else if (nodeIterator->keys.empty())
            nodeIterator = lNodes.erase(nodeIterator);
        else
            nodeIterator++;
    }

    bool bFinish = false;

    int iteration = 0;

    vector<pair<int, ExtractorNode *>> vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size() * 4);

    while (!bFinish)
    {
        iteration++;

        int prevSize = lNodes.size();

        nodeIterator = lNodes.begin();

        int nToExpand = 0;

        vSizeAndPointerToNode.clear();

        while (nodeIterator != lNodes.end())
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
                nodeIterator->divideNode(node1_out,
                                         node2_out,
                                         node3_out,
                                         node4_out);

                // Add childs if they contain points
                if (node1_out.keys.size() > 0)
                {
                    lNodes.push_front(node1_out);
                    if (node1_out.keys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(node1_out.keys.size(), &lNodes.front()));
                        lNodes.front().nodeIterator = lNodes.begin();
                    }
                }
                if (node2_out.keys.size() > 0)
                {
                    lNodes.push_front(node2_out);
                    if (node2_out.keys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(node2_out.keys.size(), &lNodes.front()));
                        lNodes.front().nodeIterator = lNodes.begin();
                    }
                }
                if (node3_out.keys.size() > 0)
                {
                    lNodes.push_front(node3_out);
                    if (node3_out.keys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(node3_out.keys.size(), &lNodes.front()));
                        lNodes.front().nodeIterator = lNodes.begin();
                    }
                }
                if (node4_out.keys.size() > 0)
                {
                    lNodes.push_front(node4_out);
                    if (node4_out.keys.size() > 1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(
                            make_pair(node4_out.keys.size(), &lNodes.front()));
                        lNodes.front().nodeIterator = lNodes.begin();
                    }
                }

                nodeIterator = lNodes.erase(nodeIterator);
                continue;
            }
        }

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        if ((int)lNodes.size() >= featureCount_in ||
            (int)lNodes.size() == prevSize)
        {
            bFinish = true;
        }
        else if (((int)lNodes.size() + nToExpand * 3) > featureCount_in)
        {

            while (!bFinish)
            {

                prevSize = lNodes.size();

                vector<pair<int, ExtractorNode *>> vPrevSizeAndPointerToNode =
                    vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(),
                     vPrevSizeAndPointerToNode.end(),
                     compareNodes);
                for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--)
                {
                    ExtractorNode node1_out, node2_out, node3_out, node4_out;
                    vPrevSizeAndPointerToNode[j].second->divideNode(node1_out,
                                                                    node2_out,
                                                                    node3_out,
                                                                    node4_out);

                    // Add childs if they contain points
                    if (node1_out.keys.size() > 0)
                    {
                        lNodes.push_front(node1_out);
                        if (node1_out.keys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                make_pair(node1_out.keys.size(),
                                          &lNodes.front()));
                            lNodes.front().nodeIterator = lNodes.begin();
                        }
                    }
                    if (node2_out.keys.size() > 0)
                    {
                        lNodes.push_front(node2_out);
                        if (node2_out.keys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                make_pair(node2_out.keys.size(),
                                          &lNodes.front()));
                            lNodes.front().nodeIterator = lNodes.begin();
                        }
                    }
                    if (node3_out.keys.size() > 0)
                    {
                        lNodes.push_front(node3_out);
                        if (node3_out.keys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                make_pair(node3_out.keys.size(),
                                          &lNodes.front()));
                            lNodes.front().nodeIterator = lNodes.begin();
                        }
                    }
                    if (node4_out.keys.size() > 0)
                    {
                        lNodes.push_front(node4_out);
                        if (node4_out.keys.size() > 1)
                        {
                            vSizeAndPointerToNode.push_back(
                                make_pair(node4_out.keys.size(),
                                          &lNodes.front()));
                            lNodes.front().nodeIterator = lNodes.begin();
                        }
                    }

                    lNodes.erase(
                        vPrevSizeAndPointerToNode[j].second->nodeIterator);

                    if ((int)lNodes.size() >= featureCount_in)
                        break;
                }

                if ((int)lNodes.size() >= featureCount_in ||
                    (int)lNodes.size() == prevSize)
                    bFinish = true;
            }
        }
    }

    // Retain the best point in each node
    vector<cv::KeyPoint> vResultKeys;
    vResultKeys.reserve(featureCount);
    for (list<ExtractorNode>::iterator nodeIterator = lNodes.begin();
         nodeIterator != lNodes.end();
         nodeIterator++)
    {
        vector<cv::KeyPoint> &vNodeKeys   = nodeIterator->keys;
        cv::KeyPoint         *pKP         = &vNodeKeys[0];
        float                 maxResponse = pKP->response;

        for (size_t k = 1; k < vNodeKeys.size(); k++)
        {
            if (vNodeKeys[k].response > maxResponse)
            {
                pKP         = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}

} // namespace core
} // namespace vs_graphs
