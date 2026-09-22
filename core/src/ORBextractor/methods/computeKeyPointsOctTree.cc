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

const int PATCH_SIZE = 31;
const int EDGE_THRESHOLD = 19;

using namespace cv;
using namespace std;

namespace vs_graphs
{
namespace core
{

void ORBextractor::computeKeyPointsOctTree(
    vector<vector<KeyPoint>> &keypointsPerLevel_out)
{
    keypointsPerLevel_out.resize(levelCount);

    const float W = 35;

    for (int level = 0; level < levelCount; ++level)
    {
        const int minBorderX = EDGE_THRESHOLD - 3;
        const int minBorderY = minBorderX;
        const int maxBorderX = imagePyramid[level].cols - EDGE_THRESHOLD + 3;
        const int maxBorderY = imagePyramid[level].rows - EDGE_THRESHOLD + 3;

        vector<cv::KeyPoint> keysToDistribute_in;
        keysToDistribute_in.reserve(featureCount * 10);

        const float width  = (maxBorderX - minBorderX);
        const float height = (maxBorderY - minBorderY);

        const int nCols = width / W;
        const int nRows = height / W;
        const int wCell = ceil(width / nCols);
        const int hCell = ceil(height / nRows);

        for (int i = 0; i < nRows; i++)
        {
            const float iniY = minBorderY + i * hCell;
            float       maxY = iniY + hCell + 6;

            if (iniY >= maxBorderY - 3)
                continue;
            if (maxY > maxBorderY)
                maxY = maxBorderY;

            for (int j = 0; j < nCols; j++)
            {
                const float iniX = minBorderX + j * wCell;
                float       maxX = iniX + wCell + 6;
                if (iniX >= maxBorderX - 6)
                    continue;
                if (maxX > maxBorderX)
                    maxX = maxBorderX;

                vector<cv::KeyPoint> vKeysCell;

                FAST(imagePyramid[level]
                         .rowRange(iniY, maxY)
                         .colRange(iniX, maxX),
                     vKeysCell,
                     initialFastThreshold,
                     true);

                /*if(bRight && j <= 13){
                    FAST(imagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,10,true);
                }
                else if(!bRight && j >= 16){
                    FAST(imagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,10,true);
                }
                else{
                    FAST(imagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                         vKeysCell,initialFastThreshold,true);
                }*/

                if (vKeysCell.empty())
                {
                    FAST(imagePyramid[level]
                             .rowRange(iniY, maxY)
                             .colRange(iniX, maxX),
                         vKeysCell,
                         minimumFastThreshold,
                         true);
                    /*if(bRight && j <= 13){
                        FAST(imagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                             vKeysCell,5,true);
                    }
                    else if(!bRight && j >= 16){
                        FAST(imagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                             vKeysCell,5,true);
                    }
                    else{
                        FAST(imagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX),
                             vKeysCell,minimumFastThreshold,true);
                    }*/
                }

                if (!vKeysCell.empty())
                {
                    for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin();
                         vit != vKeysCell.end();
                         vit++)
                    {
                        (*vit).pt.x += j * wCell;
                        (*vit).pt.y += i * hCell;
                        keysToDistribute_in.push_back(*vit);
                    }
                }
            }
        }

        vector<KeyPoint> &keypoints = keypointsPerLevel_out[level];
        keypoints.reserve(featureCount);

        keypoints = distributeOctTree(keysToDistribute_in,
                                      minBorderX,
                                      maxBorderX,
                                      minBorderY,
                                      maxBorderY,
                                      featuresPerLevel[level],
                                      level);

        const int scaledPatchSize = PATCH_SIZE * scaleFactors[level];

        // Add border to coordinates and scale information
        const int nkps = keypoints.size();
        for (int i = 0; i < nkps; i++)
        {
            keypoints[i].pt.x += minBorderX;
            keypoints[i].pt.y += minBorderY;
            keypoints[i].octave = level;
            keypoints[i].size   = scaledPatchSize;
        }
    }

    // compute orientations
    for (int level = 0; level < levelCount; ++level)
        computeOrientation(imagePyramid[level],
                           keypointsPerLevel_out[level],
                           orientationMaxOffset);
}

} // namespace core
} // namespace vs_graphs
