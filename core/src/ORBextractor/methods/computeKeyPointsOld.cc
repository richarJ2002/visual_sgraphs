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

void ORBextractor::computeKeyPointsOld(
    std::vector<std::vector<KeyPoint>> &keypointsPerLevel_out)
{
    keypointsPerLevel_out.resize(levelCount);

    float imageRatio = (float)imagePyramid[0].cols / imagePyramid[0].rows;

    for (int level = 0; level < levelCount; ++level)
    {
        const int nDesiredFeatures = featuresPerLevel[level];

        const int levelCols = sqrt((float)nDesiredFeatures / (5 * imageRatio));
        const int levelRows = imageRatio * levelCols;

        const int minBorderX = EDGE_THRESHOLD;
        const int minBorderY = minBorderX;
        const int maxBorderX = imagePyramid[level].cols - EDGE_THRESHOLD;
        const int maxBorderY = imagePyramid[level].rows - EDGE_THRESHOLD;

        const int W     = maxBorderX - minBorderX;
        const int H     = maxBorderY - minBorderY;
        const int cellW = ceil((float)W / levelCols);
        const int cellH = ceil((float)H / levelRows);

        const int nCells        = levelRows * levelCols;
        const int nfeaturesCell = ceil((float)nDesiredFeatures / nCells);

        vector<vector<vector<KeyPoint>>> cellKeyPoints(
            levelRows,
            vector<vector<KeyPoint>>(levelCols));

        vector<vector<int>>  nToRetain(levelRows, vector<int>(levelCols, 0));
        vector<vector<int>>  nTotal(levelRows, vector<int>(levelCols, 0));
        vector<vector<bool>> isExhausted(levelRows,
                                         vector<bool>(levelCols, false));
        vector<int>          iniXCol(levelCols);
        vector<int>          iniYRow(levelRows);
        int                  nNoMore       = 0;
        int                  nToDistribute = 0;

        float hY = cellH + 6;

        for (int i = 0; i < levelRows; i++)
        {
            const float iniY = minBorderY + i * cellH - 3;
            iniYRow[i]       = iniY;

            if (i == levelRows - 1)
            {
                hY = maxBorderY + 3 - iniY;
                if (hY <= 0)
                    continue;
            }

            float hX = cellW + 6;

            for (int j = 0; j < levelCols; j++)
            {
                float iniX;

                if (i == 0)
                {
                    iniX       = minBorderX + j * cellW - 3;
                    iniXCol[j] = iniX;
                }
                else
                {
                    iniX = iniXCol[j];
                }

                if (j == levelCols - 1)
                {
                    hX = maxBorderX + 3 - iniX;
                    if (hX <= 0)
                        continue;
                }

                Mat cellImage = imagePyramid[level]
                                    .rowRange(iniY, iniY + hY)
                                    .colRange(iniX, iniX + hX);

                cellKeyPoints[i][j].reserve(nfeaturesCell * 5);

                FAST(cellImage,
                     cellKeyPoints[i][j],
                     initialFastThreshold,
                     true);

                if (cellKeyPoints[i][j].size() <= 3)
                {
                    cellKeyPoints[i][j].clear();

                    FAST(cellImage,
                         cellKeyPoints[i][j],
                         minimumFastThreshold,
                         true);
                }

                const int nKeys = cellKeyPoints[i][j].size();
                nTotal[i][j]    = nKeys;

                if (nKeys > nfeaturesCell)
                {
                    nToRetain[i][j]   = nfeaturesCell;
                    isExhausted[i][j] = false;
                }
                else
                {
                    nToRetain[i][j] = nKeys;
                    nToDistribute += nfeaturesCell - nKeys;
                    isExhausted[i][j] = true;
                    nNoMore++;
                }
            }
        }

        // Retain by score

        while (nToDistribute > 0 && nNoMore < nCells)
        {
            int nNewFeaturesCell =
                nfeaturesCell + ceil((float)nToDistribute / (nCells - nNoMore));
            nToDistribute = 0;

            for (int i = 0; i < levelRows; i++)
            {
                for (int j = 0; j < levelCols; j++)
                {
                    if (!isExhausted[i][j])
                    {
                        if (nTotal[i][j] > nNewFeaturesCell)
                        {
                            nToRetain[i][j]   = nNewFeaturesCell;
                            isExhausted[i][j] = false;
                        }
                        else
                        {
                            nToRetain[i][j] = nTotal[i][j];
                            nToDistribute += nNewFeaturesCell - nTotal[i][j];
                            isExhausted[i][j] = true;
                            nNoMore++;
                        }
                    }
                }
            }
        }

        vector<KeyPoint> &keypoints = keypointsPerLevel_out[level];
        keypoints.reserve(nDesiredFeatures * 2);

        const int scaledPatchSize = PATCH_SIZE * scaleFactors[level];

        // Retain by score and transform coordinates
        for (int i = 0; i < levelRows; i++)
        {
            for (int j = 0; j < levelCols; j++)
            {
                vector<KeyPoint> &keysCell = cellKeyPoints[i][j];
                KeyPointsFilter::retainBest(keysCell, nToRetain[i][j]);
                if ((int)keysCell.size() > nToRetain[i][j])
                    keysCell.resize(nToRetain[i][j]);

                for (size_t k = 0, kend = keysCell.size(); k < kend; k++)
                {
                    keysCell[k].pt.x += iniXCol[j];
                    keysCell[k].pt.y += iniYRow[i];
                    keysCell[k].octave = level;
                    keysCell[k].size   = scaledPatchSize;
                    keypoints.push_back(keysCell[k]);
                }
            }
        }

        if ((int)keypoints.size() > nDesiredFeatures)
        {
            KeyPointsFilter::retainBest(keypoints, nDesiredFeatures);
            keypoints.resize(nDesiredFeatures);
        }
    }

    // and compute orientations
    for (int level = 0; level < levelCount; ++level)
        computeOrientation(imagePyramid[level],
                           keypointsPerLevel_out[level],
                           orientationMaxOffset);
}

} // namespace core
} // namespace vs_graphs
