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
#include <rclcpp/logging.hpp>
#include <vector>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{

ORBextractorStatus ORBextractor::computeKeyPointsOld(
    std::vector<std::vector<cv::KeyPoint>> &keypointsPerLevel_inout)
{
    keypointsPerLevel_inout.resize(levelCount);

    float imageRatio =
        static_cast<float>(imagePyramid[0].cols) / imagePyramid[0].rows;

    for (int level = 0; level < levelCount; ++level)
    {
        const int desiredFeatureCount = featuresPerLevel[level];

        const int levelCols = std::sqrt(
            static_cast<float>(desiredFeatureCount) / (5 * imageRatio));
        const int levelRows = imageRatio * levelCols;

        const int minimumBorderX = EDGE_THRESHOLD;
        const int minimumBorderY = minimumBorderX;
        const int maximumBorderX = imagePyramid[level].cols - EDGE_THRESHOLD;
        const int maximumBorderY = imagePyramid[level].rows - EDGE_THRESHOLD;

        const int W     = maximumBorderX - minimumBorderX;
        const int H     = maximumBorderY - minimumBorderY;
        const int cellW = std::ceil(static_cast<float>(W) / levelCols);
        const int cellH = std::ceil(static_cast<float>(H) / levelRows);

        const int cellCount = levelRows * levelCols;
        const int nfeaturesCell =
            std::ceil(static_cast<float>(desiredFeatureCount) / cellCount);

        std::vector<std::vector<std::vector<cv::KeyPoint>>> cellKeyPoints(
            levelRows,
            std::vector<std::vector<cv::KeyPoint>>(levelCols));

        std::vector<std::vector<int>> toRetainCount(
            levelRows,
            std::vector<int>(levelCols, 0));
        std::vector<std::vector<int>> totalCount(
            levelRows,
            std::vector<int>(levelCols, 0));
        std::vector<std::vector<bool>> isExhausted(
            levelRows,
            std::vector<bool>(levelCols, false));
        std::vector<int> initialXCol(levelCols);
        std::vector<int> initialYRow(levelRows);
        int              noMoreCount       = 0;
        int              toDistributeCount = 0;

        float hY = cellH + 6;

        for (int rowIndex = 0; rowIndex < levelRows; rowIndex++)
        {
            const float initialY  = minimumBorderY + rowIndex * cellH - 3;
            initialYRow[rowIndex] = initialY;

            if (rowIndex == levelRows - 1)
            {
                hY = maximumBorderY + 3 - initialY;
                if (hY <= 0)
                    continue;
            }

            float hX = cellW + 6;

            for (int columnIndex = 0; columnIndex < levelCols; columnIndex++)
            {
                float initialX;

                if (rowIndex == 0)
                {
                    initialX = minimumBorderX + columnIndex * cellW - 3;
                    initialXCol[columnIndex] = initialX;
                }
                else
                {
                    initialX = initialXCol[columnIndex];
                }

                if (columnIndex == levelCols - 1)
                {
                    hX = maximumBorderX + 3 - initialX;
                    if (hX <= 0)
                        continue;
                }

                cv::Mat cellImage = imagePyramid[level]
                                        .rowRange(initialY, initialY + hY)
                                        .colRange(initialX, initialX + hX);

                cellKeyPoints[rowIndex][columnIndex].reserve(nfeaturesCell * 5);

                cv::FAST(cellImage,
                         cellKeyPoints[rowIndex][columnIndex],
                         initialFastThreshold,
                         true);

                if (cellKeyPoints[rowIndex][columnIndex].size() <= 3)
                {
                    cellKeyPoints[rowIndex][columnIndex].clear();

                    cv::FAST(cellImage,
                             cellKeyPoints[rowIndex][columnIndex],
                             minimumFastThreshold,
                             true);
                }

                const int keyCount =
                    cellKeyPoints[rowIndex][columnIndex].size();
                totalCount[rowIndex][columnIndex] = keyCount;

                if (keyCount > nfeaturesCell)
                {
                    toRetainCount[rowIndex][columnIndex] = nfeaturesCell;
                    isExhausted[rowIndex][columnIndex]   = false;
                }
                else
                {
                    toRetainCount[rowIndex][columnIndex] = keyCount;
                    toDistributeCount += nfeaturesCell - keyCount;
                    isExhausted[rowIndex][columnIndex] = true;
                    noMoreCount++;
                }
            }
        }

        // Retain by score

        while (toDistributeCount > 0 && noMoreCount < cellCount)
        {
            int newFeaturesCellCount =
                nfeaturesCell +
                std::ceil(static_cast<float>(toDistributeCount) /
                          (cellCount - noMoreCount));
            toDistributeCount = 0;

            for (int rowIndex = 0; rowIndex < levelRows; rowIndex++)
            {
                for (int columnIndex = 0; columnIndex < levelCols;
                     columnIndex++)
                {
                    if (!isExhausted[rowIndex][columnIndex])
                    {
                        if (totalCount[rowIndex][columnIndex] >
                            newFeaturesCellCount)
                        {
                            toRetainCount[rowIndex][columnIndex] =
                                newFeaturesCellCount;
                            isExhausted[rowIndex][columnIndex] = false;
                        }
                        else
                        {
                            toRetainCount[rowIndex][columnIndex] =
                                totalCount[rowIndex][columnIndex];
                            toDistributeCount +=
                                newFeaturesCellCount -
                                totalCount[rowIndex][columnIndex];
                            isExhausted[rowIndex][columnIndex] = true;
                            noMoreCount++;
                        }
                    }
                }
            }
        }

        std::vector<cv::KeyPoint> &keypoints = keypointsPerLevel_inout[level];
        keypoints.reserve(desiredFeatureCount * 2);

        const int scaledPatchSize = PATCH_SIZE * scaleFactors[level];

        // Retain by score and transform coordinates
        for (int rowIndex = 0; rowIndex < levelRows; rowIndex++)
        {
            for (int columnIndex = 0; columnIndex < levelCols; columnIndex++)
            {
                std::vector<cv::KeyPoint> &keysCell =
                    cellKeyPoints[rowIndex][columnIndex];
                cv::KeyPointsFilter::retainBest(
                    keysCell,
                    toRetainCount[rowIndex][columnIndex]);
                if (static_cast<int>(keysCell.size()) >
                    toRetainCount[rowIndex][columnIndex])
                    keysCell.resize(toRetainCount[rowIndex][columnIndex]);

                for (size_t cellKeyPointIndex = 0, kend = keysCell.size();
                     cellKeyPointIndex < kend;
                     cellKeyPointIndex++)
                {
                    keysCell[cellKeyPointIndex].pt.x +=
                        initialXCol[columnIndex];
                    keysCell[cellKeyPointIndex].pt.y += initialYRow[rowIndex];
                    keysCell[cellKeyPointIndex].octave = level;
                    keysCell[cellKeyPointIndex].size   = scaledPatchSize;
                    keypoints.push_back(keysCell[cellKeyPointIndex]);
                }
            }
        }

        if (static_cast<int>(keypoints.size()) > desiredFeatureCount)
        {
            cv::KeyPointsFilter::retainBest(keypoints, desiredFeatureCount);
            keypoints.resize(desiredFeatureCount);
        }
    }

    // and compute orientations
    for (int level = 0; level < levelCount; ++level)
    {
        if (computeOrientation(imagePyramid[level],
                               keypointsPerLevel_inout[level],
                               orientationMaxOffset) !=
            ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: computeOrientation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }

    return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
