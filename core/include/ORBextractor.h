/**
 * This file is part of ORB-SLAM3.
 * Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details: https://www.gnu.org/licenses/
 */

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <cstdint>
#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

namespace vs_graphs
{
namespace core
{

    class ExtractorNode
    {
    public:
        ExtractorNode() : isExhausted(false) {}

        void DivideNode(ExtractorNode &node1_out, ExtractorNode &node2_out, ExtractorNode &node3_out, ExtractorNode &node4_out);

        std::vector<cv::KeyPoint> keys;
        cv::Point2i topLeft, topRight, bottomLeft, bottomRight;
        std::list<ExtractorNode>::iterator nodeIterator;
        bool isExhausted;
    };

    class ORBextractor
    {
    public:
        enum class Score : std::uint8_t
        {
            HARRIS_SCORE = 0U,
            FAST_SCORE   = 1U
        };

        ORBextractor(int featureCount_in, float scaleFactor_in, int levelCount_in,
                     int initialFastThreshold_in, int minimumFastThreshold_in);

        ~ORBextractor() {}

        // Compute the ORB features and descriptors on an image.
        // ORB are dispersed on the image using an octree.
        // Mask is ignored in the current implementation.
        int operator()(cv::InputArray image_in, cv::InputArray mask_in,
                       std::vector<cv::KeyPoint> &keypoints_out,
                       cv::OutputArray descriptors_out, std::vector<int> &lappingArea_in);

        int inline getLevelCount()
        {
            return levelCount;
        }

        float inline getScaleFactor()
        {
            return scaleFactor;
        }

        std::vector<float> inline getScaleFactors()
        {
            return scaleFactors;
        }

        std::vector<float> inline getInverseScaleFactors()
        {
            return inverseScaleFactors;
        }

        std::vector<float> inline getScaleSigmaSquares()
        {
            return levelSigmaSquares;
        }

        std::vector<float> inline getInverseScaleSigmaSquares()
        {
            return inverseLevelSigmaSquares;
        }

        std::vector<cv::Mat> imagePyramid;

    protected:
        void computePyramid(cv::Mat image_in);
        void computeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>> &keypointsPerLevel_out);
        std::vector<cv::KeyPoint> distributeOctTree(const std::vector<cv::KeyPoint> &keysToDistribute_in, const int &minX_in,
                                                    const int &maxX_in, const int &minY_in, const int &maxY_in, const int &featureCount_in, const int &level_in);

        void computeKeyPointsOld(std::vector<std::vector<cv::KeyPoint>> &keypointsPerLevel_out);
        std::vector<cv::Point> briefPattern;

        int featureCount;
        double scaleFactor;
        int levelCount;
        int initialFastThreshold;
        int minimumFastThreshold;

        std::vector<int> featuresPerLevel;

        std::vector<int> orientationMaxOffset;

        std::vector<float> scaleFactors;
        std::vector<float> inverseScaleFactors;
        std::vector<float> levelSigmaSquares;
        std::vector<float> inverseLevelSigmaSquares;

    public:
        // Adaptive FAST threshold: dynamically adjust thresholds when tracking degrades
        void setInitialFastThreshold(int threshold_in) { initialFastThreshold = threshold_in; }
        void setMinimumFastThreshold(int threshold_in) { minimumFastThreshold = threshold_in; }
        int getInitialFastThreshold() const { return initialFastThreshold; }
        int getMinimumFastThreshold() const { return minimumFastThreshold; }
    };

} // namespace core
} // namespace vs_graphs

#endif
