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
 * @file         ORBextractor.h
 *
 * @brief        Declares the ORB feature extractor.
 */

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include "ExtractorNodeStatus.h"
#include "ORBextractorStatus.h"
#include <cstdint>
#include <list>
#include <opencv2/core.hpp>
#include <vector>

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Image region node of the octree used to spread
 *               keypoints.
 */
class ExtractorNode
{
  public:
    /*!
     * @brief        Creates an unexhausted node.
     */
    ExtractorNode() :
        isExhausted(false)
    {}

    /*!
     * @brief        Splits the node into four quadrant children.
     *
     * @param[in,out] node1_inout
     *               Top-left child.
     * @param[in,out] node2_inout
     *               Top-right child.
     * @param[in,out] node3_inout
     *               Bottom-left child.
     * @param[in,out] node4_inout
     *               Bottom-right child.
     */
    [[nodiscard]] ExtractorNodeStatus divideNode(ExtractorNode &node1_inout,
                                                 ExtractorNode &node2_inout,
                                                 ExtractorNode &node3_inout,
                                                 ExtractorNode &node4_inout);

    /*!
     * @brief        Keypoints falling inside the node.
     */
    std::vector<cv::KeyPoint> keys;
    /*!
     * @brief        Region corners in pixels.
     */
    cv::Point2i               topLeft, topRight, bottomLeft, bottomRight;
    /*!
     * @brief        Position of the node in the level list.
     */
    std::list<ExtractorNode>::iterator nodeIterator;
    /*!
     * @brief        True once the node holds no distributable
     *               keypoints.
     */
    bool                               isExhausted;
};

/*!
 * @brief        Extracts scale-pyramid ORB features and
 *               descriptors from grayscale images.
 */
class ORBextractor
{
  public:
    /*!
     * @brief        Keypoint scoring scheme.
     */
    enum class Score : std::uint8_t
    {
        /*!
         * @brief        Harris corner response scoring.
         */
        HARRIS_SCORE = 0U,
        /*!
         * @brief        FAST corner response scoring.
         */
        FAST_SCORE = 1U
    };

    /*!
     * @brief        Creates an extractor and precomputes the
     *               pyramid scales and level budgets.
     *
     * @param[in]    featureCount_in
     *               Target number of features.
     * @param[in]    scaleFactor_in
     *               Scale step between pyramid levels.
     * @param[in]    levelCount_in
     *               Number of pyramid levels.
     * @param[in]    initialFastThreshold_in
     *               FAST threshold used at extraction.
     * @param[in]    minimumFastThreshold_in
     *               FAST threshold used when extraction is
     *               retried.
     */
    ORBextractor(int   featureCount_in,
                 float scaleFactor_in,
                 int   levelCount_in,
                 int   initialFastThreshold_in,
                 int   minimumFastThreshold_in);

    /*!
     * @brief        Destroys the extractor.
     */
    ~ORBextractor() {}

    /*!
     * @brief        Extracts ORB features dispersed over the
     *               image with an octree.
     *
     * @param[in]    image_in
     *               Image to process; shall be a non-empty
     *               eight-bit single-channel image.
     * @param[in]    mask_in
     *               Ignored by the current implementation.
     * @param[in,out] keypoints_inout
     *               Extracted keypoints in image coordinates.
     * @param[in]    descriptors_in
     *               One thirty-two-byte row per keypoint;
     *               released when no keypoint is found.
     * @param[in]    lappingArea_in
     *               Stereo overlap column bounds used to order
     *               mono and stereo keypoints.
     *
     * @return       Number of keypoints outside the stereo
     *               overlap, or minus one for an empty image.
     */
    int operator()(cv::InputArray             image_in,
                   cv::InputArray             mask_in,
                   std::vector<cv::KeyPoint> &keypoints_inout,
                   cv::OutputArray            descriptors_in,
                   std::vector<int>          &lappingArea_in);

    /*!
     * @brief        Returns the number of pyramid levels.
     *
     * @param[out] levelCount_out Configured level count.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus getLevelCount(int &levelCount_out)
    {
        levelCount_out = levelCount;
        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the pyramid scale step.
     *
     * @param[out] scaleFactor_out Configured scale factor.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus getScaleFactor(float &scaleFactor_out)
    {
        scaleFactor_out = scaleFactor;
        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the per-level scale factors.
     *
     * @param[out] scaleFactors_out Scale factor of every pyramid level.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus
        getScaleFactors(std::vector<float> &scaleFactors_out)
    {
        scaleFactors_out = scaleFactors;
        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the per-level inverse scale factors.
     *
     * @param[out] inverseScaleFactors_out Inverse scale factor of every pyramid
     * level.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus
        getInverseScaleFactors(std::vector<float> &inverseScaleFactors_out)
    {
        inverseScaleFactors_out = inverseScaleFactors;
        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the per-level scale sigma squares.
     *
     * @param[out] scaleSigmaSquares_out Sigma square of every pyramid level.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus
        getScaleSigmaSquares(std::vector<float> &scaleSigmaSquares_out)
    {
        scaleSigmaSquares_out = levelSigmaSquares;
        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }

    /*!
     * @brief        Returns the per-level inverse sigma squares.
     *
     * @param[out] inverseScaleSigmaSquares_out Inverse sigma square of every
     * pyramid level.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus getInverseScaleSigmaSquares(
        std::vector<float> &inverseScaleSigmaSquares_out)
    {
        inverseScaleSigmaSquares_out = inverseLevelSigmaSquares;
        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }

    /*!
     * @brief        Grayscale pyramid with one image per level.
     */
    std::vector<cv::Mat> imagePyramid;

  protected:
    /*!
     * @brief        Builds the scaled image pyramid.
     *
     * @param[in]    image_in
     *               Full-resolution source image.
     */
    [[nodiscard]] ORBextractorStatus computePyramid(cv::Mat image_in);
    /*!
     * @brief        Detects keypoints on every pyramid level
     *               with the octree distribution.
     *
     * @param[in,out] keypointsPerLevel_inout
     *               Detected keypoints grouped by level.
     */
    [[nodiscard]] ORBextractorStatus computeKeyPointsOctTree(
        std::vector<std::vector<cv::KeyPoint>> &keypointsPerLevel_inout);
    /*!
     * @brief        Distributes keypoints inside a region with
     *               the octree.
     *
     * @param[in]    keysToDistribute_in
     *               Candidate keypoints of the region.
     * @param[in]    minimumX_in
     *               Region left bound in pixels.
     * @param[in]    maximumX_in
     *               Region right bound in pixels.
     * @param[in]    minimumY_in
     *               Region top bound in pixels.
     * @param[in]    maximumY_in
     *               Region bottom bound in pixels.
     * @param[in]    featureCount_in
     *               Keypoint budget of the region.
     * @param[in]    level_in
     *               Pyramid level under distribution.
     *
     * @param[out] keyPoints_out Distributed keypoints of the region.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus
        distributeOctTree(const std::vector<cv::KeyPoint> &keysToDistribute_in,
                          const int                       &minimumX_in,
                          const int                       &maximumX_in,
                          const int                       &minimumY_in,
                          const int                       &maximumY_in,
                          const int                       &featureCount_in,
                          const int                       &level_in,
                          std::vector<cv::KeyPoint>       &keyPoints_out);

    /*!
     * @brief        Detects keypoints on every pyramid level
     *               without the octree distribution.
     *
     * @param[in,out] keypointsPerLevel_inout
     *               Detected keypoints grouped by level.
     */
    [[nodiscard]] ORBextractorStatus computeKeyPointsOld(
        std::vector<std::vector<cv::KeyPoint>> &keypointsPerLevel_inout);
    /*!
     * @brief        Sampling pattern of the BRIEF descriptor.
     */
    std::vector<cv::Point> briefPattern;

    /*!
     * @brief        Target number of features.
     */
    int    featureCount;
    /*!
     * @brief        Scale step between pyramid levels.
     */
    double scaleFactor;
    /*!
     * @brief        Number of pyramid levels.
     */
    int    levelCount;
    /*!
     * @brief        FAST threshold used at extraction.
     */
    int    initialFastThreshold;
    /*!
     * @brief        FAST threshold used when extraction is
     *               retried.
     */
    int    minimumFastThreshold;

    /*!
     * @brief        Feature budget of every pyramid level.
     */
    std::vector<int> featuresPerLevel;

    /*!
     * @brief        Orientation search offsets.
     */
    std::vector<int> orientationMaxOffset;

    /*!
     * @brief        Scale factor of every pyramid level.
     */
    std::vector<float> scaleFactors;
    /*!
     * @brief        Inverse scale factor of every pyramid
     *               level.
     */
    std::vector<float> inverseScaleFactors;
    /*!
     * @brief        Sigma square of every pyramid level.
     */
    std::vector<float> levelSigmaSquares;
    /*!
     * @brief        Inverse sigma square of every pyramid
     *               level.
     */
    std::vector<float> inverseLevelSigmaSquares;

  public:
    // Adaptive FAST threshold: dynamically adjust thresholds when tracking
    // degrades
    /*!
     * @brief        Sets the FAST threshold used at extraction.
     *
     * @param[in]    threshold_in
     *               New extraction threshold.
     */
    [[nodiscard]] ORBextractorStatus setInitialFastThreshold(int threshold_in)
    {
        initialFastThreshold = threshold_in;

        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }
    /*!
     * @brief        Sets the FAST threshold used when extraction
     *               is retried.
     *
     * @param[in]    threshold_in
     *               New retry threshold.
     */
    [[nodiscard]] ORBextractorStatus setMinimumFastThreshold(int threshold_in)
    {
        minimumFastThreshold = threshold_in;

        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the FAST threshold used at
     *               extraction.
     *
     * @param[out] getInitialFastThreshold_out Current extraction threshold.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus
        getInitialFastThreshold(int &getInitialFastThreshold_out) const
    {
        getInitialFastThreshold_out = initialFastThreshold;
        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }
    /*!
     * @brief        Returns the FAST threshold used when
     *               extraction is retried.
     *
     * @param[out] getMinimumFastThreshold_out Current retry threshold.
     * @return ORBEXTRACTOR_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBextractorStatus
        getMinimumFastThreshold(int &getMinimumFastThreshold_out) const
    {
        getMinimumFastThreshold_out = minimumFastThreshold;
        return ORBextractorStatus::ORBEXTRACTOR_STATUS_SUCCESS;
    }
};

} // namespace core
} // namespace vs_graphs

#endif
