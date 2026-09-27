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
     * @param[out]   node1_out
     *               Top-left child.
     * @param[out]   node2_out
     *               Top-right child.
     * @param[out]   node3_out
     *               Bottom-left child.
     * @param[out]   node4_out
     *               Bottom-right child.
     */
    void divideNode(ExtractorNode &node1_out,
                    ExtractorNode &node2_out,
                    ExtractorNode &node3_out,
                    ExtractorNode &node4_out);

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
     * @param[out]   keypoints_out
     *               Extracted keypoints in image coordinates.
     * @param[out]   descriptors_out
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
                   std::vector<cv::KeyPoint> &keypoints_out,
                   cv::OutputArray            descriptors_out,
                   std::vector<int>          &lappingArea_in);

    /*!
     * @brief        Returns the number of pyramid levels.
     *
     * @return       Configured level count.
     */
    int inline getLevelCount()
    {
        return levelCount;
    }

    /*!
     * @brief        Returns the pyramid scale step.
     *
     * @return       Configured scale factor.
     */
    float inline getScaleFactor()
    {
        return scaleFactor;
    }

    /*!
     * @brief        Returns the per-level scale factors.
     *
     * @return       Scale factor of every pyramid level.
     */
    std::vector<float> inline getScaleFactors()
    {
        return scaleFactors;
    }

    /*!
     * @brief        Returns the per-level inverse scale factors.
     *
     * @return       Inverse scale factor of every pyramid
     *               level.
     */
    std::vector<float> inline getInverseScaleFactors()
    {
        return inverseScaleFactors;
    }

    /*!
     * @brief        Returns the per-level scale sigma squares.
     *
     * @return       Sigma square of every pyramid level.
     */
    std::vector<float> inline getScaleSigmaSquares()
    {
        return levelSigmaSquares;
    }

    /*!
     * @brief        Returns the per-level inverse sigma squares.
     *
     * @return       Inverse sigma square of every pyramid
     *               level.
     */
    std::vector<float> inline getInverseScaleSigmaSquares()
    {
        return inverseLevelSigmaSquares;
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
    void computePyramid(cv::Mat image_in);
    /*!
     * @brief        Detects keypoints on every pyramid level
     *               with the octree distribution.
     *
     * @param[out]   keypointsPerLevel_out
     *               Detected keypoints grouped by level.
     */
    void computeKeyPointsOctTree(
        std::vector<std::vector<cv::KeyPoint>> &keypointsPerLevel_out);
    /*!
     * @brief        Distributes keypoints inside a region with
     *               the octree.
     *
     * @param[in]    keysToDistribute_in
     *               Candidate keypoints of the region.
     * @param[in]    minX_in
     *               Region left bound in pixels.
     * @param[in]    maxX_in
     *               Region right bound in pixels.
     * @param[in]    minY_in
     *               Region top bound in pixels.
     * @param[in]    maxY_in
     *               Region bottom bound in pixels.
     * @param[in]    featureCount_in
     *               Keypoint budget of the region.
     * @param[in]    level_in
     *               Pyramid level under distribution.
     *
     * @return       Distributed keypoints of the region.
     */
    std::vector<cv::KeyPoint>
        distributeOctTree(const std::vector<cv::KeyPoint> &keysToDistribute_in,
                          const int                       &minX_in,
                          const int                       &maxX_in,
                          const int                       &minY_in,
                          const int                       &maxY_in,
                          const int                       &featureCount_in,
                          const int                       &level_in);

    /*!
     * @brief        Detects keypoints on every pyramid level
     *               without the octree distribution.
     *
     * @param[out]   keypointsPerLevel_out
     *               Detected keypoints grouped by level.
     */
    void computeKeyPointsOld(
        std::vector<std::vector<cv::KeyPoint>> &keypointsPerLevel_out);
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
    void setInitialFastThreshold(int threshold_in)
    {
        initialFastThreshold = threshold_in;
    }
    /*!
     * @brief        Sets the FAST threshold used when extraction
     *               is retried.
     *
     * @param[in]    threshold_in
     *               New retry threshold.
     */
    void setMinimumFastThreshold(int threshold_in)
    {
        minimumFastThreshold = threshold_in;
    }
    /*!
     * @brief        Returns the FAST threshold used at
     *               extraction.
     *
     * @return       Current extraction threshold.
     */
    int getInitialFastThreshold() const
    {
        return initialFastThreshold;
    }
    /*!
     * @brief        Returns the FAST threshold used when
     *               extraction is retried.
     *
     * @return       Current retry threshold.
     */
    int getMinimumFastThreshold() const
    {
        return minimumFastThreshold;
    }
};

} // namespace core
} // namespace vs_graphs

#endif
