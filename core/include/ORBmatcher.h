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
 * @file         ORBmatcher.h
 *
 * @brief        Declares the ORB descriptor matcher.
 */

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include "ORBmatcherStatus.h"
#include "sophus/sim3.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <optional>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace vs_graphs
{
namespace core
{

/*!
 * @brief        Matches ORB descriptors across frames and
 *               keyframes for tracking and loop detection.
 */
class ORBmatcher
{
  public:
    /*!
     * @brief        Creates a matcher with the given ratio test
     *               and orientation policy.
     *
     * @param[in]    nnRatio_in
     *               Nearest-neighbour ratio test threshold.
     * @param[in]    checkOrientation_in
     *               True to enforce rotation-histogram
     *               consistency.
     */
    ORBmatcher(float nnRatio_in = 0.6, bool checkOrientation_in = true) :
        nearestNeighborRatio(nnRatio_in),
        shouldCheckOrientation(checkOrientation_in)
    {}

    /*!
     * @brief        Computes the Hamming distance between two
     *               ORB descriptors.
     *
     * @param[in]    descriptor1_in
     *               First descriptor row.
     * @param[in]    descriptor2_in
     *               Second descriptor row.
     *
     * @param[out] descriptorDistance_out Hamming distance between the
     * descriptors.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ORBmatcherStatus
        computeDescriptorDistance(const cv::Mat &descriptor1_in,
                                  const cv::Mat &descriptor2_in,
                                  int           &descriptorDistance_out);

    /*!
     * @brief        Matches frame keypoints against projected
     *               map points; used to track the local map.
     *
     * @param[in,out] frame_inout
     *                Frame receiving the matches.
     * @param[in]    mapPoints_in
     *               Non-owning candidate map points.
     * @param[in]    threshold_in
     *               Search radius in pixels.
     * @param[in]    farPoints_in
     *               True to widen the radius for far points.
     * @param[in]    farPointsThreshold_in
     *               Depth above which points count as far, in
     *               metres.
     * @param[in]    depthThreshold_in
     *               When set, depth-guided search: the window of a point
     *               with a tracked depth shrinks to 70 % below this depth
     *               and grows to 120 % at or above it, in metres. This
     *               disambiguates repetitive corridors.
     *
     * @param[out] byProjection_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus searchByProjection(
        Frame                         &frame_inout,
        const std::vector<MapPoint *> &mapPoints_in,
        int                           &byProjection_out,
        const float                    threshold_in          = 3,
        const bool                     farPoints_in          = false,
        const float                    farPointsThreshold_in = 50.0f,
        const std::optional<float>    &depthThreshold_in     = std::nullopt);

    /*!
     * @brief        Matches the current frame against map points
     *               tracked in the last frame.
     *
     * @param[in,out] currentFrame_inout
     *                Current frame receiving the matches.
     * @param[in]    lastFrame_in
     *               Previous frame holding the tracked points.
     * @param[in]    threshold_in
     *               Search radius in pixels.
     * @param[in]    mono_in
     *               True for the monocular search policy.
     *
     * @param[out] byProjection_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus searchByProjection(Frame &currentFrame_inout,
                                                      const Frame &lastFrame_in,
                                                      const float  threshold_in,
                                                      const bool   mono_in,
                                                      int &byProjection_out);

    /*!
     * @brief        Matches a keyframe against a frame for
     *               relocalisation.
     *
     * @param[in,out] currentFrame_inout
     *                Frame receiving the matches.
     * @param[in]    p_keyframe_in
     *               Non-owning keyframe holding the map points;
     *               shall be non-null.
     * @param[in]    alreadyFound_in
     *               Map points excluded from the search.
     * @param[in]    threshold_in
     *               Search radius in pixels.
     * @param[in]    orbDistance_in
     *               Maximum descriptor distance accepted.
     *
     * @param[out] byProjection_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        searchByProjection(Frame                      &currentFrame_inout,
                           KeyFrame                   *p_keyframe_in,
                           const std::set<MapPoint *> &alreadyFound_in,
                           const float                 threshold_in,
                           const int                   orbDistance_in,
                           int                        &byProjection_out);

    /*!
     * @brief        Matches map points under a similarity
     *               transform for loop detection.
     *
     * @param[in]    p_keyframe_in
     *               Non-owning keyframe receiving projections;
     *               shall be non-null.
     * @param[in]     similarity_in
     *                Similarity mapping points into the
     *                keyframe.
     * @param[in]    points_in
     *               Non-owning candidate map points.
     * @param[in,out] matched_inout
     *               Matched map point per candidate entry.
     * @param[in]    threshold_in
     *               Search radius in pixels.
     * @param[in]    hammingRatio_in
     *               Scale applied to the Hamming acceptance
     *               threshold.
     *
     * @param[out] byProjection_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        searchByProjection(KeyFrame                      *p_keyframe_in,
                           Sophus::Sim3<float>           &similarity_in,
                           const std::vector<MapPoint *> &points_in,
                           std::vector<MapPoint *>       &matched_inout,
                           int                            threshold_in,
                           int                           &byProjection_out,
                           float hammingRatio_in = 1.0);

    /*!
     * @brief        Matches map points under a similarity
     *               transform for place recognition.
     *
     * @param[in]    p_keyframe_in
     *               Non-owning keyframe receiving projections;
     *               shall be non-null.
     * @param[in]     similarity_in
     *                Similarity mapping points into the
     *                keyframe.
     * @param[in]    points_in
     *               Non-owning candidate map points.
     * @param[in]    pointsKeyframes_in
     *               Non-owning keyframe owning each candidate.
     * @param[in,out] matched_inout
     *               Matched map point per candidate entry.
     * @param[in,out] matchedKeyframes_inout
     *               Keyframe matched per candidate entry.
     * @param[in]    threshold_in
     *               Search radius in pixels.
     * @param[in]    hammingRatio_in
     *               Scale applied to the Hamming acceptance
     *               threshold.
     *
     * @param[out] byProjection_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        searchByProjection(KeyFrame                      *p_keyframe_in,
                           Sophus::Sim3<float>           &similarity_in,
                           const std::vector<MapPoint *> &points_in,
                           const std::vector<KeyFrame *> &pointsKeyframes_in,
                           std::vector<MapPoint *>       &matched_inout,
                           std::vector<KeyFrame *> &matchedKeyframes_inout,
                           int                      threshold_in,
                           int                     &byProjection_out,
                           float                    hammingRatio_in = 1.0);

    /*!
     * @brief        Matches keyframe map points against frame
     *               descriptors constrained by vocabulary nodes.
     *
     * @param[in]    p_keyframe_in
     *               Non-owning keyframe holding the map points;
     *               shall be non-null.
     * @param[in,out] frame_inout
     *                Frame receiving the matches.
     * @param[out]   mapPointMatches_out
     *               Matched map point per frame keypoint.
     *
     * @param[out] byBoW_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        searchByBoW(KeyFrame                *p_keyframe_in,
                    Frame                   &frame_inout,
                    std::vector<MapPoint *> &mapPointMatches_out,
                    int                     &byBoW_out);
    /*!
     * @brief        Matches map points between two keyframes
     *               constrained by vocabulary nodes.
     *
     * @param[in]    p_keyframe1_in
     *               Non-owning first keyframe; shall be
     *               non-null.
     * @param[in]    p_keyframe2_in
     *               Non-owning second keyframe; shall be
     *               non-null.
     * @param[out]   matches12_out
     *               Matched map point per first-keyframe point.
     *
     * @param[out] byBoW_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        searchByBoW(KeyFrame                *p_keyframe1_in,
                    KeyFrame                *p_keyframe2_in,
                    std::vector<MapPoint *> &matches12_out,
                    int                     &byBoW_out);

    /*!
     * @brief        Matches two frames for monocular map
     *               initialization.
     *
     * @param[in,out] frame1_inout
     *                First frame receiving the matches.
     * @param[in,out] frame2_inout
     *                Second frame receiving the matches.
     * @param[in,out] previousMatched_inout
     *               Matched locations in the first frame.
     * @param[out]   matches12_out
     *               Match index per first-frame keypoint.
     * @param[in]    windowSize_in
     *               Search window half-size in pixels.
     *
     * @param[out] forInitialization_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        searchForInitialization(Frame                    &frame1_inout,
                                Frame                    &frame2_inout,
                                std::vector<cv::Point2f> &previousMatched_inout,
                                std::vector<int>         &matches12_out,
                                int                      &forInitialization_out,
                                int                       windowSize_in = 10);

    /*!
     * @brief        Matches keypoints between keyframes and
     *               checks the epipolar constraint.
     *
     * @param[in]    p_keyframe1_in
     *               Non-owning first keyframe; shall be
     *               non-null.
     * @param[in]    p_keyframe2_in
     *               Non-owning second keyframe; shall be
     *               non-null.
     * @param[out]   matchedPairs_out
     *               Matched keypoint index pairs.
     * @param[in]    stereoOnly_in
     *               True to keep stereo pairs only.
     * @param[in]    coarse_in
     *               True to widen the search window.
     *
     * @param[out] forTriangulation_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus searchForTriangulation(
        KeyFrame                          *p_keyframe1_in,
        KeyFrame                          *p_keyframe2_in,
        std::vector<pair<size_t, size_t>> &matchedPairs_out,
        const bool                         stereoOnly_in,
        int                               &forTriangulation_out,
        const bool                         coarse_in = false);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a
    // Sim3 [s12*R12|t12] In the stereo and RGB-D case, s12=1 int
    // SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *>
    // &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12,
    // const float th);
    /*!
     * @brief        Matches map points between keyframes under
     *               a Sim3 transform.
     *
     * @param[in]    p_keyframe1_in
     *               Non-owning first keyframe; shall be
     *               non-null.
     * @param[in]    p_keyframe2_in
     *               Non-owning second keyframe; shall be
     *               non-null.
     * @param[in,out] matches12_inout
     *               Matched map point per first-keyframe point.
     * @param[in]    transform12_in
     *               Similarity from the first keyframe into the
     *               second.
     * @param[in]    threshold_in
     *               Search radius in pixels.
     *
     * @param[out] bySim3_out Number of matches found.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        searchBySim3(KeyFrame                *p_keyframe1_in,
                     KeyFrame                *p_keyframe2_in,
                     std::vector<MapPoint *> &matches12_inout,
                     const Sophus::Sim3f     &transform12_in,
                     const float              threshold_in,
                     int                     &bySim3_out);

    /*!
     * @brief        Fuses duplicated map points projected into
     *               a keyframe.
     *
     * @param[in,out] p_keyframe_inout
     *               Non-owning keyframe receiving projections;
     *               shall be non-null.
     * @param[in]    mapPoints_in
     *               Non-owning candidate map points.
     * @param[in]    threshold_in
     *               Search radius in pixels.
     * @param[in]    right_in
     *               True to project into the right stereo view.
     *
     * @param[out] fusedCount_out Number of fused points.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus fuse(KeyFrame *p_keyframe_inout,
                                        const vector<MapPoint *> &mapPoints_in,
                                        int        &fusedCount_out,
                                        const float threshold_in = 3.0,
                                        const bool  right_in     = false);

    /*!
     * @brief        Fuses duplicated map points projected under
     *               a Sim3 transform.
     *
     * @param[in,out] p_keyframe_inout
     *               Non-owning keyframe receiving projections;
     *               shall be non-null.
     * @param[in]     similarity_in
     *                Similarity mapping points into the
     *                keyframe.
     * @param[in]    points_in
     *               Non-owning candidate map points.
     * @param[in]    threshold_in
     *               Search radius in pixels.
     * @param[in,out] replacePoints_inout
     *               Replacement map point per fused candidate.
     *
     * @param[out] fusedCount_out Number of fused points.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        fuse(KeyFrame                      *p_keyframe_inout,
             Sophus::Sim3f                 &similarity_in,
             const std::vector<MapPoint *> &points_in,
             float                          threshold_in,
             std::vector<MapPoint *>       &replacePoints_inout,
             int                           &fusedCount_out);

  public:
    /*!
     * @brief        Low Hamming distance acceptance threshold.
     */
    static const int TH_LOW;
    /*!
     * @brief        High Hamming distance acceptance threshold.
     */
    static const int TH_HIGH;
    /*!
     * @brief        Orientation histogram length.
     */
    static const int HISTO_LENGTH;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    /*!
     * @brief        Returns the search radius for a viewing
     *               cosine.
     *
     * @param[in]    viewCosine_in
     *               Cosine between the viewing rays.
     *
     * @param[out] radius_out Search radius in pixels.
     * @return ORBMATCHER_STATUS_SUCCESS.
     */
    [[nodiscard]] ORBmatcherStatus
        radiusByViewingCos(const float &viewCosine_in, float &radius_out);

    /*!
     * @brief        Finds the three fullest orientation
     *               histogram bins.
     *
     * @param[in]    p_histogram_in
     *               Orientation histogram; shall be non-null.
     * @param[in]    length_in
     *               Number of histogram bins.
     * @param[in,out] maximum1_inout
     *               Index of the fullest bin.
     * @param[in,out] maximum2_inout
     *               Index of the second fullest bin.
     * @param[out]   maximum3_out
     *               Index of the third fullest bin.
     */
    [[nodiscard]] ORBmatcherStatus
        computeThreeMaxima(std::vector<int> *p_histogram_in,
                           const int         length_in,
                           int              &maximum1_inout,
                           int              &maximum2_inout,
                           int              &maximum3_out);

    /*!
     * @brief        Nearest-neighbour ratio test threshold.
     */
    float nearestNeighborRatio;
    /*!
     * @brief        True to enforce rotation-histogram
     *               consistency.
     */
    bool  shouldCheckOrientation;
};

} // namespace core
} // namespace vs_graphs

#endif // ORBMATCHER_H
