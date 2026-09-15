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

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "sophus/sim3.hpp"

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace vs_graphs
{
namespace core
{

    class ORBmatcher
    {
    public:
        ORBmatcher(float nnRatio_in = 0.6, bool checkOrientation_in = true);

        // Computes the Hamming distance between two ORB descriptors
        static int DescriptorDistance(const cv::Mat &descriptor1_in, const cv::Mat &descriptor2_in);

        // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
        // Used to track the local map (Tracking)
        int SearchByProjection(Frame &frame_inout, const std::vector<MapPoint *> &mapPoints_in, const float threshold_in = 3, const bool farPoints_in = false, const float farPointsThreshold_in = 50.0f);

        // Project MapPoints tracked in last frame into the current frame and search matches.
        // Used to track from previous frame (Tracking)
        int SearchByProjection(Frame &currentFrame_inout, const Frame &lastFrame_in, const float threshold_in, const bool mono_in);

        // Project MapPoints seen in KeyFrame into the Frame and search matches.
        // Used in relocalisation (Tracking)
        int SearchByProjection(Frame &currentFrame_inout, KeyFrame *p_keyframe_in, const std::set<MapPoint *> &alreadyFound_in, const float threshold_in, const int orbDistance_in);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in loop detection (Loop Closing)
        int SearchByProjection(KeyFrame *p_keyframe_in, Sophus::Sim3<float> &similarity_in, const std::vector<MapPoint *> &points_in, std::vector<MapPoint *> &matched_out, int threshold_in, float hammingRatio_in = 1.0);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in Place Recognition (Loop Closing and Merging)
        int SearchByProjection(KeyFrame *p_keyframe_in, Sophus::Sim3<float> &similarity_in, const std::vector<MapPoint *> &points_in, const std::vector<KeyFrame *> &pointsKeyframes_in, std::vector<MapPoint *> &matched_out, std::vector<KeyFrame *> &matchedKeyframes_out, int threshold_in, float hammingRatio_in = 1.0);

        // Depth-guided search for RGB-D: uses MapPoint tracked depth to constrain search radius
        // Helps in low-texture repetitive corridors where visual appearance is ambiguous
        int SearchByProjectionWithDepth(Frame &frame_inout, const std::vector<MapPoint *> &mapPoints_in, const float threshold_in, const bool farPoints_in, const float farPointsThreshold_in, const float depthThreshold_in);

        // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
        // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
        // Used in Relocalisation and Loop Detection
        int SearchByBoW(KeyFrame *p_keyframe_in, Frame &frame_inout, std::vector<MapPoint *> &mapPointMatches_out);
        int SearchByBoW(KeyFrame *p_keyframe1_in, KeyFrame *p_keyframe2_in, std::vector<MapPoint *> &matches12_out);

        // Matching for the Map Initialization (only used in the monocular case)
        int SearchForInitialization(Frame &frame1_inout, Frame &frame2_inout, std::vector<cv::Point2f> &prevMatched_out, std::vector<int> &matches12_out, int windowSize_in = 10);

        // Matching to triangulate new MapPoints. Check Epipolar Constraint.
        int SearchForTriangulation(KeyFrame *p_keyframe1_in, KeyFrame *p_keyframe2_in,
                                   std::vector<pair<size_t, size_t>> &matchedPairs_out, const bool stereoOnly_in, const bool coarse_in = false);

        // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
        // In the stereo and RGB-D case, s12=1
        // int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
        int SearchBySim3(KeyFrame *p_keyframe1_in, KeyFrame *p_keyframe2_in, std::vector<MapPoint *> &matches12_out, const Sophus::Sim3f &transform12_in, const float threshold_in);

        // Project MapPoints into KeyFrame and search for duplicated MapPoints.
        int Fuse(KeyFrame *p_keyframe_in, const vector<MapPoint *> &mapPoints_in, const float threshold_in = 3.0, const bool right_in = false);

        // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
        int Fuse(KeyFrame *p_keyframe_in, Sophus::Sim3f &similarity_in, const std::vector<MapPoint *> &points_in, float threshold_in, vector<MapPoint *> &replacePoints_out);

    public:
        static const int TH_LOW;
        static const int TH_HIGH;
        static const int HISTO_LENGTH;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        float RadiusByViewingCos(const float &viewCosine_in);

        void ComputeThreeMaxima(std::vector<int> *histogram_in, const int length_in, int &max1_out, int &max2_out, int &max3_out);

        float mfNNratio;
        bool mbCheckOrientation;
    };

} // namespace core
} // namespace vs_graphs

#endif // ORBMATCHER_H
