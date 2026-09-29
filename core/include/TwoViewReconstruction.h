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

#ifndef TwoViewReconstruction_H
#define TwoViewReconstruction_H

#include "TwoViewReconstructionStatus.h"
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <unordered_set>

#include <sophus/se3.hpp>

namespace vs_graphs
{
namespace core
{

class TwoViewReconstruction
{
    typedef std::pair<int, int> Match;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Fix the reference frame
    TwoViewReconstruction(const Eigen::Matrix3f &calibrationMatrix_in,
                          float                  sigma_in             = 1.0,
                          int                    maxIterationCount_in = 200)
    {
        calibrationMatrix = calibrationMatrix_in;

        this->sigma   = sigma_in;
        sigmaSquared  = sigma_in * sigma_in;
        maxIterations = maxIterationCount_in;
    }

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from
    // motion
    [[nodiscard]] TwoViewReconstructionStatus
        reconstruct(const std::vector<cv::KeyPoint> &keys1_in,
                    const std::vector<cv::KeyPoint> &keys2_in,
                    const std::vector<int>          &matches12_in,
                    Sophus::SE3f                    &T21_inout,
                    std::vector<cv::Point3f>        &vP3D_inout,
                    std::vector<bool>               &triangulatedFlags_inout,
                    bool                            &isReconstructed_out);

  private:
    [[nodiscard]] TwoViewReconstructionStatus
        findHomography(std::vector<bool> &matchesInliersFlags_out,
                       float             &score_inout,
                       Eigen::Matrix3f   &H21_out);
    [[nodiscard]] TwoViewReconstructionStatus
        findFundamental(std::vector<bool> &inliersFlags_inout,
                        float             &score_inout,
                        Eigen::Matrix3f   &F21_out);

    [[nodiscard]] TwoViewReconstructionStatus
        computeH21(const std::vector<cv::Point2f> &points1_in,
                   const std::vector<cv::Point2f> &points2_in,
                   Eigen::Matrix3f                &h21_out);
    [[nodiscard]] TwoViewReconstructionStatus
        computeF21(const std::vector<cv::Point2f> &points1_in,
                   const std::vector<cv::Point2f> &points2_in,
                   Eigen::Matrix3f                &f21_out);

    [[nodiscard]] TwoViewReconstructionStatus
        checkHomography(const Eigen::Matrix3f &H21_in,
                        const Eigen::Matrix3f &H12_in,
                        std::vector<bool>     &matchesInliersFlags_inout,
                        float                  sigma_in,
                        float                 &score_out);

    [[nodiscard]] TwoViewReconstructionStatus
        checkFundamental(const Eigen::Matrix3f &F21_in,
                         std::vector<bool>     &matchesInliersFlags_inout,
                         float                  sigma_in,
                         float                 &score_out);

    [[nodiscard]] TwoViewReconstructionStatus
        reconstructF(std::vector<bool>        &matchesInliersFlags_inout,
                     Eigen::Matrix3f          &F21_in,
                     Eigen::Matrix3f          &K_in,
                     Sophus::SE3f             &T21_out,
                     std::vector<cv::Point3f> &vP3D_out,
                     std::vector<bool>        &triangulatedFlags_out,
                     float                     minimumParallax_in,
                     int                       minimumTriangulated_in,
                     bool                     &isReconstructed_out);

    [[nodiscard]] TwoViewReconstructionStatus
        reconstructH(std::vector<bool>        &matchesInliersFlags_inout,
                     Eigen::Matrix3f          &H21_in,
                     Eigen::Matrix3f          &K_in,
                     Sophus::SE3f             &T21_out,
                     std::vector<cv::Point3f> &vP3D_inout,
                     std::vector<bool>        &triangulatedFlags_out,
                     float                     minimumParallax_in,
                     int                       minimumTriangulated_in,
                     bool                     &isReconstructed_out);

    [[nodiscard]] TwoViewReconstructionStatus
        normalize(const std::vector<cv::KeyPoint> &keys_in,
                  std::vector<cv::Point2f>        &normalizedPoints_inout,
                  Eigen::Matrix3f                 &T_out);

    [[nodiscard]] TwoViewReconstructionStatus
        checkRT(const Eigen::Matrix3f           &R_in,
                const Eigen::Vector3f           &t_in,
                const std::vector<cv::KeyPoint> &keys1_in,
                const std::vector<cv::KeyPoint> &keys2_in,
                const std::vector<Match>        &matches12_in,
                std::vector<bool>               &matchesInliersFlags_in,
                const Eigen::Matrix3f           &K_in,
                std::vector<cv::Point3f>        &vP3D_inout,
                float                            threshold2_in,
                std::vector<bool>               &goodFlags_out,
                float                           &parallax_out,
                int                             &goodPointCount_out);

    [[nodiscard]] TwoViewReconstructionStatus
        decomposeE(const Eigen::Matrix3f &E_in,
                   Eigen::Matrix3f       &R1_out,
                   Eigen::Matrix3f       &R2_out,
                   Eigen::Vector3f       &t_out);

    // Keypoints from Reference Frame (Frame 1)
    std::vector<cv::KeyPoint> keys1;

    // Keypoints from Current Frame (Frame 2)
    std::vector<cv::KeyPoint> keys2;

    // Current Matches from Reference to Current
    std::vector<Match> matches12;
    std::vector<bool>  matchedFlags1;

    // Calibration
    Eigen::Matrix3f calibrationMatrix;

    // Standard Deviation and Variance
    float sigma, sigmaSquared;

    // Ransac max iterations
    int maxIterations;

    // Ransac sets
    std::vector<std::vector<size_t>> sets;
};

} // namespace core
} // namespace vs_graphs

#endif // TwoViewReconstruction_H
