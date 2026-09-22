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

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
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
    TwoViewReconstruction(const Eigen::Matrix3f &k,
                          float                  sigma      = 1.0,
                          int                    iterations = 200)
    {
        calibrationMatrix = k;

        this->sigma   = sigma;
        sigmaSquared  = sigma * sigma;
        maxIterations = iterations;
    }

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from
    // motion
    bool Reconstruct(const std::vector<cv::KeyPoint> &vKeys1,
                     const std::vector<cv::KeyPoint> &vKeys2,
                     const std::vector<int>          &vMatches12,
                     Sophus::SE3f                    &T21,
                     std::vector<cv::Point3f>        &vP3D,
                     std::vector<bool>               &vbTriangulated);

  private:
    void findHomography(std::vector<bool> &vbMatchesInliers,
                        float             &score,
                        Eigen::Matrix3f   &H21);
    void findFundamental(std::vector<bool> &vbInliers,
                         float             &score,
                         Eigen::Matrix3f   &F21);

    Eigen::Matrix3f computeH21(const std::vector<cv::Point2f> &vP1,
                               const std::vector<cv::Point2f> &vP2);
    Eigen::Matrix3f computeF21(const std::vector<cv::Point2f> &vP1,
                               const std::vector<cv::Point2f> &vP2);

    float checkHomography(const Eigen::Matrix3f &H21,
                          const Eigen::Matrix3f &H12,
                          std::vector<bool>     &vbMatchesInliers,
                          float                  sigma);

    float checkFundamental(const Eigen::Matrix3f &F21,
                           std::vector<bool>     &vbMatchesInliers,
                           float                  sigma);

    bool reconstructF(std::vector<bool>        &vbMatchesInliers,
                      Eigen::Matrix3f          &F21,
                      Eigen::Matrix3f          &K,
                      Sophus::SE3f             &T21,
                      std::vector<cv::Point3f> &vP3D,
                      std::vector<bool>        &vbTriangulated,
                      float                     minParallax,
                      int                       minTriangulated);

    bool reconstructH(std::vector<bool>        &vbMatchesInliers,
                      Eigen::Matrix3f          &H21,
                      Eigen::Matrix3f          &K,
                      Sophus::SE3f             &T21,
                      std::vector<cv::Point3f> &vP3D,
                      std::vector<bool>        &vbTriangulated,
                      float                     minParallax,
                      int                       minTriangulated);

    void normalize(const std::vector<cv::KeyPoint> &vKeys,
                   std::vector<cv::Point2f>        &vNormalizedPoints,
                   Eigen::Matrix3f                 &T);

    int checkRT(const Eigen::Matrix3f           &R,
                const Eigen::Vector3f           &t,
                const std::vector<cv::KeyPoint> &vKeys1,
                const std::vector<cv::KeyPoint> &vKeys2,
                const std::vector<Match>        &vMatches12,
                std::vector<bool>               &vbMatchesInliers,
                const Eigen::Matrix3f           &K,
                std::vector<cv::Point3f>        &vP3D,
                float                            th2,
                std::vector<bool>               &vbGood,
                float                           &parallax);

    void decomposeE(const Eigen::Matrix3f &E,
                    Eigen::Matrix3f       &R1,
                    Eigen::Matrix3f       &R2,
                    Eigen::Vector3f       &t);

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
