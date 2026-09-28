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

/*!****************************************************************************
 * Author:   Steffen Urban                                              *
 * Contact:  urbste@gmail.com                                          *
 * License:  Copyright (c) 2016 Steffen Urban, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

#ifndef VS_GRAPHS_CORE_MLPNPSOLVER_H
#define VS_GRAPHS_CORE_MLPNPSOLVER_H

#include "Frame.h"
#include "MapPoint.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace vs_graphs
{
namespace core
{
class MLPnPsolver
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MLPnPsolver(const Frame              &frame_in,
                const vector<MapPoint *> &mapPointMatches_in) :
        inlierCount(0),
        iterationCount(0),
        bestInlierCount(0),
        correspondenceCount(0),
        p_camera(frame_in.p_camera)
    {
        mapPointMatches = mapPointMatches_in;
        bearingVectors.reserve(frame_in.mapPoints.size());
        points2D.reserve(frame_in.mapPoints.size());
        sigmaSquared.reserve(frame_in.mapPoints.size());
        points3Dw.reserve(frame_in.mapPoints.size());
        keypointIndices.reserve(frame_in.mapPoints.size());
        allIndices.reserve(frame_in.mapPoints.size());

        int outputIndex = 0;
        for (size_t matchIndex = 0, matchCount = mapPointMatches.size();
             matchIndex < matchCount;
             matchIndex++)
        {
            MapPoint *p_mapPoint = mapPointMatches_in[matchIndex];

            if (p_mapPoint)
            {
                if (!p_mapPoint->isBad())
                {
                    if (matchIndex >= frame_in.keyPointsUndistorted.size())
                        continue;
                    const cv::KeyPoint &keyPoint =
                        frame_in.keyPointsUndistorted[matchIndex];

                    points2D.push_back(keyPoint.pt);
                    sigmaSquared.push_back(
                        frame_in.levelSigmaSquared[keyPoint.octave]);

                    // Bearing vector should be normalized
                    cv::Point3f bearingVectorCv =
                        p_camera->unproject(keyPoint.pt);
                    bearingVectorCv /= bearingVectorCv.z;
                    BearingVector bearingVector(bearingVectorCv.x,
                                                bearingVectorCv.y,
                                                bearingVectorCv.z);
                    bearingVectors.push_back(bearingVector);

                    // 3D coordinates
                    Eigen::Matrix<float, 3, 1> worldPositionEigen =
                        p_mapPoint->getWorldPos();
                    Point3 worldPosition(worldPositionEigen(0),
                                         worldPositionEigen(1),
                                         worldPositionEigen(2));
                    points3Dw.push_back(worldPosition);

                    keypointIndices.push_back(matchIndex);
                    allIndices.push_back(outputIndex);

                    outputIndex++;
                }
            }
        }

        setRansacParameters();
    }

    ~MLPnPsolver();

    void setRansacParameters(double probability_in       = 0.99,
                             int    minimumInliers_in    = 8,
                             int    maximumIterations_in = 300,
                             int    minimumSet_in        = 6,
                             float  epsilon_in           = 0.4,
                             float  threshold2_in        = 5.991);

    // Find metod is necessary?

    bool iterate(int              iterationCount_in,
                 bool            &areIterationsExhausted_out,
                 vector<bool>    &inliersFlags_out,
                 int             &inlierCount_out,
                 Eigen::Matrix4f &Tout_out);

    // Type definitions needed by the original code

    /*! A 3-vector of unit length used to describe landmark
     * observations/bearings in camera frames (always expressed in camera
     * frames)
     */
    typedef Eigen::Vector3d BearingVector;

    /*! An array of bearing-vectors */
    typedef std::vector<BearingVector, Eigen::aligned_allocator<BearingVector>>
        BearingVectors;

    /*! A 2-matrix containing the 2D covariance information of a bearing vector
     */
    typedef Eigen::Matrix2d Covariance2Matrix;

    /*! A 3-matrix containing the 3D covariance information of a bearing vector
     */
    typedef Eigen::Matrix3d Covariance3Matrix;

    /*! An array of 3D covariance matrices */
    typedef std::vector<Covariance3Matrix,
                        Eigen::aligned_allocator<Covariance3Matrix>>
        Covariance3Matrices;

    /*! A 3-vector describing a point in 3D-space */
    typedef Eigen::Vector3d Point3;

    /*! An array of 3D-points */
    typedef std::vector<Point3, Eigen::aligned_allocator<Point3>> Points3;

    /*! A homogeneous 3-vector describing a point in 3D-space */
    typedef Eigen::Vector4d Point4;

    /*! An array of homogeneous 3D-points */
    typedef std::vector<Point4, Eigen::aligned_allocator<Point4>> Points4;

    /*! A 3-vector containing the rodrigues parameters of a rotation matrix */
    typedef Eigen::Vector3d RodriguesVector;

    /*! A rotation matrix */
    typedef Eigen::Matrix3d RotationMatrix;

    /*! A 3x4 transformation matrix containing rotation \f$ \mathbf{R} \f$ and
     *  translation \f$ \mathbf{t} \f$ as follows:
     *  \f$ \left( \begin{array}{cc} \mathbf{R} & \mathbf{t} \end{array} \right)
     * \f$
     */
    typedef Eigen::Matrix<double, 3, 4> TransformationMatrix;

    /*! A 3-vector describing a translation/camera position */
    typedef Eigen::Vector3d TranslationVector;

  private:
    void checkInliers();
    bool refine();

    // Functions from de original MLPnP code

    /*
     * Computes the camera pose given 3D points coordinates (in the camera
     * reference system), the camera rays and (optionally) the covariance matrix
     * of those camera rays. Result is stored in solution
     */
    void computePose(const BearingVectors      &f_in,
                     const Points3             &p_in,
                     const Covariance3Matrices &covMats_in,
                     const std::vector<int>    &indices_in,
                     TransformationMatrix      &result_inout);

    void mlpnp_gn(Eigen::VectorXd                    &x_inout,
                  const Points3                      &points_in,
                  const std::vector<Eigen::MatrixXd> &nullspaces_in,
                  const Eigen::SparseMatrix<double>   Kll_in,
                  bool                                shouldUseCovariance_in);

    void mlpnp_residuals_and_jacs(
        const Eigen::VectorXd              &x_in,
        const Points3                      &points_in,
        const std::vector<Eigen::MatrixXd> &nullspaces_in,
        Eigen::VectorXd                    &r_inout,
        Eigen::MatrixXd                    &fjac_in,
        bool                                getJacs_in);

    void mlpnpJacs(const Point3            &point_in,
                   const Eigen::Vector3d   &nullspace_r,
                   const Eigen::Vector3d   &nullspace_s_in,
                   const RodriguesVector   &w_in,
                   const TranslationVector &t_in,
                   Eigen::MatrixXd         &jacs_in);

    // Auxiliar methods

    /*!
     * \brief Compute a rotation matrix from Rodrigues axis angle.
     *
     * \param[in] omega The Rodrigues-parameters of a rotation.
     * \return The 3x3 rotation matrix.
     */
    Eigen::Matrix3d rodrigues2rot(const Eigen::Vector3d &omega_in);

    /*!
     * \brief Compute the Rodrigues-parameters of a rotation matrix.
     *
     * \param[in] R The 3x3 rotation matrix.
     * \return The Rodrigues-parameters.
     */
    Eigen::Vector3d rot2rodrigues(const Eigen::Matrix3d &R_in);

    //----------------------------------------------------
    // Fields of the solver
    //----------------------------------------------------
    vector<MapPoint *> mapPointMatches;

    // 2D Points
    vector<cv::Point2f> points2D;
    // Substitued by bearing vectors
    BearingVectors      bearingVectors;

    vector<float> sigmaSquared;

    // 3D Points
    // vector<cv::Point3f> mvP3Dw;
    Points3 points3Dw;

    // Index in Frame
    vector<size_t> keypointIndices;

    // Current Estimation
    double          mRi[3][3];
    double          mti[3];
    Eigen::Matrix4f mTcwi;
    vector<bool>    inlierFlags;
    int             inlierCount;

    // Current Ransac State
    int             iterationCount;
    vector<bool>    bestInlierFlags;
    int             bestInlierCount;
    Eigen::Matrix4f mBestTcw;

    // Refined
    Eigen::Matrix4f mRefinedTcw;
    vector<bool>    refinedInlierFlags;
    int             refinedInlierCount;

    // Number of Correspondences
    int correspondenceCount;

    // Indices for random selection [0 .. N-1]
    vector<size_t> allIndices;

    // RANSAC probability
    double ransacProb;

    // RANSAC min inliers
    int ransacMinInliers;

    // RANSAC max iterations
    int ransacMaxIterations;

    // RANSAC expected inliers/total ratio
    float ransacEpsilon;

    // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
    float ransacThreshold;

    // RANSAC Minimun Set used at each iteration
    int ransacMinSet;

    // Max square error associated with scale level. Max error =
    // th*th*sigma(level)*sigma(level)
    vector<float> maxError;

    camera_models::geometriccamera::GeometricCamera *p_camera;
};

} // namespace core
} // namespace vs_graphs
#endif // VS_GRAPHS_CORE_MLPNPSOLVER_H
