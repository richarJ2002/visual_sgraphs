/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file         GeometricTools.h
 *
 * @brief        Declares static two-view geometry helpers.
 */

#ifndef GEOMETRIC_TOOLS_H
#define GEOMETRIC_TOOLS_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

namespace vs_graphs
{
namespace core
{

class KeyFrame;

/*!
 * @brief        Static two-view geometry helpers shared by the
 *               estimator.
 */
class GeometricTools
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief        Computes the fundamental matrix between two
     *               keyframe views.
     *
     * @param[in]    pKF1
     *               Non-owning first keyframe; shall be non-null.
     * @param[in]    pKF2
     *               Non-owning second keyframe; shall be
     *               non-null.
     *
     * @return       Fundamental matrix mapping the second view
     *               into the first.
     */
    static Eigen::Matrix3f computeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

    /*!
     * @brief        Triangulates two normalized observations
     *               with a linear solve.
     *
     * @param[in]    x_c1
     *               Normalized observation in the first view.
     * @param[in]    x_c2
     *               Normalized observation in the second view.
     * @param[in]    Tc1w
     *               Three-by-four projection matrix of the
     *               first view.
     * @param[in]    Tc2w
     *               Three-by-four projection matrix of the
     *               second view.
     * @param[out]   x3D
     *               Triangulated point.
     *
     * @return       True and x3D set when the homogeneous scale
     *               is non-zero.
     */
    static bool triangulate(Eigen::Vector3f            &x_c1,
                            Eigen::Vector3f            &x_c2,
                            Eigen::Matrix<float, 3, 4> &Tc1w,
                            Eigen::Matrix<float, 3, 4> &Tc2w,
                            Eigen::Vector3f            &x3D);

    /*!
     * @brief        Checks element-wise agreement between a cv
     *               matrix and an Eigen matrix.
     *
     *              Mismatches are reported to standard output.
     *
     * @param[in]    cvMat
     *               OpenCV matrix to compare.
     * @param[in]    eigMat
     *               Eigen matrix to compare.
     *
     * @return       True when sizes match and every coefficient
     *               agrees within 1e-3.
     */
    template <int rows, int cols>
    static bool checkMatrices(const cv::Mat                          &cvMat,
                              const Eigen::Matrix<float, rows, cols> &eigMat)
    {
        const float epsilon = 1e-3;
        // std::cout << cvMat.cols - cols << cvMat.rows - rows << std::endl;
        if (rows != cvMat.rows || cols != cvMat.cols)
        {
            std::cout << "wrong cvmat size\n";
            return false;
        }
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                if ((cvMat.at<float>(i, j) > (eigMat(i, j) + epsilon)) ||
                    (cvMat.at<float>(i, j) < (eigMat(i, j) - epsilon)))
                {
                    std::cout << "cv mat:\n" << cvMat << std::endl;
                    std::cout << "eig mat:\n" << eigMat << std::endl;
                    return false;
                }
        return true;
    }

    /*!
     * @brief        Checks element-wise agreement between two
     *               Eigen matrices.
     *
     *              Mismatches are reported to standard output.
     *
     * @param[in]    eigMat1
     *               First matrix to compare.
     * @param[in]    eigMat2
     *               Second matrix to compare.
     *
     * @return       True when every coefficient agrees within
     *               1e-3.
     */
    template <typename T, int rows, int cols>
    static bool checkMatrices(const Eigen::Matrix<T, rows, cols> &eigMat1,
                              const Eigen::Matrix<T, rows, cols> &eigMat2)
    {
        const float epsilon = 1e-3;
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                if ((eigMat1(i, j) > (eigMat2(i, j) + epsilon)) ||
                    (eigMat1(i, j) < (eigMat2(i, j) - epsilon)))
                {
                    std::cout << "eig mat 1:\n" << eigMat1 << std::endl;
                    std::cout << "eig mat 2:\n" << eigMat2 << std::endl;
                    return false;
                }
        return true;
    }
};

} // namespace core
} // namespace vs_graphs

#endif // GEOMETRIC_TOOLS_H
