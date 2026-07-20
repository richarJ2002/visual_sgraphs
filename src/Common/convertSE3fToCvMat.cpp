/*!
 * @File:         convertSE3fToCvMat.cpp
 *
 * @Brief:        Converts a Sophus SE(3) transformation into a 4-by-4 OpenCV
 *                homogeneous transformation matrix.
 *
 * @Date:         20/07/2026
 *
 */

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

/* Function Includes */
#include "Common.hpp"

/* Object Includes */
#include <sophus/se3.hpp>

/* Data Includes */
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

/* Generic Libraries */
/* None */

/*!
 * @brief       Converts a Sophus SE(3) transformation into a 4-by-4 OpenCV
 *              homogeneous transformation matrix.
 *
 *              The returned matrix uses single-precision floating-point values
 *              and contains the rotation and translation components in the
 *              following form:
 *
 *                  [ R  t ]
 *                  [ 0  1 ]
 *
 * @param[in]   transformation_SE3f_in
 *              Sophus SE(3) transformation to convert.
 *
 * @return      A 4-by-4 OpenCV matrix of type CV_32FC1 containing the
 *              homogeneous transformation matrix.
 */
cv::Mat convertSE3fToCvMat(const Sophus::SE3f &transformation_SE3f_in)
{
    /* Extract the homogeneous Eigen transformation matrix */
    const Eigen::Matrix4f transformationMatrix_Eigen =
        transformation_SE3f_in.matrix();

    /* Convert the Eigen matrix into an OpenCV matrix */
    cv::Mat transformationMatrix_OpenCV;

    cv::eigen2cv(transformationMatrix_Eigen, transformationMatrix_OpenCV);

    return transformationMatrix_OpenCV;
}
