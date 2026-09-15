/**
 * This file is a modified version of a file from ORB-SLAM3.
 * 
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 * 
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 * 
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef CONVERTER_H
#define CONVERTER_H

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"

namespace vs_graphs
{
namespace core
{

    class Converter
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &descriptors_in);

        static g2o::SE3Quat toSE3Quat(const cv::Mat &transform_in);
        static g2o::SE3Quat toSE3Quat(const Sophus::SE3f &transform_in);
        static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &similarity_in);

        // TODO templetize these functions
        static cv::Mat toCvMat(const g2o::SE3Quat &rigidTransform_in);
        static cv::Mat toCvMat(const g2o::Sim3 &similarity_in);
        static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &matrix_in);
        static cv::Mat toCvMat(const Eigen::Matrix<float, 4, 4> &matrix_in);
        static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 4> &matrix_in);
        static cv::Mat toCvMat(const Eigen::Matrix3d &matrix_in);
        static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &matrix_in);
        static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 1> &matrix_in);
        static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 3> &matrix_in);

        static cv::Mat toCvMat(const Eigen::MatrixXf &matrix_in);
        static cv::Mat toCvMat(const Eigen::MatrixXd &matrix_in);

        static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &rotation_in, const Eigen::Matrix<double, 3, 1> &translation_in);
        static cv::Mat toCvSkewMatrix(const cv::Mat &vector_in);

        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &vector_in);
        static Eigen::Matrix<float, 3, 1> toVector3f(const cv::Mat &vector_in);
        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &point_in);
        static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &matrix_in);
        static Eigen::Matrix<double, 4, 4> toMatrix4d(const cv::Mat &matrix_in);
        static Eigen::Matrix<float, 3, 3> toMatrix3f(const cv::Mat &matrix_in);
        static Eigen::Matrix<float, 4, 4> toMatrix4f(const cv::Mat &matrix_in);
        static std::vector<float> toQuaternion(const cv::Mat &rotationMatrix_in);

        static bool isRotationMatrix(const cv::Mat &rotationMatrix_in);
        static std::vector<float> toEuler(const cv::Mat &rotationMatrix_in);

        // TODO: Sophus migration, to be deleted in the future
        static Sophus::SE3<float> toSophus(const cv::Mat &transform_in);
        static Sophus::Sim3f toSophus(const g2o::Sim3 &similarity_in);
    };

} // namespace core
} // namespace vs_graphs

#endif // CONVERTER_H
