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

/*!
 * @file         Converter.cc
 *
 * @brief        Implements pose and matrix conversions between library types.
 */

#include "Converter.h"

namespace vs_graphs
{
namespace core
{

/* Each row header shares its storage with the source matrix. */
std::vector<cv::Mat>
    Converter::toDescriptorVector(const cv::Mat &descriptors_in)
{
    std::vector<cv::Mat> descriptorVector;
    descriptorVector.reserve(descriptors_in.rows);
    for (int rowIndex = 0; rowIndex < descriptors_in.rows; rowIndex++)
        descriptorVector.push_back(descriptors_in.row(rowIndex));

    return descriptorVector;
    }

    g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &transform_in)
    {
        Eigen::Matrix<double, 3, 3> rotationMatrix;
        rotationMatrix << transform_in.at<float>(0, 0),
            transform_in.at<float>(0, 1), transform_in.at<float>(0, 2),
            transform_in.at<float>(1, 0), transform_in.at<float>(1, 1),
            transform_in.at<float>(1, 2), transform_in.at<float>(2, 0),
            transform_in.at<float>(2, 1), transform_in.at<float>(2, 2);

        Eigen::Matrix<double, 3, 1> translationVector(
            transform_in.at<float>(0, 3),
            transform_in.at<float>(1, 3),
            transform_in.at<float>(2, 3));

        return g2o::SE3Quat(rotationMatrix, translationVector);
    }

    g2o::SE3Quat Converter::toSE3Quat(const Sophus::SE3f &transform_in)
    {
        return g2o::SE3Quat(transform_in.unit_quaternion().cast<double>(),
                            transform_in.translation().cast<double>());
    }

    cv::Mat Converter::toCvMat(const g2o::SE3Quat &rigidTransform_in)
    {
        Eigen::Matrix<double, 4, 4> eigenMatrix =
            rigidTransform_in.to_homogeneous_matrix();
        return toCvMat(eigenMatrix);
    }

    cv::Mat Converter::toCvMat(const g2o::Sim3 &similarity_in)
    {
        Eigen::Matrix3d eigenRotation =
            similarity_in.rotation().toRotationMatrix();
        Eigen::Vector3d eigenTranslation = similarity_in.translation();
        double          scale            = similarity_in.scale();
        /* Fold the similarity scale into the rotation part. */
        return toCvSE3(scale * eigenRotation, eigenTranslation);
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &matrix_in)
    {
        cv::Mat cvMat(4, 4, CV_32F);
        for (int rowIndex = 0; rowIndex < 4; rowIndex++)
            for (int columnIndex = 0; columnIndex < 4; columnIndex++)
                cvMat.at<float>(rowIndex, columnIndex) =
                    matrix_in(rowIndex, columnIndex);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 4, 4> &matrix_in)
    {
        cv::Mat cvMat(4, 4, CV_32F);
        for (int rowIndex = 0; rowIndex < 4; rowIndex++)
            for (int columnIndex = 0; columnIndex < 4; columnIndex++)
                cvMat.at<float>(rowIndex, columnIndex) =
                    matrix_in(rowIndex, columnIndex);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 3, 4> &matrix_in)
    {
        cv::Mat cvMat(3, 4, CV_32F);
        for (int rowIndex = 0; rowIndex < 3; rowIndex++)
            for (int columnIndex = 0; columnIndex < 4; columnIndex++)
                cvMat.at<float>(rowIndex, columnIndex) =
                    matrix_in(rowIndex, columnIndex);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix3d &matrix_in)
    {
        cv::Mat cvMat(3, 3, CV_32F);
        for (int rowIndex = 0; rowIndex < 3; rowIndex++)
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
                cvMat.at<float>(rowIndex, columnIndex) =
                    matrix_in(rowIndex, columnIndex);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix3f &matrix_in)
    {
        cv::Mat cvMat(3, 3, CV_32F);
        for (int rowIndex = 0; rowIndex < 3; rowIndex++)
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
                cvMat.at<float>(rowIndex, columnIndex) =
                    matrix_in(rowIndex, columnIndex);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::MatrixXf &matrix_in)
    {
        cv::Mat cvMat(matrix_in.rows(), matrix_in.cols(), CV_32F);
        for (int rowIndex = 0; rowIndex < matrix_in.rows(); rowIndex++)
            for (int columnIndex = 0; columnIndex < matrix_in.cols();
                 columnIndex++)
                cvMat.at<float>(rowIndex, columnIndex) =
                    matrix_in(rowIndex, columnIndex);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::MatrixXd &matrix_in)
    {
        cv::Mat cvMat(matrix_in.rows(), matrix_in.cols(), CV_32F);
        for (int rowIndex = 0; rowIndex < matrix_in.rows(); rowIndex++)
            for (int columnIndex = 0; columnIndex < matrix_in.cols();
                 columnIndex++)
                cvMat.at<float>(rowIndex, columnIndex) =
                    matrix_in(rowIndex, columnIndex);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &matrix_in)
    {
        cv::Mat cvMat(3, 1, CV_32F);
        for (int rowIndex = 0; rowIndex < 3; rowIndex++)
            cvMat.at<float>(rowIndex) = matrix_in(rowIndex);

        return cvMat.clone();
    }

    cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 3, 1> &matrix_in)
    {
        cv::Mat cvMat(3, 1, CV_32F);
        for (int rowIndex = 0; rowIndex < 3; rowIndex++)
            cvMat.at<float>(rowIndex) = matrix_in(rowIndex);

        return cvMat.clone();
    }

    cv::Mat
        Converter::toCvSE3(const Eigen::Matrix<double, 3, 3> &rotation_in,
                           const Eigen::Matrix<double, 3, 1> &translation_in)
    {
        cv::Mat cvMat = cv::Mat::eye(4, 4, CV_32F);
        for (int rowIndex = 0; rowIndex < 3; rowIndex++)
        {
            for (int columnIndex = 0; columnIndex < 3; columnIndex++)
            {
                cvMat.at<float>(rowIndex, columnIndex) =
                    rotation_in(rowIndex, columnIndex);
            }
        }
        for (int rowIndex = 0; rowIndex < 3; rowIndex++)
        {
            cvMat.at<float>(rowIndex, 3) = translation_in(rowIndex);
        }

        return cvMat.clone();
    }

    Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Mat &vector_in)
    {
        Eigen::Matrix<double, 3, 1> eigenVector;
        eigenVector << vector_in.at<float>(0), vector_in.at<float>(1),
            vector_in.at<float>(2);

        return eigenVector;
    }

    Eigen::Matrix<float, 3, 1> Converter::toVector3f(const cv::Mat &vector_in)
    {
        Eigen::Matrix<float, 3, 1> eigenVector;
        eigenVector << vector_in.at<float>(0), vector_in.at<float>(1),
            vector_in.at<float>(2);

        return eigenVector;
    }

    Eigen::Matrix<double, 3, 1>
        Converter::toVector3d(const cv::Point3f &point_in)
    {
        Eigen::Matrix<double, 3, 1> eigenVector;
        eigenVector << point_in.x, point_in.y, point_in.z;

        return eigenVector;
    }

    Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const cv::Mat &matrix_in)
    {
        Eigen::Matrix<double, 3, 3> eigenMatrix;

        eigenMatrix << matrix_in.at<float>(0, 0), matrix_in.at<float>(0, 1),
            matrix_in.at<float>(0, 2), matrix_in.at<float>(1, 0),
            matrix_in.at<float>(1, 1), matrix_in.at<float>(1, 2),
            matrix_in.at<float>(2, 0), matrix_in.at<float>(2, 1),
            matrix_in.at<float>(2, 2);

        return eigenMatrix;
    }

    Eigen::Matrix<double, 4, 4> Converter::toMatrix4d(const cv::Mat &matrix_in)
    {
        Eigen::Matrix<double, 4, 4> eigenMatrix;

        eigenMatrix << matrix_in.at<float>(0, 0), matrix_in.at<float>(0, 1),
            matrix_in.at<float>(0, 2), matrix_in.at<float>(0, 3),
            matrix_in.at<float>(1, 0), matrix_in.at<float>(1, 1),
            matrix_in.at<float>(1, 2), matrix_in.at<float>(1, 3),
            matrix_in.at<float>(2, 0), matrix_in.at<float>(2, 1),
            matrix_in.at<float>(2, 2), matrix_in.at<float>(2, 3),
            matrix_in.at<float>(3, 0), matrix_in.at<float>(3, 1),
            matrix_in.at<float>(3, 2), matrix_in.at<float>(3, 3);
        return eigenMatrix;
    }

    Eigen::Matrix<float, 3, 3> Converter::toMatrix3f(const cv::Mat &matrix_in)
    {
        Eigen::Matrix<float, 3, 3> eigenMatrix;

        eigenMatrix << matrix_in.at<float>(0, 0), matrix_in.at<float>(0, 1),
            matrix_in.at<float>(0, 2), matrix_in.at<float>(1, 0),
            matrix_in.at<float>(1, 1), matrix_in.at<float>(1, 2),
            matrix_in.at<float>(2, 0), matrix_in.at<float>(2, 1),
            matrix_in.at<float>(2, 2);

        return eigenMatrix;
    }

    Eigen::Matrix<float, 4, 4> Converter::toMatrix4f(const cv::Mat &matrix_in)
    {
        Eigen::Matrix<float, 4, 4> eigenMatrix;

        eigenMatrix << matrix_in.at<float>(0, 0), matrix_in.at<float>(0, 1),
            matrix_in.at<float>(0, 2), matrix_in.at<float>(0, 3),
            matrix_in.at<float>(1, 0), matrix_in.at<float>(1, 1),
            matrix_in.at<float>(1, 2), matrix_in.at<float>(1, 3),
            matrix_in.at<float>(2, 0), matrix_in.at<float>(2, 1),
            matrix_in.at<float>(2, 2), matrix_in.at<float>(2, 3),
            matrix_in.at<float>(3, 0), matrix_in.at<float>(3, 1),
            matrix_in.at<float>(3, 2), matrix_in.at<float>(3, 3);
        return eigenMatrix;
    }

    std::vector<float> Converter::toQuaternion(const cv::Mat &rotationMatrix_in)
    {
        Eigen::Matrix<double, 3, 3> eigenMatrix = toMatrix3d(rotationMatrix_in);
        Eigen::Quaterniond          quaternion(eigenMatrix);

        std::vector<float> quaternionVector(4);
        quaternionVector[0] = quaternion.x();
        quaternionVector[1] = quaternion.y();
        quaternionVector[2] = quaternion.z();
        quaternionVector[3] = quaternion.w();

        return quaternionVector;
    }

    cv::Mat Converter::toCvSkewMatrix(const cv::Mat &vector_in)
    {
        return (cv::Mat_<float>(3, 3) << 0,
                -vector_in.at<float>(2),
                vector_in.at<float>(1),
                vector_in.at<float>(2),
                0,
                -vector_in.at<float>(0),
                -vector_in.at<float>(1),
                vector_in.at<float>(0),
                0);
    }

    bool Converter::isRotationMatrix(const cv::Mat &rotationMatrix_in)
    {
        cv::Mat rotationTranspose;
        cv::transpose(rotationMatrix_in, rotationTranspose);
        cv::Mat shouldBeIdentity = rotationTranspose * rotationMatrix_in;
        cv::Mat identity         = cv::Mat::eye(3, 3, shouldBeIdentity.type());

        /* Accept small numerical drift around exact orthonormality. */
        return cv::norm(identity, shouldBeIdentity) < 1e-6;
    }

    std::vector<float> Converter::toEuler(const cv::Mat &rotationMatrix_in)
    {
        assert(isRotationMatrix(rotationMatrix_in));
        float symmetricSum = sqrt(rotationMatrix_in.at<float>(0, 0) *
                                      rotationMatrix_in.at<float>(0, 0) +
                                  rotationMatrix_in.at<float>(1, 0) *
                                      rotationMatrix_in.at<float>(1, 0));

        bool singular = symmetricSum < 1e-6; // If

        float xAngle, yAngle, zAngle;
        if (!singular)
        {
            xAngle = atan2(rotationMatrix_in.at<float>(2, 1),
                           rotationMatrix_in.at<float>(2, 2));
            yAngle = atan2(-rotationMatrix_in.at<float>(2, 0), symmetricSum);
            zAngle = atan2(rotationMatrix_in.at<float>(1, 0),
                           rotationMatrix_in.at<float>(0, 0));
        }
        else
        {
            /* Fall back to the gimbal-lock form near the singularity. */
            xAngle = atan2(-rotationMatrix_in.at<float>(1, 2),
                           rotationMatrix_in.at<float>(1, 1));
            yAngle = atan2(-rotationMatrix_in.at<float>(2, 0), symmetricSum);
            zAngle = 0;
        }

        std::vector<float> eulerAngles(3);
        eulerAngles[0] = xAngle;
        eulerAngles[1] = yAngle;
        eulerAngles[2] = zAngle;

        return eulerAngles;
    }

    Sophus::SE3<float> Converter::toSophus(const cv::Mat &transform_in)
    {
        Eigen::Matrix<double, 3, 3> eigenMatrix =
            toMatrix3d(transform_in.rowRange(0, 3).colRange(0, 3));
        Eigen::Quaternionf quaternion(eigenMatrix.cast<float>());

        Eigen::Matrix<float, 3, 1> translation =
            toVector3d(transform_in.rowRange(0, 3).col(3)).cast<float>();

        return Sophus::SE3<float>(quaternion, translation);
    }

    Sophus::Sim3f Converter::toSophus(const g2o::Sim3 &similarity_in)
    {
        return Sophus::Sim3f(Sophus::RxSO3d((float)similarity_in.scale(),
                                            similarity_in.rotation().matrix())
                                 .cast<float>(),
                             similarity_in.translation().cast<float>());
    }

} // namespace core
} // namespace vs_graphs
