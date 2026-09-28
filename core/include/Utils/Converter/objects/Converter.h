/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CONVERTER_H
#define CONVERTER_H

/*!
 * @file         Converter.h
 *
 * @brief        Declares pose and matrix conversions between library types.
 */

#include <vector>

#include <opencv2/core/core.hpp>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Utils/Converter/objects/ConverterStatus.h"
#include <Eigen/Dense>

#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"

namespace vs_graphs
{
namespace core
{
namespace utils
{
namespace converter
{

/*!
 * @brief        Converts poses and matrices between the cv, Eigen,
 *               g2o and Sophus representations used by the estimator.
 *
 *               All methods are stateless and thread-safe.
 */
class Converter
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief        Splits a descriptor matrix into one matrix per row.
     *
     * @param[in]    descriptors_in
     *               Matrix whose rows hold one descriptor each.
     *
     * @param[out] descriptorVector_out One single-row matrix per input row.
     * Each entry views the input data and stays valid only while the input
     * matrix lives.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toDescriptorVector(const cv::Mat        &descriptors_in,
                           std::vector<cv::Mat> &descriptorVector_out);

    /*!
     * @brief        Converts a 4x4 pose matrix to a rigid transform.
     *
     * @param[in]    transform_in
     *               Pose matrix. Shall be a 4x4 CV_32F matrix.
     *
     * @param[out] se3Quat_out Rigid transform built from the rotation and
     * translation parts of the input.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus toSE3Quat(const cv::Mat &transform_in,
                                                   g2o::SE3Quat  &se3Quat_out);
    /*!
     * @brief        Converts a Sophus SE3 pose to a rigid transform.
     *
     * @param[in]    transform_in
     *               Sophus SE3 pose to convert.
     *
     * @param[out] se3Quat_out Rigid transform with the same rotation and
     * translation, widened to double precision.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toSE3Quat(const Sophus::SE3f &transform_in, g2o::SE3Quat &se3Quat_out);
    /*!
     * @brief        Converts a similarity transform to a rigid
     *               transform.
     *
     * @param[in]    similarity_in
     *               Similarity transform to convert.
     *
     * @return       Rigid transform converted from the input.
     */
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &similarity_in);

    // TODO templetize these functions
    /*!
     * @brief        Converts a rigid transform to a 4x4 cv matrix.
     *
     * @param[in]    rigidTransform_in
     *               Rigid transform to convert.
     *
     * @param[out] cvMat_out 4x4 CV_32F homogeneous matrix.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const g2o::SE3Quat &rigidTransform_in, cv::Mat &cvMat_out);
    /*!
     * @brief        Converts a similarity transform to a 4x4 cv
     *               matrix.
     *
     * @param[in]    similarity_in
     *               Similarity transform to convert.
     *
     * @param[out] cvMat_out 4x4 CV_32F homogeneous matrix.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus toCvMat(const g2o::Sim3 &similarity_in,
                                                 cv::Mat         &cvMat_out);
    /*!
     * @brief        Converts a double 4x4 matrix to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen matrix to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::Matrix<double, 4, 4> &matrix_in,
                cv::Mat                           &cvMat_out);
    /*!
     * @brief        Converts a float 4x4 matrix to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen matrix to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::Matrix<float, 4, 4> &matrix_in,
                cv::Mat                          &cvMat_out);
    /*!
     * @brief        Converts a float 3x4 matrix to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen matrix to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::Matrix<float, 3, 4> &matrix_in,
                cv::Mat                          &cvMat_out);
    /*!
     * @brief        Converts a double 3x3 matrix to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen matrix to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::Matrix3d &matrix_in, cv::Mat &cvMat_out);
    /*!
     * @brief        Converts a double 3-vector to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen vector to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::Matrix<double, 3, 1> &matrix_in,
                cv::Mat                           &cvMat_out);
    /*!
     * @brief        Converts a float 3-vector to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen vector to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::Matrix<float, 3, 1> &matrix_in,
                cv::Mat                          &cvMat_out);
    /*!
     * @brief        Converts a float 3x3 matrix to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen matrix to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::Matrix<float, 3, 3> &matrix_in,
                cv::Mat                          &cvMat_out);

    /*!
     * @brief        Converts a dynamic float matrix to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen matrix to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::MatrixXf &matrix_in, cv::Mat &cvMat_out);
    /*!
     * @brief        Converts a dynamic double matrix to a cv matrix.
     *
     * @param[in]    matrix_in
     *               Eigen matrix to convert.
     *
     * @param[out] cvMat_out CV_32F matrix with the same dimensions and
     * coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvMat(const Eigen::MatrixXd &matrix_in, cv::Mat &cvMat_out);

    /*!
     * @brief        Assembles a 4x4 cv matrix from rotation and
     *               translation parts.
     *
     * @param[in]    rotation_in
     *               3x3 rotation part.
     * @param[in]    translation_in
     *               3x1 translation part.
     *
     * @param[out] cvSE3_out 4x4 CV_32F homogeneous matrix.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvSE3(const Eigen::Matrix<double, 3, 3> &rotation_in,
                const Eigen::Matrix<double, 3, 1> &translation_in,
                cv::Mat                           &cvSE3_out);
    /*!
     * @brief        Builds the skew-symmetric matrix of a 3-vector.
     *
     * @param[in]    vector_in
     *               Vector to convert. Shall hold three
     *               single-precision elements.
     *
     * @param[out] cvSkewMatrix_out 3x3 CV_32F skew-symmetric matrix.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toCvSkewMatrix(const cv::Mat &vector_in, cv::Mat &cvSkewMatrix_out);

    /*!
     * @brief        Copies a cv vector into a double 3-vector.
     *
     * @param[in]    vector_in
     *               Source vector. Shall hold at least three
     *               single-precision elements.
     *
     * @param[out] vector3d_out Eigen vector with the first three elements.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toVector3d(const cv::Mat               &vector_in,
                   Eigen::Matrix<double, 3, 1> &vector3d_out);
    /*!
     * @brief        Copies a cv vector into a float 3-vector.
     *
     * @param[in]    vector_in
     *               Source vector. Shall hold at least three
     *               single-precision elements.
     *
     * @param[out] vector3f_out Eigen vector with the first three elements.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toVector3f(const cv::Mat              &vector_in,
                   Eigen::Matrix<float, 3, 1> &vector3f_out);
    /*!
     * @brief        Copies a 3D point into a double 3-vector.
     *
     * @param[in]    point_in
     *               Source point.
     *
     * @param[out] vector3d_out Eigen vector with the point coordinates.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toVector3d(const cv::Point3f           &point_in,
                   Eigen::Matrix<double, 3, 1> &vector3d_out);
    /*!
     * @brief        Copies a cv matrix into a double 3x3 matrix.
     *
     * @param[in]    matrix_in
     *               Source matrix. Shall be a 3x3 CV_32F matrix.
     *
     * @param[out] matrix3d_out Eigen matrix with the copied coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toMatrix3d(const cv::Mat               &matrix_in,
                   Eigen::Matrix<double, 3, 3> &matrix3d_out);
    /*!
     * @brief        Copies a cv matrix into a double 4x4 matrix.
     *
     * @param[in]    matrix_in
     *               Source matrix. Shall be a 4x4 CV_32F matrix.
     *
     * @param[out] matrix4d_out Eigen matrix with the copied coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toMatrix4d(const cv::Mat               &matrix_in,
                   Eigen::Matrix<double, 4, 4> &matrix4d_out);
    /*!
     * @brief        Copies a cv matrix into a float 3x3 matrix.
     *
     * @param[in]    matrix_in
     *               Source matrix. Shall be a 3x3 CV_32F matrix.
     *
     * @param[out] matrix3f_out Eigen matrix with the copied coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toMatrix3f(const cv::Mat              &matrix_in,
                   Eigen::Matrix<float, 3, 3> &matrix3f_out);
    /*!
     * @brief        Copies a cv matrix into a float 4x4 matrix.
     *
     * @param[in]    matrix_in
     *               Source matrix. Shall be a 4x4 CV_32F matrix.
     *
     * @param[out] matrix4f_out Eigen matrix with the copied coefficients.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toMatrix4f(const cv::Mat              &matrix_in,
                   Eigen::Matrix<float, 4, 4> &matrix4f_out);
    /*!
     * @brief        Converts a rotation matrix to a quaternion vector.
     *
     * @param[in]    rotationMatrix_in
     *               Source matrix. Shall be a 3x3 CV_32F rotation
     *               matrix.
     *
     * @param[out] quaternion_out Four-element vector holding x, y, z and w in
     * that order.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toQuaternion(const cv::Mat      &rotationMatrix_in,
                     std::vector<float> &quaternion_out);

    /*!
     * @brief        Checks whether a matrix is a valid rotation
     *               matrix.
     *
     * @param[in]    rotationMatrix_in
     *               Matrix to check.
     *
     * @param[out] isRotationMatrix_out True when the transpose times the matrix
     * is the identity within 1e-6.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        isRotationMatrix(const cv::Mat &rotationMatrix_in,
                         bool          &isRotationMatrix_out);
    /*!
     * @brief        Converts a rotation matrix to Euler angles.
     *
     * @param[in]    rotationMatrix_in
     *               Source matrix. Shall be a 3x3 CV_32F rotation
     *               matrix.
     *
     * @param[out] euler_out Three-element vector holding the x, y and z angles
     * in radians in that order.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toEuler(const cv::Mat      &rotationMatrix_in,
                std::vector<float> &euler_out);

    // TODO: Sophus migration, to be deleted in the future
    /*!
     * @brief        Converts a 4x4 pose matrix to a Sophus SE3 pose.
     *
     * @param[in]    transform_in
     *               Pose matrix. Shall be a 4x4 CV_32F matrix.
     *
     * @param[out] sophus_out Sophus SE3 pose with the same rotation and
     * translation in single precision.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toSophus(const cv::Mat &transform_in, Sophus::SE3<float> &sophus_out);
    /*!
     * @brief        Converts a g2o similarity to a Sophus Sim3 pose.
     *
     * @param[in]    similarity_in
     *               Similarity transform to convert.
     *
     * @param[out] sophus_out Sophus Sim3 pose with the same rotation, scale and
     * translation in single precision.
     * @return CONVERTER_STATUS_SUCCESS.
     */
    [[nodiscard]] static ConverterStatus
        toSophus(const g2o::Sim3 &similarity_in, Sophus::Sim3f &sophus_out);
};

} // namespace converter
} // namespace utils
} // namespace core
} // namespace vs_graphs

#endif // CONVERTER_H
