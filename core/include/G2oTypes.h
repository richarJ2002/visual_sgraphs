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

/*!
 * @file            G2oTypes.h
 *
 * @brief           Declares the g2o vertex and edge types for visual-inertial
 *                  optimisation: IMU-camera poses, velocities, biases and IMU
 *                  preintegration constraints.
 */

#ifndef G2OTYPES_H
#define G2OTYPES_H

#include "EdgeAccRWStatus.h"
#include "EdgeGyroRWStatus.h"
#include "EdgeInertialGSStatus.h"
#include "EdgeInertialStatus.h"
#include "EdgeMonoOnlyPoseStatus.h"
#include "EdgeMonoStatus.h"
#include "EdgePriorAccStatus.h"
#include "EdgePriorGyroStatus.h"
#include "EdgePriorPoseImuStatus.h"
#include "EdgeStereoOnlyPoseStatus.h"
#include "EdgeStereoStatus.h"
#include "G2oTypesStatus.h"
#include "GDirectionStatus.h"
#include "ImuCamPoseStatus.h"
#include "InvDepthPointStatus.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"

#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Frame.h>
#include <KeyFrame.h>

#include "Utils/Converter/objects/Converter.h"
#include <math.h>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

class KeyFrame;
class Frame;
namespace camera_models
{
namespace geometriccamera
{
class GeometricCamera;
} // namespace geometriccamera
} // namespace camera_models

typedef Eigen::Matrix<double, 6, 1>   Vector6d;
typedef Eigen::Matrix<double, 9, 1>   Vector9d;
typedef Eigen::Matrix<double, 12, 1>  Vector12d;
typedef Eigen::Matrix<double, 15, 1>  Vector15d;
typedef Eigen::Matrix<double, 12, 12> Matrix12d;
typedef Eigen::Matrix<double, 15, 15> Matrix15d;
typedef Eigen::Matrix<double, 9, 9>   Matrix9d;

[[nodiscard]] G2oTypesStatus expSO3(const double     angleAxisX_in,
                                    const double     angleAxisY_in,
                                    const double     angleAxisZ_in,
                                    Eigen::Matrix3d &rotation_out);
[[nodiscard]] G2oTypesStatus expSO3(const Eigen::Vector3d &rotationVector_in,
                                    Eigen::Matrix3d       &rotation_out);

[[nodiscard]] G2oTypesStatus logSO3(const Eigen::Matrix3d &rotationMatrix_in,
                                    Eigen::Vector3d       &rotationVector_out);

[[nodiscard]] G2oTypesStatus
    inverseRightJacobianSO3(const Eigen::Vector3d &rotationVector_in,
                            Eigen::Matrix3d       &inverseRightJacobian_out);
[[nodiscard]] G2oTypesStatus
    rightJacobianSO3(const Eigen::Vector3d &rotationVector_in,
                     Eigen::Matrix3d       &rightJacobian_out);
[[nodiscard]] G2oTypesStatus
    rightJacobianSO3(const double     angleAxisX_in,
                     const double     angleAxisY_in,
                     const double     angleAxisZ_in,
                     Eigen::Matrix3d &rightJacobian_out);

[[nodiscard]] G2oTypesStatus
    computeSkewMatrix(const Eigen::Vector3d &angularVelocity_in,
                      Eigen::Matrix3d       &skewMatrix_out);
[[nodiscard]] G2oTypesStatus
    inverseRightJacobianSO3(const double     angleAxisX_in,
                            const double     angleAxisY_in,
                            const double     angleAxisZ_in,
                            Eigen::Matrix3d &inverseRightJacobian_out);

template <typename T = double>
[[nodiscard]] G2oTypesStatus
    normalizeRotation(const Eigen::Matrix<T, 3, 3> &R,
                      Eigen::Matrix<T, 3, 3>       &rotation_out)
{
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R,
                                                 Eigen::ComputeFullU |
                                                     Eigen::ComputeFullV);
    rotation_out = svd.matrixU() * svd.matrixV().transpose();
    return G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS;
}

class ImuCamPose
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuCamPose() {}
    ImuCamPose(KeyFrame *p_keyFrame_inout) :
        its(0)
    {
        // Load IMU pose
        Eigen::Vector3f keyFrameImuPosition{};
        if (p_keyFrame_inout->getImuPosition(keyFrameImuPosition) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        twb = keyFrameImuPosition.cast<double>();
        Eigen::Matrix3f keyFrameImuRotation{};
        if (p_keyFrame_inout->getImuRotation(keyFrameImuRotation) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Rwb = keyFrameImuRotation.cast<double>();

        // Load camera poses
        int camCount;
        if (p_keyFrame_inout->p_camera2)
            camCount = 2;
        else
            camCount = 1;

        tcw.resize(camCount);
        Rcw.resize(camCount);
        tcb.resize(camCount);
        Rcb.resize(camCount);
        Rbc.resize(camCount);
        tbc.resize(camCount);
        pCamera.resize(camCount);

        // Left camera
        Eigen::Vector3f keyFrameTranslation{};
        if (p_keyFrame_inout->getTranslation(keyFrameTranslation) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getTranslation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        tcw[0] = keyFrameTranslation.cast<double>();
        Eigen::Matrix3f keyFrameRotation{};
        if (p_keyFrame_inout->getRotation(keyFrameRotation) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getRotation returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        Rcw[0] = keyFrameRotation.cast<double>();
        tcb[0] =
            p_keyFrame_inout->imuCalibration.mTcb.translation().cast<double>();
        Rcb[0] = p_keyFrame_inout->imuCalibration.mTcb.rotationMatrix()
                     .cast<double>();
        Rbc[0] = Rcb[0].transpose();
        tbc[0] =
            p_keyFrame_inout->imuCalibration.mTbc.translation().cast<double>();
        pCamera[0] = p_keyFrame_inout->p_camera;
        bf         = p_keyFrame_inout->mbf;

        if (camCount > 1)
        {
            Sophus::SE3f keyFrameRelativePoseTrl{};
            if (p_keyFrame_inout->getRelativePoseTrl(keyFrameRelativePoseTrl) !=
                KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRelativePoseTrl returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Matrix4d stereoPose_leftCameraToRightCamera =
                keyFrameRelativePoseTrl.matrix().cast<double>();
            Rcw[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * Rcw[0];
            tcw[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * tcw[0] +
                stereoPose_leftCameraToRightCamera.block<3, 1>(0, 3);
            tcb[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * tcb[0] +
                stereoPose_leftCameraToRightCamera.block<3, 1>(0, 3);
            Rcb[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * Rcb[0];
            Rbc[1]     = Rcb[1].transpose();
            tbc[1]     = -Rbc[1] * tcb[1];
            pCamera[1] = p_keyFrame_inout->p_camera2;
        }

        // For posegraph 4DoF
        Rwb0 = Rwb;
        DR.setIdentity();
    }
    ImuCamPose(Frame *p_pF_inout) :
        its(0)
    {
        // Load IMU pose
        Eigen::Matrix<float, 3, 1> pFGetImuPosition{};
        if (p_pF_inout->getImuPosition(pFGetImuPosition) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuPosition returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        twb = pFGetImuPosition.cast<double>();
        Eigen::Matrix<float, 3, 3> pFImuRotation{};
        if (p_pF_inout->getImuRotation(pFImuRotation) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getImuRotation returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
        Rwb = pFImuRotation.cast<double>();

        // Load camera poses
        int camCount;
        if (p_pF_inout->p_camera2)
            camCount = 2;
        else
            camCount = 1;

        tcw.resize(camCount);
        Rcw.resize(camCount);
        tcb.resize(camCount);
        Rcb.resize(camCount);
        Rbc.resize(camCount);
        tbc.resize(camCount);
        pCamera.resize(camCount);

        // Left camera
        Sophus::SE3<float> pFGetPose{};
        if (p_pF_inout->getPose(pFGetPose) != FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        tcw[0] = pFGetPose.translation().cast<double>();
        Sophus::SE3<float> pFGetPose2{};
        if (p_pF_inout->getPose(pFGetPose2) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getPose returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        Rcw[0] = pFGetPose2.rotationMatrix().cast<double>();
        tcb[0] = p_pF_inout->imuCalibration.mTcb.translation().cast<double>();
        Rcb[0] =
            p_pF_inout->imuCalibration.mTcb.rotationMatrix().cast<double>();
        Rbc[0] = Rcb[0].transpose();
        tbc[0] = p_pF_inout->imuCalibration.mTbc.translation().cast<double>();
        pCamera[0] = p_pF_inout->p_camera;
        bf         = p_pF_inout->mbf;

        if (camCount > 1)
        {
            Sophus::SE3f pFRelativePoseTrl{};
            if (p_pF_inout->getRelativePoseTrl(pFRelativePoseTrl) !=
                FrameStatus::FRAME_STATUS_SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                             "%s: getRelativePoseTrl returned a failure status "
                             "although it cannot fail; continuing as before.",
                             __func__);
            }
            Eigen::Matrix4d stereoPose_leftCameraToRightCamera =
                pFRelativePoseTrl.matrix().cast<double>();
            Rcw[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * Rcw[0];
            tcw[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * tcw[0] +
                stereoPose_leftCameraToRightCamera.block<3, 1>(0, 3);
            tcb[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * tcb[0] +
                stereoPose_leftCameraToRightCamera.block<3, 1>(0, 3);
            Rcb[1] =
                stereoPose_leftCameraToRightCamera.block<3, 3>(0, 0) * Rcb[0];
            Rbc[1]     = Rcb[1].transpose();
            tbc[1]     = -Rbc[1] * tcb[1];
            pCamera[1] = p_pF_inout->p_camera2;
        }

        // For posegraph 4DoF
        Rwb0 = Rwb;
        DR.setIdentity();
    }
    ImuCamPose(Eigen::Matrix3d &cameraRotation_cameraToWorld_inout,
               Eigen::Vector3d &cameraTranslation_cameraToWorld_inout,
               KeyFrame        *p_keyFrame_inout) :
        its(0)
    {
        // This is only for posegrpah, we do not care about multicamera
        tcw.resize(1);
        Rcw.resize(1);
        tcb.resize(1);
        Rcb.resize(1);
        Rbc.resize(1);
        tbc.resize(1);
        pCamera.resize(1);

        tcb[0] =
            p_keyFrame_inout->imuCalibration.mTcb.translation().cast<double>();
        Rcb[0] = p_keyFrame_inout->imuCalibration.mTcb.rotationMatrix()
                     .cast<double>();
        Rbc[0] = Rcb[0].transpose();
        tbc[0] =
            p_keyFrame_inout->imuCalibration.mTbc.translation().cast<double>();
        twb = cameraRotation_cameraToWorld_inout * tcb[0] +
              cameraTranslation_cameraToWorld_inout;
        Rwb        = cameraRotation_cameraToWorld_inout * Rcb[0];
        Rcw[0]     = cameraRotation_cameraToWorld_inout.transpose();
        tcw[0]     = -Rcw[0] * cameraTranslation_cameraToWorld_inout;
        pCamera[0] = p_keyFrame_inout->p_camera;
        bf         = p_keyFrame_inout->mbf;

        // For posegraph 4DoF
        Rwb0 = Rwb;
        DR.setIdentity();
    }

    [[nodiscard]] ImuCamPoseStatus setParam(
        const std::vector<Eigen::Matrix3d> &cameraRotations_worldToCamera_in,
        const std::vector<Eigen::Vector3d> &cameraTranslations_worldToCamera_in,
        const std::vector<Eigen::Matrix3d> &extrinsicRotations_cameraToBody_in,
        const std::vector<Eigen::Vector3d>
                     &extrinsicTranslations_cameraToBody_in,
        const double &baselineFocalProduct_in);

    [[nodiscard]] ImuCamPoseStatus
        update(const double *p_updateVector_in); // update in the imu reference
    [[nodiscard]] ImuCamPoseStatus updateW(
        const double *p_updateVector_in); // update in the world reference
    [[nodiscard]] ImuCamPoseStatus
        project(const Eigen::Vector3d &Xw_in,
                Eigen::Vector2d       &projection_out,
                int                    cameraIndex_in = 0) const; // Mono
    [[nodiscard]] ImuCamPoseStatus
         projectStereo(const Eigen::Vector3d &Xw_in,
                       Eigen::Vector3d       &stereo_out,
                       int cameraIndex_in = 0) const; // Stereo
    bool isDepthPositive(const Eigen::Vector3d &Xw_in,
                         int                    cameraIndex_in = 0) const;

  public:
    // For IMU
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;

    // For set of cameras
    std::vector<Eigen::Matrix3d>                                   Rcw;
    std::vector<Eigen::Vector3d>                                   tcw;
    std::vector<Eigen::Matrix3d>                                   Rcb, Rbc;
    std::vector<Eigen::Vector3d>                                   tcb, tbc;
    double                                                         bf;
    std::vector<camera_models::geometriccamera::GeometricCamera *> pCamera;

    // For posegraph 4DoF
    Eigen::Matrix3d Rwb0;
    Eigen::Matrix3d DR;

    int its;
};

class InvDepthPoint
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    InvDepthPoint() {}
    InvDepthPoint(double    rho_in,
                  double    u_in,
                  double    v_in,
                  KeyFrame *p_hostKeyFrame_inout) :
        rho(rho_in),
        u(u_in),
        v(v_in),
        fx(p_hostKeyFrame_inout->fx),
        fy(p_hostKeyFrame_inout->fy),
        cx(p_hostKeyFrame_inout->cx),
        cy(p_hostKeyFrame_inout->cy),
        bf(p_hostKeyFrame_inout->mbf)
    {}

    [[nodiscard]] InvDepthPointStatus
        update(const double *p_inverseDepthDelta_in);

    double rho;
    double u, v; // they are not variables, observation in the host frame

    double fx, fy, cx, cy, bf; // from host frame

    int its;
};

// Optimizable parameters are IMU pose
class VertexPose : public g2o::BaseVertex<6, ImuCamPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose() {}
    VertexPose(KeyFrame *p_keyFrame_inout)
    {
        setEstimate(ImuCamPose(p_keyFrame_inout));
    }
    VertexPose(Frame *p_pF_inout)
    {
        setEstimate(ImuCamPose(p_pF_inout));
    }

    virtual bool read(std::istream &inputStream_inout);
    virtual bool write(std::ostream &outputStream_out) const;

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double *p_update_in)
    {
        if (_estimate.update(p_update_in) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: update returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        updateCache();
    }
};

class VertexPose4DoF : public g2o::BaseVertex<4, ImuCamPose>
{
    // Translation and yaw are the only optimizable variables
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPose4DoF() {}
    VertexPose4DoF(KeyFrame *p_keyFrame_inout)
    {
        setEstimate(ImuCamPose(p_keyFrame_inout));
    }
    VertexPose4DoF(Frame *p_pF_inout)
    {
        setEstimate(ImuCamPose(p_pF_inout));
    }
    VertexPose4DoF(Eigen::Matrix3d &cameraRotation_cameraToWorld_inout,
                   Eigen::Vector3d &cameraTranslation_cameraToWorld_inout,
                   KeyFrame        *p_keyFrame_inout)
    {

        setEstimate(ImuCamPose(cameraRotation_cameraToWorld_inout,
                               cameraTranslation_cameraToWorld_inout,
                               p_keyFrame_inout));
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double *p_update_in)
    {
        double update6DoF[6];
        update6DoF[0] = 0;
        update6DoF[1] = 0;
        update6DoF[2] = p_update_in[0];
        update6DoF[3] = p_update_in[1];
        update6DoF[4] = p_update_in[2];
        update6DoF[5] = p_update_in[3];
        if (_estimate.updateW(update6DoF) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: updateW returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        updateCache();
    }
};

class VertexVelocity : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexVelocity() {}
    VertexVelocity(KeyFrame *p_keyFrame_inout)
    {
        Eigen::Vector3f keyFrameVelocity{};
        if (p_keyFrame_inout->getVelocity(keyFrameVelocity) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(keyFrameVelocity.cast<double>());
    }
    VertexVelocity(Frame *p_pF_inout)
    {
        Eigen::Vector3f pFGetVelocity{};
        if (p_pF_inout->getVelocity(pFGetVelocity) !=
            FrameStatus::FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getVelocity returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(pFGetVelocity.cast<double>());
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double *p_update_in)
    {
        Eigen::Vector3d uv;
        uv << p_update_in[0], p_update_in[1], p_update_in[2];
        setEstimate(estimate() + uv);
    }
};

class VertexGyroBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGyroBias() {}
    VertexGyroBias(KeyFrame *p_keyFrame_inout)
    {
        Eigen::Vector3f keyFrameGyroBias{};
        if (p_keyFrame_inout->getGyroBias(keyFrameGyroBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getGyroBias returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(keyFrameGyroBias.cast<double>());
    }
    VertexGyroBias(Frame *p_pF_inout)
    {
        Eigen::Vector3d bg;
        bg << p_pF_inout->imuBias.bwx, p_pF_inout->imuBias.bwy,
            p_pF_inout->imuBias.bwz;
        setEstimate(bg);
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double *p_update_in)
    {
        Eigen::Vector3d ubg;
        ubg << p_update_in[0], p_update_in[1], p_update_in[2];
        setEstimate(estimate() + ubg);
    }
};

class VertexAccBias : public g2o::BaseVertex<3, Eigen::Vector3d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexAccBias() {}
    VertexAccBias(KeyFrame *p_keyFrame_inout)
    {
        Eigen::Vector3f keyFrameAccBias{};
        if (p_keyFrame_inout->getAccBias(keyFrameAccBias) !=
            KeyFrameStatus::KEY_FRAME_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getAccBias returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        setEstimate(keyFrameAccBias.cast<double>());
    }
    VertexAccBias(Frame *p_pF_inout)
    {
        Eigen::Vector3d ba;
        ba << p_pF_inout->imuBias.bax, p_pF_inout->imuBias.bay,
            p_pF_inout->imuBias.baz;
        setEstimate(ba);
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double *p_update_in)
    {
        Eigen::Vector3d uba;
        uba << p_update_in[0], p_update_in[1], p_update_in[2];
        setEstimate(estimate() + uba);
    }
};

// Gravity direction vertex
class GDirection
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GDirection() :
        gravityRotation_gravityToWorld(Eigen::Matrix3d::Identity()),
        gravityRotation_worldToGravity(Eigen::Matrix3d::Identity()),
        its(0)
    {}

    explicit GDirection(
        const Eigen::Matrix3d &gravityRotation_gravityToWorld_in) :
        gravityRotation_gravityToWorld(gravityRotation_gravityToWorld_in),
        gravityRotation_worldToGravity(
            gravityRotation_gravityToWorld_in.transpose()),
        its(0)
    {}

    [[nodiscard]] GDirectionStatus update(const double *p_pu_in)
    {
        Eigen::Matrix3d rotation{};
        if (expSO3(p_pu_in[0], p_pu_in[1], 0.0, rotation) !=
            G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: expSO3 returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        gravityRotation_gravityToWorld =
            gravityRotation_gravityToWorld * rotation;
        gravityRotation_worldToGravity =
            gravityRotation_gravityToWorld.transpose();

        return GDirectionStatus::GDIRECTION_STATUS_SUCCESS;
    }

    Eigen::Matrix3d gravityRotation_gravityToWorld,
        gravityRotation_worldToGravity;

    int its;
};

class VertexGDir : public g2o::BaseVertex<2, GDirection>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexGDir() {}
    VertexGDir(Eigen::Matrix3d rwg_in)
    {
        setEstimate(GDirection(rwg_in));
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double *p_update_in)
    {
        if (_estimate.update(p_update_in) !=
            GDirectionStatus::GDIRECTION_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: update returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        updateCache();
    }
};

// scale vertex
class VertexScale : public g2o::BaseVertex<1, double>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexScale()
    {
        setEstimate(1.0);
    }
    VertexScale(double ps_in)
    {
        setEstimate(ps_in);
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    virtual void setToOriginImpl()
    {
        setEstimate(1.0);
    }

    virtual void oplusImpl(const double *p_update_in)
    {
        setEstimate(estimate() * exp(*p_update_in));
    }
};

// Inverse depth point (just one parameter, inverse depth at the host frame)
class VertexInvDepth : public g2o::BaseVertex<1, InvDepthPoint>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexInvDepth() {}
    VertexInvDepth(double    invDepth_in,
                   double    u_in,
                   double    v_in,
                   KeyFrame *p_hostKeyFrame_inout)
    {
        setEstimate(
            InvDepthPoint(invDepth_in, u_in, v_in, p_hostKeyFrame_inout));
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    virtual void setToOriginImpl() {}

    virtual void oplusImpl(const double *p_update_in)
    {
        if (_estimate.update(p_update_in) !=
            InvDepthPointStatus::INV_DEPTH_POINT_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: update returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        updateCache();
    }
};

class EdgeMono
    : public g2o::
          BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMono(int cameraIndex_in = 0) :
        cam_idx(cameraIndex_in)
    {}

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[1]);
        const Eigen::Vector2d observation(_measurement);
        Eigen::Vector2d       projection{};
        if (p_poseVertex->estimate().project(p_pointVertex->estimate(),
                                             projection,
                                             cam_idx) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: project returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        _error = observation - projection;
    }

    virtual void linearizeOplus();

    bool isDepthPositive()
    {
        const g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[1]);
        return p_poseVertex->estimate().isDepthPositive(
            p_pointVertex->estimate(),
            cam_idx);
    }

    [[nodiscard]] EdgeMonoStatus
        getJacobian(Eigen::Matrix<double, 2, 9> &jacobian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 2, 9> J;
        J.block<2, 3>(0, 0) = _jacobianOplusXi;
        J.block<2, 6>(0, 3) = _jacobianOplusXj;
        jacobian_out        = J;
        return EdgeMonoStatus::EDGE_MONO_STATUS_SUCCESS;
    }

    [[nodiscard]] EdgeMonoStatus
        getHessian(Eigen::Matrix<double, 9, 9> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 2, 9> J;
        J.block<2, 3>(0, 0) = _jacobianOplusXi;
        J.block<2, 6>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeMonoStatus::EDGE_MONO_STATUS_SUCCESS;
    }

  public:
    const int cam_idx;
};

class EdgeMonoOnlyPose
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeMonoOnlyPose(const Eigen::Vector3f &Xw_in, int cameraIndex_in = 0) :
        Xw(Xw_in.cast<double>()),
        cam_idx(cameraIndex_in)
    {}

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[0]);
        const Eigen::Vector2d observation(_measurement);
        Eigen::Vector2d       projection{};
        if (p_poseVertex->estimate().project(Xw, projection, cam_idx) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: project returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        _error = observation - projection;
    }

    virtual void linearizeOplus();

    bool isDepthPositive()
    {
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[0]);
        return p_poseVertex->estimate().isDepthPositive(Xw, cam_idx);
    }

    [[nodiscard]] EdgeMonoOnlyPoseStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
    {
        linearizeOplus();
        hessian_out =
            _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        return EdgeMonoOnlyPoseStatus::EDGE_MONO_ONLY_POSE_STATUS_SUCCESS;
    }

  public:
    const Eigen::Vector3d Xw;
    const int             cam_idx;
};

class EdgeStereo
    : public g2o::
          BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, VertexPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereo(int cameraIndex_in = 0) :
        cam_idx(cameraIndex_in)
    {}

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const g2o::VertexSBAPointXYZ *p_pointVertex =
            static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[1]);
        const Eigen::Vector3d observation(_measurement);
        Eigen::Vector3d       stereo{};
        if (p_poseVertex->estimate().projectStereo(p_pointVertex->estimate(),
                                                   stereo,
                                                   cam_idx) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: projectStereo returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        _error = observation - stereo;
    }

    virtual void linearizeOplus();

    [[nodiscard]] EdgeStereoStatus
        getJacobian(Eigen::Matrix<double, 3, 9> &jacobian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 9> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 6>(0, 3) = _jacobianOplusXj;
        jacobian_out        = J;
        return EdgeStereoStatus::EDGE_STEREO_STATUS_SUCCESS;
    }

    [[nodiscard]] EdgeStereoStatus
        getHessian(Eigen::Matrix<double, 9, 9> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 9> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 6>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeStereoStatus::EDGE_STEREO_STATUS_SUCCESS;
    }

  public:
    const int cam_idx;
};

class EdgeStereoOnlyPose
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeStereoOnlyPose(const Eigen::Vector3f &Xw_in, int cameraIndex_in = 0) :
        Xw(Xw_in.cast<double>()),
        cam_idx(cameraIndex_in)
    {}

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const VertexPose *p_poseVertex =
            static_cast<const VertexPose *>(_vertices[0]);
        const Eigen::Vector3d observation(_measurement);
        Eigen::Vector3d       stereo{};
        if (p_poseVertex->estimate().projectStereo(Xw, stereo, cam_idx) !=
            ImuCamPoseStatus::IMU_CAM_POSE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: projectStereo returned a failure status although "
                         "it cannot fail; continuing as before.",
                         __func__);
        }
        _error = observation - stereo;
    }

    virtual void linearizeOplus();

    [[nodiscard]] EdgeStereoOnlyPoseStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
    {
        linearizeOplus();
        hessian_out =
            _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        return EdgeStereoOnlyPoseStatus::EDGE_STEREO_ONLY_POSE_STATUS_SUCCESS;
    }

  public:
    const Eigen::Vector3d Xw; // 3D point coordinates
    const int             cam_idx;
};

class EdgeInertial : public g2o::BaseMultiEdge<9, Vector9d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeInertial(IMU::Preintegrated *p_int_inout) :
        JRg(p_int_inout->JRg.cast<double>()),
        JVg(p_int_inout->JVg.cast<double>()),
        JPg(p_int_inout->JPg.cast<double>()),
        JVa(p_int_inout->JVa.cast<double>()),
        JPa(p_int_inout->JPa.cast<double>()),
        p_preintegrated(p_int_inout),
        dt(p_int_inout->dT)
    {
        // This edge links 6 vertices
        resize(6);
        g << 0, 0, -IMU::GRAVITY_VALUE;

        Matrix9d Info =
            p_int_inout->C.block<9, 9>(0, 0).cast<double>().inverse();
        Info = (Info + Info.transpose()) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(Info);
        Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 9; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        Info = es.eigenvectors() * eigs.asDiagonal() *
               es.eigenvectors().transpose();
        setInformation(Info);
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void         computeError();
    virtual void linearizeOplus();

    [[nodiscard]] EdgeInertialStatus
        getHessian(Eigen::Matrix<double, 24, 24> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 24> J;
        J.block<9, 6>(0, 0)  = _jacobianOplus[0];
        J.block<9, 3>(0, 6)  = _jacobianOplus[1];
        J.block<9, 3>(0, 9)  = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        hessian_out          = J.transpose() * information() * J;
        return EdgeInertialStatus::EDGE_INERTIAL_STATUS_SUCCESS;
    }

    [[nodiscard]] EdgeInertialStatus
        getHessianNoPose1(Eigen::Matrix<double, 18, 18> &hessianNoPose1_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 18> J;
        J.block<9, 3>(0, 0)  = _jacobianOplus[1];
        J.block<9, 3>(0, 3)  = _jacobianOplus[2];
        J.block<9, 3>(0, 6)  = _jacobianOplus[3];
        J.block<9, 6>(0, 9)  = _jacobianOplus[4];
        J.block<9, 3>(0, 15) = _jacobianOplus[5];
        hessianNoPose1_out   = J.transpose() * information() * J;
        return EdgeInertialStatus::EDGE_INERTIAL_STATUS_SUCCESS;
    }

    [[nodiscard]] EdgeInertialStatus
        getHessian2(Eigen::Matrix<double, 9, 9> &hessian2_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 9> J;
        J.block<9, 6>(0, 0) = _jacobianOplus[4];
        J.block<9, 3>(0, 6) = _jacobianOplus[5];
        hessian2_out        = J.transpose() * information() * J;
        return EdgeInertialStatus::EDGE_INERTIAL_STATUS_SUCCESS;
    }

    const Eigen::Matrix3d JRg, JVg, JPg;
    const Eigen::Matrix3d JVa, JPa;
    IMU::Preintegrated   *p_preintegrated;
    const double          dt;
    Eigen::Vector3d       g;
};

// Edge inertial whre gravity is included as optimizable variable and it is not
// supposed to be pointing in -z axis, as well as scale
class EdgeInertialGS : public g2o::BaseMultiEdge<9, Vector9d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // EdgeInertialGS(IMU::Preintegrated* pInt);
    EdgeInertialGS(IMU::Preintegrated *p_int_inout) :
        JRg(p_int_inout->JRg.cast<double>()),
        JVg(p_int_inout->JVg.cast<double>()),
        JPg(p_int_inout->JPg.cast<double>()),
        JVa(p_int_inout->JVa.cast<double>()),
        JPa(p_int_inout->JPa.cast<double>()),
        p_preintegrated(p_int_inout),
        dt(p_int_inout->dT)
    {
        // This edge links 8 vertices
        resize(8);
        gI << 0, 0, -IMU::GRAVITY_VALUE;

        Matrix9d Info =
            p_int_inout->C.block<9, 9>(0, 0).cast<double>().inverse();
        Info = (Info + Info.transpose()) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(Info);
        Eigen::Matrix<double, 9, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 9; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        Info = es.eigenvectors() * eigs.asDiagonal() *
               es.eigenvectors().transpose();
        setInformation(Info);
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void         computeError();
    virtual void linearizeOplus();

    const Eigen::Matrix3d JRg, JVg, JPg;
    const Eigen::Matrix3d JVa, JPa;
    IMU::Preintegrated   *p_preintegrated;
    const double          dt;
    Eigen::Vector3d       g, gI;

    [[nodiscard]] EdgeInertialGSStatus
        getHessian(Eigen::Matrix<double, 27, 27> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 27> J;
        J.block<9, 6>(0, 0)  = _jacobianOplus[0];
        J.block<9, 3>(0, 6)  = _jacobianOplus[1];
        J.block<9, 3>(0, 9)  = _jacobianOplus[2];
        J.block<9, 3>(0, 12) = _jacobianOplus[3];
        J.block<9, 6>(0, 15) = _jacobianOplus[4];
        J.block<9, 3>(0, 21) = _jacobianOplus[5];
        J.block<9, 2>(0, 24) = _jacobianOplus[6];
        J.block<9, 1>(0, 26) = _jacobianOplus[7];
        hessian_out          = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    [[nodiscard]] EdgeInertialGSStatus
        getHessian2(Eigen::Matrix<double, 27, 27> &hessian2_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 27> J;
        J.block<9, 3>(0, 0)  = _jacobianOplus[2];
        J.block<9, 3>(0, 3)  = _jacobianOplus[3];
        J.block<9, 2>(0, 6)  = _jacobianOplus[6];
        J.block<9, 1>(0, 8)  = _jacobianOplus[7];
        J.block<9, 3>(0, 9)  = _jacobianOplus[1];
        J.block<9, 3>(0, 12) = _jacobianOplus[5];
        J.block<9, 6>(0, 15) = _jacobianOplus[0];
        J.block<9, 6>(0, 21) = _jacobianOplus[4];
        hessian2_out         = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    [[nodiscard]] EdgeInertialGSStatus
        getHessian3(Eigen::Matrix<double, 9, 9> &hessian3_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 9> J;
        J.block<9, 3>(0, 0) = _jacobianOplus[2];
        J.block<9, 3>(0, 3) = _jacobianOplus[3];
        J.block<9, 2>(0, 6) = _jacobianOplus[6];
        J.block<9, 1>(0, 8) = _jacobianOplus[7];
        hessian3_out        = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    [[nodiscard]] EdgeInertialGSStatus
        getHessianScale(Eigen::Matrix<double, 1, 1> &hessianScale_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 1> J = _jacobianOplus[7];
        hessianScale_out              = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    [[nodiscard]] EdgeInertialGSStatus
        getHessianBiasGyro(Eigen::Matrix<double, 3, 3> &hessianBiasGyro_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 3> J = _jacobianOplus[2];
        hessianBiasGyro_out           = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    [[nodiscard]] EdgeInertialGSStatus
        getHessianBiasAcc(Eigen::Matrix<double, 3, 3> &hessianBiasAcc_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 3> J = _jacobianOplus[3];
        hessianBiasAcc_out            = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }

    [[nodiscard]] EdgeInertialGSStatus
        getHessianGDir(Eigen::Matrix<double, 2, 2> &hessianGDir_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 9, 2> J = _jacobianOplus[6];
        hessianGDir_out               = J.transpose() * information() * J;
        return EdgeInertialGSStatus::EDGE_INERTIAL_GSSTATUS_SUCCESS;
    }
};

class EdgeGyroRW
    : public g2o::
          BaseBinaryEdge<3, Eigen::Vector3d, VertexGyroBias, VertexGyroBias>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeGyroRW() {}

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const VertexGyroBias *p_firstGyroBiasVertex =
            static_cast<const VertexGyroBias *>(_vertices[0]);
        const VertexGyroBias *p_secondGyroBiasVertex =
            static_cast<const VertexGyroBias *>(_vertices[1]);
        _error = p_secondGyroBiasVertex->estimate() -
                 p_firstGyroBiasVertex->estimate();
    }

    virtual void linearizeOplus()
    {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    [[nodiscard]] EdgeGyroRWStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeGyroRWStatus::EDGE_GYRO_RWSTATUS_SUCCESS;
    }

    [[nodiscard]] EdgeGyroRWStatus getHessian2(Eigen::Matrix3d &hessian2_out)
    {
        linearizeOplus();
        hessian2_out =
            _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
        return EdgeGyroRWStatus::EDGE_GYRO_RWSTATUS_SUCCESS;
    }
};

class EdgeAccRW
    : public g2o::
          BaseBinaryEdge<3, Eigen::Vector3d, VertexAccBias, VertexAccBias>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeAccRW() {}

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const VertexAccBias *p_firstAccelerometerBiasVertex =
            static_cast<const VertexAccBias *>(_vertices[0]);
        const VertexAccBias *p_secondAccelerometerBiasVertex =
            static_cast<const VertexAccBias *>(_vertices[1]);
        _error = p_secondAccelerometerBiasVertex->estimate() -
                 p_firstAccelerometerBiasVertex->estimate();
    }

    virtual void linearizeOplus()
    {
        _jacobianOplusXi = -Eigen::Matrix3d::Identity();
        _jacobianOplusXj.setIdentity();
    }

    [[nodiscard]] EdgeAccRWStatus
        getHessian(Eigen::Matrix<double, 6, 6> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 0) = _jacobianOplusXi;
        J.block<3, 3>(0, 3) = _jacobianOplusXj;
        hessian_out         = J.transpose() * information() * J;
        return EdgeAccRWStatus::EDGE_ACC_RWSTATUS_SUCCESS;
    }

    [[nodiscard]] EdgeAccRWStatus getHessian2(Eigen::Matrix3d &hessian2_out)
    {
        linearizeOplus();
        hessian2_out =
            _jacobianOplusXj.transpose() * information() * _jacobianOplusXj;
        return EdgeAccRWStatus::EDGE_ACC_RWSTATUS_SUCCESS;
    }
};

class ConstraintPoseImu
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintPoseImu(const Eigen::Matrix3d &bodyRotation_bodyToWorld_in,
                      const Eigen::Vector3d &bodyTranslation_bodyToWorld_in,
                      const Eigen::Vector3d &vwb_in,
                      const Eigen::Vector3d &bg_in,
                      const Eigen::Vector3d &ba_in,
                      const Matrix15d       &H_in) :
        Rwb(bodyRotation_bodyToWorld_in),
        twb(bodyTranslation_bodyToWorld_in),
        vwb(vwb_in),
        bg(bg_in),
        ba(ba_in),
        H(H_in)
    {
        H = (H + H) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
        Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 15; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        H = es.eigenvectors() * eigs.asDiagonal() *
            es.eigenvectors().transpose();
    }

    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d vwb;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
    Matrix15d       H;
};

class EdgePriorPoseImu : public g2o::BaseMultiEdge<15, Vector15d>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePriorPoseImu(ConstraintPoseImu *p_c_inout)
    {
        resize(4);
        Rwb = p_c_inout->Rwb;
        twb = p_c_inout->twb;
        vwb = p_c_inout->vwb;
        bg  = p_c_inout->bg;
        ba  = p_c_inout->ba;
        setInformation(p_c_inout->H);
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void         computeError();
    virtual void linearizeOplus();

    [[nodiscard]] EdgePriorPoseImuStatus
        getHessian(Eigen::Matrix<double, 15, 15> &hessian_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 15, 15> J;
        J.block<15, 6>(0, 0)  = _jacobianOplus[0];
        J.block<15, 3>(0, 6)  = _jacobianOplus[1];
        J.block<15, 3>(0, 9)  = _jacobianOplus[2];
        J.block<15, 3>(0, 12) = _jacobianOplus[3];
        hessian_out           = J.transpose() * information() * J;
        return EdgePriorPoseImuStatus::EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS;
    }

    [[nodiscard]] EdgePriorPoseImuStatus
        getHessianNoPose(Eigen::Matrix<double, 9, 9> &hessianNoPose_out)
    {
        linearizeOplus();
        Eigen::Matrix<double, 15, 9> J;
        J.block<15, 3>(0, 0) = _jacobianOplus[1];
        J.block<15, 3>(0, 3) = _jacobianOplus[2];
        J.block<15, 3>(0, 6) = _jacobianOplus[3];
        hessianNoPose_out    = J.transpose() * information() * J;
        return EdgePriorPoseImuStatus::EDGE_PRIOR_POSE_IMU_STATUS_SUCCESS;
    }
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb, vwb;
    Eigen::Vector3d bg, ba;
};

// Priors for biases
class EdgePriorAcc
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexAccBias>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorAcc(const Eigen::Vector3f &bprior_in) :
        bprior(bprior_in.cast<double>())
    {}

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const VertexAccBias *p_accelerometerBiasVertex =
            static_cast<const VertexAccBias *>(_vertices[0]);
        _error = bprior - p_accelerometerBiasVertex->estimate();
    }
    virtual void linearizeOplus();

    [[nodiscard]] EdgePriorAccStatus
        getHessian(Eigen::Matrix<double, 3, 3> &hessian_out)
    {
        linearizeOplus();
        hessian_out =
            _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        return EdgePriorAccStatus::EDGE_PRIOR_ACC_STATUS_SUCCESS;
    }

    const Eigen::Vector3d bprior;
};

class EdgePriorGyro
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexGyroBias>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePriorGyro(const Eigen::Vector3f &bprior_in) :
        bprior(bprior_in.cast<double>())
    {}

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const VertexGyroBias *p_gyroBiasVertex =
            static_cast<const VertexGyroBias *>(_vertices[0]);
        _error = bprior - p_gyroBiasVertex->estimate();
    }
    virtual void linearizeOplus();

    [[nodiscard]] EdgePriorGyroStatus
        getHessian(Eigen::Matrix<double, 3, 3> &hessian_out)
    {
        linearizeOplus();
        hessian_out =
            _jacobianOplusXi.transpose() * information() * _jacobianOplusXi;
        return EdgePriorGyroStatus::EDGE_PRIOR_GYRO_STATUS_SUCCESS;
    }

    const Eigen::Vector3d bprior;
};

class Edge4DoF
    : public g2o::BaseBinaryEdge<6, Vector6d, VertexPose4DoF, VertexPose4DoF>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Edge4DoF(const Eigen::Matrix4d &deltaT_in)
    {
        dTij = deltaT_in;
        dRij = deltaT_in.block<3, 3>(0, 0);
        dtij = deltaT_in.block<3, 1>(0, 3);
    }

    virtual bool read([[maybe_unused]] std::istream &is_inout)
    {
        return false;
    }
    virtual bool write([[maybe_unused]] std::ostream &os_inout) const
    {
        return false;
    }

    void computeError()
    {
        const VertexPose4DoF *VPi =
            static_cast<const VertexPose4DoF *>(_vertices[0]);
        const VertexPose4DoF *VPj =
            static_cast<const VertexPose4DoF *>(_vertices[1]);
        Eigen::Vector3d rotationVector{};
        if (logSO3(VPi->estimate().Rcw[0] * VPj->estimate().Rcw[0].transpose() *
                       dRij.transpose(),
                   rotationVector) != G2oTypesStatus::G2O_TYPES_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: logSO3 returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        _error << rotationVector,
            VPi->estimate().Rcw[0] * (-VPj->estimate().Rcw[0].transpose() *
                                      VPj->estimate().tcw[0]) +
                VPi->estimate().tcw[0] - dtij;
    }

    // virtual void linearizeOplus(); // numerical implementation

    Eigen::Matrix4d dTij;
    Eigen::Matrix3d dRij;
    Eigen::Vector3d dtij;
};

} // namespace core
} // namespace vs_graphs

#endif // G2OTYPES_H
