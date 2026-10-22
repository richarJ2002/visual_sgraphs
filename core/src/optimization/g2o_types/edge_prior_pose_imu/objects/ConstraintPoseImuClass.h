/*!
 * @file            ConstraintPoseImuClass.h
 *
 * @brief           Declares the ConstraintPoseImu prior on the full IMU
 *                  state of one frame, consumed by EdgePriorPoseImu.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_POSE_IMU_OBJECTS_CONSTRAINT_POSE_IMU_CLASS_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_POSE_IMU_OBJECTS_CONSTRAINT_POSE_IMU_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace edge_prior_pose_imu
{

/*!
 * @brief           Prior on the full IMU state of one frame: pose, velocity and
 *                  both biases, with the information matrix of that prior.
 *                  Consumed by EdgePriorPoseImu.
 */
class ConstraintPoseImu
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * @brief           Stores the prior values and the symmetric part of the
     *                  information matrix, (H_in + H_in^T) / 2, with its
     *                  eigenvalues below 1e-12 set to zero.
     *
     * @param[in]       bodyRotation_bodyToWorld_in
     *                  Prior rotation of the IMU (body) frame in the world.
     *
     * @param[in]       bodyTranslation_bodyToWorld_in
     *                  Prior position of the IMU origin in the world, metres.
     *
     * @param[in]       vwb_in
     *                  Prior velocity of the IMU in the world frame, metres per
     *                  second.
     *
     * @param[in]       bg_in
     *                  Prior gyroscope bias, radians per second.
     *
     * @param[in]       ba_in
     *                  Prior accelerometer bias, metres per second squared.
     *
     * @param[in]       H_in
     *                  Information matrix of the prior over rotation (3),
     *                  translation (3), velocity (3), gyroscope bias (3),
     *                  accelerometer bias (3).
     */
    ConstraintPoseImu(const Eigen::Matrix3d &bodyRotation_bodyToWorld_in,
                      const Eigen::Vector3d &bodyTranslation_bodyToWorld_in,
                      const Eigen::Vector3d &vwb_in,
                      const Eigen::Vector3d &bg_in,
                      const Eigen::Vector3d &ba_in,
                      const Eigen::Matrix<double, 15, 15> &H_in) :
        Rwb(bodyRotation_bodyToWorld_in),
        twb(bodyTranslation_bodyToWorld_in),
        vwb(vwb_in),
        bg(bg_in),
        ba(ba_in),
        H(H_in)
    {
        /* eval() builds the sum before H is overwritten (no aliasing). */
        H = (H + H.transpose()).eval() / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(H);
        Eigen::Matrix<double, 15, 1> eigs = es.eigenvalues();
        for (int i = 0; i < 15; i++)
            if (eigs[i] < 1e-12)
                eigs[i] = 0;
        H = es.eigenvectors() * eigs.asDiagonal() *
            es.eigenvectors().transpose();
    }

    /*!
     * @brief           Prior rotation of the IMU (body) frame in the world
     *                  frame.
     */
    Eigen::Matrix3d               Rwb;
    /*!
     * @brief           Prior position of the IMU origin in the world frame,
     *                  metres.
     */
    Eigen::Vector3d               twb;
    /*!
     * @brief           Prior velocity of the IMU in the world frame, metres per
     *                  second.
     */
    Eigen::Vector3d               vwb;
    /*!
     * @brief           Prior gyroscope bias, radians per second.
     */
    Eigen::Vector3d               bg;
    /*!
     * @brief           Prior accelerometer bias, metres per second squared.
     */
    Eigen::Vector3d               ba;
    /*!
     * @brief           Information matrix of the prior, ordered rotation,
     *                  translation, velocity, gyroscope bias, accelerometer
     *                  bias; the eigenvalue clean-up of the constructor has
     *                  been applied.
     */
    Eigen::Matrix<double, 15, 15> H;
};

} /* namespace edge_prior_pose_imu */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_EDGE_PRIOR_POSE_IMU_OBJECTS_CONSTRAINT_POSE_IMU_CLASS_H \
        */
