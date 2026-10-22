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

/*!
 * @file            PreintegratedClass.h
 *
 * @brief           Declares the Preintegrated IMU measurements between
 *                  keyframes.
 */

#ifndef VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_PREINTEGRATED_CLASS_H
#define VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_PREINTEGRATED_CLASS_H

#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <boost/serialization/access.hpp>

#include <rclcpp/logging.hpp>

#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/calib/calib.h"

#include "sensor_models/imu_types/preintegrated/objects/PreintegratedIntegrableStruct.h"
#include "sensor_models/imu_types/preintegrated/objects/PreintegratedStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace imu_types
{
namespace preintegrated
{

/*!
 * @brief        Preintegration of IMU measurements between keyframes.
 */
class Preintegrated
{

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates a preintegration from a bias and
     *                  a calibration.
     *
     * @param[in]       bias_in
     *                  bias::Bias integrated against.
     *
     * @param[in]       calib_in
     *                  Calibration supplying the noise models.
     */
    Preintegrated(const bias::Bias &bias_in, const calib::Calib &calib_in)
    {
        Nga     = calib_in.Cov;
        NgaWalk = calib_in.CovWalk;
        if (initialize(bias_in) !=
            PreintegratedStatus::PREINTEGRATED_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: initialize returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
    }
    /*!
     * @brief           Copies another preintegration.
     *
     * @param[in]       p_sourcePreintegration_in
     *                  Non-owning source; shall be non-null.
     */
    /* Copy constructor */
    Preintegrated(Preintegrated *p_sourcePreintegration_in) :
        dT(p_sourcePreintegration_in->dT),
        C(p_sourcePreintegration_in->C),
        Info(p_sourcePreintegration_in->Info),
        Nga(p_sourcePreintegration_in->Nga),
        NgaWalk(p_sourcePreintegration_in->NgaWalk),
        b(p_sourcePreintegration_in->b),
        dR(p_sourcePreintegration_in->dR),
        dV(p_sourcePreintegration_in->dV),
        dP(p_sourcePreintegration_in->dP),
        JRg(p_sourcePreintegration_in->JRg),
        JVg(p_sourcePreintegration_in->JVg),
        JVa(p_sourcePreintegration_in->JVa),
        JPg(p_sourcePreintegration_in->JPg),
        JPa(p_sourcePreintegration_in->JPa),
        avgA(p_sourcePreintegration_in->avgA),
        avgW(p_sourcePreintegration_in->avgW),
        bu(p_sourcePreintegration_in->bu),
        db(p_sourcePreintegration_in->db),
        measurements(p_sourcePreintegration_in->measurements)
    {}
    /*!
     * @brief           Creates an empty preintegration.
     */
    Preintegrated() {}
    /*!
     * @brief           Copies the state of another
     *                  preintegration.
     *
     * @param[in]       p_sourcePreintegrated_in
     *                  Non-owning source; shall be non-null.
     */
    [[nodiscard]] PreintegratedStatus
        copyFrom(Preintegrated *p_sourcePreintegrated_in);
    /*!
     * @brief           Resets the deltas for a new bias.
     *
     * @param[in]       referenceBias_in
     *                  bias::Bias integrated against.
     */
    [[nodiscard]] PreintegratedStatus
        initialize(const bias::Bias &referenceBias_in);
    /*!
     * @brief           Folds one measurement into the deltas.
     *
     * @param[in]       acceleration_in
     *                  Accelerometer sample in metres per second
     *                  squared.
     *
     * @param[in]       angularVelocity_in
     *                  Gyroscope sample in radians per second.
     *
     * @param[in]       deltaTime_in
     *                  Interval since the previous sample, in
     *                  seconds.
     */
    [[nodiscard]] PreintegratedStatus
        integrateNewMeasurement(const Eigen::Vector3f &acceleration_in,
                                const Eigen::Vector3f &angularVelocity_in,
                                const float           &deltaTime_in);
    /*!
     * @brief           Rebuilds the deltas from the stored
     *                   measurements under the updated bias.
     *
     *                  Takes the preintegration lock.
     */
    [[nodiscard]] PreintegratedStatus reintegrate();
    /*!
     * @brief           Prepends the state of a previous
     *                   preintegration.
     *
     *                  Takes both preintegration locks; a
     *                  self-merge is ignored.
     *
     * @param[in]       p_previousPreintegrated_in
     *                  Non-owning previous preintegration; shall
     *                  be non-null.
     */
    [[nodiscard]] PreintegratedStatus
        mergePrevious(Preintegrated *p_previousPreintegrated_in);
    /*!
     * @brief           Stores the updated bias and refreshes the
     *                   bias difference.
     *
     *                  Takes the preintegration lock.
     *
     * @param[in]       updatedBias_in
     *                  Updated bias estimate.
     */
    [[nodiscard]] PreintegratedStatus
        setNewBias(const bias::Bias &updatedBias_in);
    /*!
     * @brief           Returns the bias change relative to the
     *                   original bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[in]       referenceBias_in
     *                  bias::Bias to compare against.
     *
     * @param[out]      deltaBias_out
     *                  Difference between the given bias and the original bias.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getDeltaBias(const bias::Bias      &referenceBias_in,
                     imu_types::bias::Bias &deltaBias_out);
    /*!
     * @brief           Returns the delta rotation corrected for
     *                   the given bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[in]       referenceBias_in
     *                  bias::Bias to correct for.
     *
     * @param[out]      deltaRotation_out
     *                  bias::Bias-corrected delta rotation.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getDeltaRotation(const bias::Bias &referenceBias_in,
                         Eigen::Matrix3f  &deltaRotation_out);
    /*!
     * @brief           Returns the delta velocity corrected for
     *                   the given bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[in]       referenceBias_in
     *                  bias::Bias to correct for.
     *
     * @param[out]      deltaVelocity_out
     *                  bias::Bias-corrected delta velocity in metres per
     * second.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getDeltaVelocity(const bias::Bias &referenceBias_in,
                         Eigen::Vector3f  &deltaVelocity_out);
    /*!
     * @brief           Returns the delta position corrected for
     *                   the given bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[in]       referenceBias_in
     *                  bias::Bias to correct for.
     *
     * @param[out]      deltaPosition_out
     *                  bias::Bias-corrected delta position in metres.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getDeltaPosition(const bias::Bias &referenceBias_in,
                         Eigen::Vector3f  &deltaPosition_out);
    /*!
     * @brief           Returns the delta rotation under the
     *                   updated bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      updatedDeltaRotation_out
     *                  Updated delta rotation.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getUpdatedDeltaRotation(Eigen::Matrix3f &updatedDeltaRotation_out);
    /*!
     * @brief           Returns the delta velocity under the
     *                   updated bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      updatedDeltaVelocity_out
     *                  Updated delta velocity in metres per second.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getUpdatedDeltaVelocity(Eigen::Vector3f &updatedDeltaVelocity_out);
    /*!
     * @brief           Returns the delta position under the
     *                   updated bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      updatedDeltaPosition_out
     *                  Updated delta position in metres.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getUpdatedDeltaPosition(Eigen::Vector3f &updatedDeltaPosition_out);
    /*!
     * @brief           Returns the delta rotation under the
     *                   original bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      originalDeltaRotation_out
     *                  Original delta rotation.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getOriginalDeltaRotation(Eigen::Matrix3f &originalDeltaRotation_out);
    /*!
     * @brief           Returns the delta velocity under the
     *                   original bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      originalDeltaVelocity_out
     *                  Original delta velocity in metres per second.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getOriginalDeltaVelocity(Eigen::Vector3f &originalDeltaVelocity_out);
    /*!
     * @brief           Returns the delta position under the
     *                   original bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      originalDeltaPosition_out
     *                  Original delta position in metres.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getOriginalDeltaPosition(Eigen::Vector3f &originalDeltaPosition_out);
    /*!
     * @brief           Returns the stored bias difference.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      deltaBias_out
     *                  Six-element difference between the updated and original
     *                  biases.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getDeltaBias(Eigen::Matrix<float, 6, 1> &deltaBias_out);
    /*!
     * @brief           Returns the original integration bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      originalBias_out
     *                  bias::Bias integrated against.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getOriginalBias(bias::Bias &originalBias_out);
    /*!
     * @brief           Returns the latest updated bias.
     *
     *                  Takes the preintegration lock.
     *
     * @param[out]      updatedBias_out
     *                  Updated bias estimate.
     *
     * @return          PREINTEGRATED_STATUS_SUCCESS.
     */
    [[nodiscard]] PreintegratedStatus
        getUpdatedBias(bias::Bias &updatedBias_out);
    /*!
     * @brief           Prints the stored measurement timestamps
     *                  to standard output.
     */
    [[nodiscard]] PreintegratedStatus printMeasurements() const;
    /*!
     * @brief           Accumulated integration time in seconds.
     */
    float                             dT;
    /*!
     * @brief           Fifteen-by-fifteen covariance.
     */
    Eigen::Matrix<float, 15, 15>      C;
    /*!
     * @brief           Fifteen-by-fifteen information matrix.
     */
    Eigen::Matrix<float, 15, 15>      Info;
    /*!
     * @brief           Gyroscope then accelerometer measurement noise variances
     *                  (diagonal) used to grow the covariance.
     */
    Eigen::DiagonalMatrix<float, 6>   Nga;
    /*!
     * @brief           Gyroscope then accelerometer bias random-walk variances
     *                  (diagonal) added to the bias block of the covariance at
     *                  every measurement.
     */
    Eigen::DiagonalMatrix<float, 6>   NgaWalk;
    /* Values for the original bias (when integration was computed) */
    /*!
     * @brief           bias::Bias the deltas were integrated against.
     */
    bias::Bias                        b;
    /*!
     * @brief           Integrated rotation increment.
     */
    Eigen::Matrix3f                   dR;
    /*!
     * @brief           Integrated velocity increment in the body frame at the
     *                  start of the integration, m/s.
     */
    Eigen::Vector3f                   dV;
    /*!
     * @brief           Integrated position increment in the body frame at the
     *                  start of the integration, metres.
     */
    Eigen::Vector3f                   dP;
    /*!
     * @brief           Jacobian of the rotation increment with respect to the
     *                  gyroscope bias.
     */
    Eigen::Matrix3f                   JRg;
    /*!
     * @brief           Jacobian of the velocity increment with respect to the
     *                  gyroscope bias.
     */
    Eigen::Matrix3f                   JVg;
    /*!
     * @brief           Jacobian of the velocity increment with respect to the
     *                  accelerometer bias.
     */
    Eigen::Matrix3f                   JVa;
    /*!
     * @brief           Jacobian of the position increment with respect to the
     *                  gyroscope bias.
     */
    Eigen::Matrix3f                   JPg;
    /*!
     * @brief           Jacobian of the position increment with respect to the
     *                  accelerometer bias.
     */
    Eigen::Matrix3f                   JPa;
    /*!
     * @brief           Time-weighted mean of the bias-corrected acceleration,
     *                  rotated by the integrated rotation, m/s^2.
     */
    Eigen::Vector3f                   avgA;
    /*!
     * @brief           Time-weighted mean of the bias-corrected angular
     *                  velocity, rad/s.
     */
    Eigen::Vector3f                   avgW;

  private:
    friend class boost::serialization::access;
    /*!
     * @brief           Serializes the preintegrated state and
     *                  the stored measurements.
     *
     * @param[in,out]   ar
     *                  Archive receiving the stored fields.
     *
     * @param[in]       version
     *                  Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version);
    /* Updated bias */
    /*!
     * @brief           Latest updated bias estimate.
     */
    bias::Bias                 bu;
    /*!
     * @brief           Difference between the updated and the
     *                  original bias, used to compute the updated values
     *                  of the preintegration.
     */
    Eigen::Matrix<float, 6, 1> db;
    /*!
     * @brief           Stored readings backing reintegration.
     */
    std::vector<Integrable>    measurements;
    /*!
     * @brief           Serializes concurrent preintegration
     *                  updates.
     */
    std::mutex                 preintegrationMutex;
};

} /* namespace preintegrated */
} /* namespace imu_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_PREINTEGRATED_CLASS_H \
        */
