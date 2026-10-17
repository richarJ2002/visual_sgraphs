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
 * @file            CalibClass.h
 *
 * @brief           Declares the Calib IMU calibration holding transforms
 *                  and noise models.
 */

#ifndef VS_GRAPHS_CORE_SRC_IMU_TYPES_CALIB_OBJECTS_CALIB_CLASS_H
#define VS_GRAPHS_CORE_SRC_IMU_TYPES_CALIB_OBJECTS_CALIB_CLASS_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include "sophus/objects/se3.hpp"

#include <boost/serialization/access.hpp>

#include <rclcpp/logging.hpp>

#include "imu_types/calib/objects/CalibStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace imu_types
{
namespace calib
{

/*!
 * @brief        IMU calibration holding transforms and noise models.
 */
class Calib
{

  public:
    /*!
     * @brief           Creates a calibration and marks it valid.
     *
     * @param[in]       extrinsicPose_cameraToBody_in
     *                  Camera-to-body (IMU) transform.
     *
     * @param[in]       gyroscopeNoiseDensity_in
     *                  Gyroscope noise density.
     *
     * @param[in]       accelerometerNoiseDensity_in
     *                  Accelerometer noise density.
     *
     * @param[in]       gyroscopeRandomWalkDensity_in
     *                  Gyroscope random-walk density.
     *
     * @param[in]       accelerometerRandomWalkDensity_in
     *                  Accelerometer random-walk density.
     */
    Calib(const Sophus::SE3<float> &extrinsicPose_cameraToBody_in,
          const float              &gyroscopeNoiseDensity_in,
          const float              &accelerometerNoiseDensity_in,
          const float              &gyroscopeRandomWalkDensity_in,
          const float              &accelerometerRandomWalkDensity_in)
    {
        if (setCalibration(extrinsicPose_cameraToBody_in,
                           gyroscopeNoiseDensity_in,
                           accelerometerNoiseDensity_in,
                           gyroscopeRandomWalkDensity_in,
                           accelerometerRandomWalkDensity_in) !=
            CalibStatus::CALIB_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: setCalibration returned a failure status "
                         "although it cannot fail; continuing as before.",
                         __func__);
        }
    }
    /*!
     * @brief           Creates an unset calibration.
     */
    Calib()
    {
        isCalibrationSet = false;
    }
    /*!
     * @brief           Stores the transform and noise densities
     *                  and marks the calibration valid.
     *
     * @param[in]       extrinsicPose_cameraToBody_in
     *                  Camera-to-body (IMU) transform.
     *
     * @param[in]       ng_in
     *                  Gyroscope noise density.
     *
     * @param[in]       na_in
     *                  Accelerometer noise density.
     *
     * @param[in]       ngw_in
     *                  Gyroscope random-walk density.
     *
     * @param[in]       naw_in
     *                  Accelerometer random-walk density.
     */
    [[nodiscard]] CalibStatus
        setCalibration(const Sophus::SE3<float> &extrinsicPose_cameraToBody_in,
                       const float              &ng_in,
                       const float              &na_in,
                       const float              &ngw_in,
                       const float              &naw_in);
    /* Sophus/Eigen implementation */
    /*!
     * @brief           Camera-to-body transform.
     */
    Sophus::SE3<float>              mTcb;
    /*!
     * @brief           Body-to-camera transform.
     */
    Sophus::SE3<float>              mTbc;
    /*!
     * @brief           Gyroscope then accelerometer measurement noise variances
     *                  (diagonal), set by setCalibration.
     */
    Eigen::DiagonalMatrix<float, 6> Cov;
    /*!
     * @brief           Gyroscope then accelerometer bias random-walk variances
     *                  (diagonal), set by setCalibration.
     */
    Eigen::DiagonalMatrix<float, 6> CovWalk;
    /*!
     * @brief           True once Set has stored a calibration.
     */
    bool                            isCalibrationSet;

  private:
    friend class boost::serialization::access;
    /*!
     * @brief           Serializes the transforms, covariances
     *                  and validity flag.
     *
     * @param[in,out]   ar
     *                  Archive receiving the stored fields.
     *
     * @param[in]       version
     *                  Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version);
};

} /* namespace calib */
} /* namespace imu_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_IMU_TYPES_CALIB_OBJECTS_CALIB_CLASS_H */
