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
 * @file            PreintegratedIntegrableStruct.h
 *
 * @brief           Declares the Preintegrated Integrable stored reading.
 */

#ifndef VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_PREINTEGRATED_INTEGRABLE_STRUCT_H
#define VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_PREINTEGRATED_INTEGRABLE_STRUCT_H

#include <Eigen/Core>

namespace vs_graphs
{
namespace core
{
namespace imu_types
{
namespace preintegrated
{

/*!
 * @brief           One stored accelerometer and gyroscope
 *                  reading.
 */
struct Integrable
{
    /*!
     * @brief           Serializes the stored reading.
     *
     * @param[in,out]   ar
     *                  Archive receiving the stored fields.
     *
     * @param[in]       version
     *                  Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*!
     * @brief           Creates an empty stored reading.
     */
    Integrable() {}
    /*!
     * @brief           Creates a stored reading.
     *
     * @param[in]       acceleration_in
     *                  Accelerometer sample in metres per
     *                  second squared.
     *
     * @param[in]       angularVelocity_in
     *                  Gyroscope sample in radians per
     *                  second.
     *
     * @param[in]       sampleInterval_in
     *                  Interval since the previous sample,
     *                  in seconds.
     */
    Integrable(const Eigen::Vector3f &acceleration_in,
               const Eigen::Vector3f &angularVelocity_in,
               const float           &sampleInterval_in) :
        a(acceleration_in),
        w(angularVelocity_in),
        t(sampleInterval_in)
    {}
    /*!
     * @brief           Stored accelerometer sample, m/s^2.
     */
    Eigen::Vector3f a;
    /*!
     * @brief           Stored gyroscope sample, rad/s.
     */
    Eigen::Vector3f w;
    /*!
     * @brief           Interval since the previous sample,
     *                  in seconds.
     */
    float           t;
};

} /* namespace preintegrated */
} /* namespace imu_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_IMU_TYPES_PREINTEGRATED_OBJECTS_PREINTEGRATED_INTEGRABLE_STRUCT_H \
        */
