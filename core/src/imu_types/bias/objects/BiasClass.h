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
 * @file            BiasClass.h
 *
 * @brief           Declares the Bias IMU biases for the gyroscope and
 *                  the accelerometer.
 */

#ifndef VS_GRAPHS_CORE_SRC_IMU_TYPES_BIAS_OBJECTS_BIAS_CLASS_H
#define VS_GRAPHS_CORE_SRC_IMU_TYPES_BIAS_OBJECTS_BIAS_CLASS_H

#include <ostream>

#include <Eigen/Core>

#include <boost/serialization/access.hpp>

#include "imu_types/bias/objects/BiasStatusEnum.h"

namespace vs_graphs
{
namespace core
{
namespace imu_types
{
namespace bias
{

/*!
 * @brief        IMU biases for the gyroscope and the accelerometer.
 */
class Bias
{

  public:
    /*!
     * @brief           Creates zero biases.
     */
    Bias() :
        bax(0),
        bay(0),
        baz(0),
        bwx(0),
        bwy(0),
        bwz(0)
    {}
    /*!
     * @brief           Creates biases from raw components.
     *
     * @param[in]       accelerometerBiasX_in
     *                  Accelerometer x bias.
     *
     * @param[in]       accelerometerBiasY_in
     *                  Accelerometer y bias.
     *
     * @param[in]       accelerometerBiasZ_in
     *                  Accelerometer z bias.
     *
     * @param[in]       gyroscopeBiasX_in
     *                  Gyroscope x bias.
     *
     * @param[in]       gyroscopeBiasY_in
     *                  Gyroscope y bias.
     *
     * @param[in]       gyroscopeBiasZ_in
     *                  Gyroscope z bias.
     */
    Bias(const float &accelerometerBiasX_in,
         const float &accelerometerBiasY_in,
         const float &accelerometerBiasZ_in,
         const float &gyroscopeBiasX_in,
         const float &gyroscopeBiasY_in,
         const float &gyroscopeBiasZ_in) :
        bax(accelerometerBiasX_in),
        bay(accelerometerBiasY_in),
        baz(accelerometerBiasZ_in),
        bwx(gyroscopeBiasX_in),
        bwy(gyroscopeBiasY_in),
        bwz(gyroscopeBiasZ_in)
    {}
    /*!
     * @brief           Copies every component from another bias.
     *
     * @param[in]       b_in
     *                  Source bias; shall be non-null.
     */
    [[nodiscard]] BiasStatus copyFrom(Bias &b_in);
    /*!
     * @brief           Appends the six bias components to the
     *                  stream.
     *
     * @param[in,out]   out_inout
     *                  Stream receiving the components.
     *
     * @param[in]       b_in
     *                  Bias whose components are written.
     *
     * @return          The output stream.
     */
    friend std::ostream &operator<<(std::ostream &out_inout, const Bias &b_in);
    /*!
     * @brief           Accelerometer bias along the body x axis, m/s^2.
     */
    float                bax;
    /*!
     * @brief           Accelerometer bias along the body y axis, m/s^2.
     */
    float                bay;
    /*!
     * @brief           Accelerometer bias along the body z axis, m/s^2.
     */
    float                baz;
    /*!
     * @brief           Gyroscope bias about the body x axis, rad/s.
     */
    float                bwx;
    /*!
     * @brief           Gyroscope bias about the body y axis, rad/s.
     */
    float                bwy;
    /*!
     * @brief           Gyroscope bias about the body z axis, rad/s.
     */
    float                bwz;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    friend class boost::serialization::access;
    /*!
     * @brief           Serializes the six bias components.
     *
     * @param[in,out]   ar
     *                  Archive receiving the stored fields.
     *
     * @param[in]       version
     *                  Archive version; currently unused.
     */
    template <class Archive>
    void serialize(Archive &ar, [[maybe_unused]] const unsigned int version);
};

} /* namespace bias */
} /* namespace imu_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_IMU_TYPES_BIAS_OBJECTS_BIAS_CLASS_H */
