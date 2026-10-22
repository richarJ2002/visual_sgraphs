/*!
 * @file            PointClass.h
 * @brief           Declares one timestamped IMU sample.
 *
 * Copyright (C) 2014-2021 University of Zaragoza, ORB-SLAM3 authors.
 * This file is part of ORB-SLAM3, distributed under GPL-3.0-or-later.
 */
#ifndef VS_GRAPHS_CORE_IMU_TYPES_POINT_CLASS_H
#define VS_GRAPHS_CORE_IMU_TYPES_POINT_CLASS_H

#include <Eigen/Core>
#include <opencv2/core/types.hpp>

namespace vs_graphs::core::imu_types::point
{
/*!
 * @brief           Value sample; acceleration and angular rate are sensor-frame
 *                  readings. Timestamp is in seconds on the sensor clock.
 */
class Point
{
  public:
    /*!
     * @brief           Creates a sample from raw components.
     *
     * @param[in]       accelerometerX_in
     *                  X acceleration, metres per second squared.
     * @param[in]       accelerometerY_in
     *                  Y acceleration, metres per second squared.
     * @param[in]       accelerometerZ_in
     *                  Z acceleration, metres per second squared.
     * @param[in]       gyroscopeX_in
     *                  X angular rate, radians per second.
     * @param[in]       gyroscopeY_in
     *                  Y angular rate, radians per second.
     * @param[in]       gyroscopeZ_in
     *                  Z angular rate, radians per second.
     * @param[in]       timestamp_in
     *                  Sample timestamp, seconds.
     */
    Point(const float &accelerometerX_in,
          const float &accelerometerY_in,
          const float &accelerometerZ_in,
          const float &gyroscopeX_in,
          const float &gyroscopeY_in,
          const float &gyroscopeZ_in,
          const double &timestamp_in) :
        a(accelerometerX_in, accelerometerY_in, accelerometerZ_in),
        w(gyroscopeX_in, gyroscopeY_in, gyroscopeZ_in),
        t(timestamp_in)
    {
    }
    /*!
     * @brief           Creates a sample from OpenCV vectors.
     *
     * @param[in]       accelerometer_in
     *                  Sensor-frame acceleration, metres per second squared.
     * @param[in]       gyroscope_in
     *                  Sensor-frame angular rate, radians per second.
     * @param[in]       timestamp_in
     *                  Sample timestamp, seconds.
     */
    Point(const cv::Point3f accelerometer_in,
          const cv::Point3f gyroscope_in,
          const double &timestamp_in) :
        a(accelerometer_in.x, accelerometer_in.y, accelerometer_in.z),
        w(gyroscope_in.x, gyroscope_in.y, gyroscope_in.z),
        t(timestamp_in)
    {
    }
    /*! @brief Accelerometer sample, metres per second squared. */
    Eigen::Vector3f a;
    /*! @brief Gyroscope sample, radians per second. */
    Eigen::Vector3f w;
    /*! @brief Sample timestamp, seconds. */
    double t;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} /* namespace vs_graphs::core::imu_types::point */
#endif /* VS_GRAPHS_CORE_IMU_TYPES_POINT_CLASS_H */
