/*!
 * @file            IntegratedRotationClass.h
 * @brief           Declares the historical single-sample SO(3) integration.
 *
 * Copyright (C) 2014-2021 University of Zaragoza, ORB-SLAM3 authors.
 * This file is part of ORB-SLAM3, distributed under GPL-3.0-or-later.
 */
#ifndef VS_GRAPHS_CORE_IMU_TYPES_INTEGRATED_ROTATION_CLASS_H
#define VS_GRAPHS_CORE_IMU_TYPES_INTEGRATED_ROTATION_CLASS_H

#include <cmath>

#include <Eigen/Core>
#include "dependency_lineage/sophus/objects/so3.hpp"

#include "sensor_models/imu_types/bias/bias.h"
#include "sensor_models/imu_types/preintegrated/objects/ImuConstants.h"

namespace vs_graphs::core::imu_types::integrated_rotation
{
/*!
 * @brief           Rotation increment and right Jacobian of a bias-corrected
 *                  body-frame angular-rate sample; owns no external resources.
 */
class IntegratedRotation
{
  public:
    /*! @brief Creates an empty increment, leaving coefficients unset. */
    IntegratedRotation() {}
    /*!
     * @brief           Integrates one rate using the historical small-angle
     *                  branch. deltaT remains unset, as in the original type;
     *                  callers use their own integration interval.
     *
     * @param[in]       angularVelocity_in
     *                  Body-frame angular rate, radians per second.
     * @param[in]       imuBias_in
     *                  Gyroscope bias to subtract, radians per second.
     * @param[in]       integrationInterval_in
     *                  Integration interval, seconds.
     */
    IntegratedRotation(const Eigen::Vector3f &angularVelocity_in,
                       const bias::Bias &imuBias_in,
                       const float &integrationInterval_in)
    {
        const float rotationVectorX =
            (angularVelocity_in(0) - imuBias_in.bwx) * integrationInterval_in;
        const float rotationVectorY =
            (angularVelocity_in(1) - imuBias_in.bwy) * integrationInterval_in;
        const float rotationVectorZ =
            (angularVelocity_in(2) - imuBias_in.bwz) * integrationInterval_in;
        const float rotationAngleSquared = rotationVectorX * rotationVectorX +
                                           rotationVectorY * rotationVectorY +
                                           rotationVectorZ * rotationVectorZ;
        const float rotationAngle = std::sqrt(rotationAngleSquared);
        Eigen::Vector3f rotationVector;
        rotationVector << rotationVectorX, rotationVectorY, rotationVectorZ;
        Eigen::Matrix3f skewMatrix = Sophus::SO3f::hat(rotationVector);
        if (rotationAngle < preintegrated::eps)
        {
            deltaR = Eigen::Matrix3f::Identity() + skewMatrix;
            rightJ = Eigen::Matrix3f::Identity();
        }
        else
        {
            deltaR = Eigen::Matrix3f::Identity() +
                     skewMatrix * std::sin(rotationAngle) / rotationAngle +
                     skewMatrix * skewMatrix *
                         (1.0f - std::cos(rotationAngle)) /
                         rotationAngleSquared;
            rightJ = Eigen::Matrix3f::Identity() -
                     skewMatrix * (1.0f - std::cos(rotationAngle)) /
                         rotationAngleSquared +
                     skewMatrix * skewMatrix *
                         (rotationAngle - std::sin(rotationAngle)) /
                         (rotationAngleSquared * rotationAngle);
        }
    }
    /*! @brief Historical interval slot, seconds; not assigned by constructors. */
    float deltaT;
    /*! @brief Integrated body-frame rotation increment. */
    Eigen::Matrix3f deltaR;
    /*! @brief Right Jacobian of the body-frame rotation increment. */
    Eigen::Matrix3f rightJ;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} /* namespace vs_graphs::core::imu_types::integrated_rotation */
#endif /* VS_GRAPHS_CORE_IMU_TYPES_INTEGRATED_ROTATION_CLASS_H */
