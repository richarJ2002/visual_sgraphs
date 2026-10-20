/*!
 * @file            SensorTypeEnum.h
 *
 * @brief           Declares the stable sensor discriminator shared by System,
 *                  Tracking and Settings without a circular class dependency.
 */

#ifndef VS_GRAPHS_CORE_SRC_SYSTEM_OBJECTS_SENSOR_TYPE_ENUM_H
#define VS_GRAPHS_CORE_SRC_SYSTEM_OBJECTS_SENSOR_TYPE_ENUM_H

namespace vs_graphs::core::system
{

/*!
 * @brief           Sensor configuration; integer values remain compatible with
 *                  the existing integer-taking Tracking and Settings APIs.
 *                  The unscoped form preserves that external API conversion.
 */
enum SensorType : int
{
    /*! @brief           Invalid, unset sensor configuration. */
    NOT_SET = -1,
    /*! @brief           Monocular visual camera. */
    MONOCULAR = 0,
    /*! @brief           Stereo visual cameras. */
    STEREO = 1,
    /*! @brief           RGB-D visual camera. */
    RGBD = 2,
    /*! @brief           Monocular camera with an IMU. */
    IMU_MONOCULAR = 3,
    /*! @brief           Stereo cameras with an IMU. */
    IMU_STEREO = 4,
    /*! @brief           RGB-D camera with an IMU. */
    IMU_RGBD = 5
};

} /* namespace vs_graphs::core::system */

#endif /* VS_GRAPHS_CORE_SRC_SYSTEM_OBJECTS_SENSOR_TYPE_ENUM_H */
