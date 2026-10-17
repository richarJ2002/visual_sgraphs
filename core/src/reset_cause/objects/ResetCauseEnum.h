/*!
 * @file            ResetCauseEnum.h
 *
 * @brief           Declares stable internal reset/new-map attribution values.
 */

#ifndef VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_ENUM_H
#define VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace reset_cause
{

/*!
 * @brief           Internal causes observable at reset and new-map call sites.
 *
 *                  The public System::Reset() and System::ResetActiveMap() API
 *                  has unknown external callers and retains no cause while
 *                  deferring work to the next tracking call. Those execution
 *                  points therefore remain explicitly unknown; known
 *                  package-owned callers report the exact request here instead
 *                  of changing System or Tracking object layout.
 */
enum class ResetCause : std::uint8_t
{
    /*!
     * @brief           Public reset with no attributed package cause.
     */
    UNATTRIBUTED_PUBLIC_REQUEST = 0U,
    /*!
     * @brief           Differing causes merged before one execution.
     */
    MULTIPLE_COALESCED_REQUESTS = 1U,
    /*!
     * @brief           Local mapper reported invalid inertial data.
     */
    LOCAL_MAPPER_BAD_IMU = 2U,
    /*!
     * @brief           Sensor timestamp moved backwards in time.
     */
    NON_MONOTONIC_SENSOR_TIMESTAMP = 3U,
    /*!
     * @brief           Timestamp jump before IMU initialization.
     */
    TIMESTAMP_JUMP_BEFORE_IMU_INITIALIZATION = 4U,
    /*!
     * @brief           Timestamp jump before the second IMU bundle adjustment.
     */
    TIMESTAMP_JUMP_BEFORE_SECOND_IMU_BA = 5U,
    /*!
     * @brief           Timestamp jump after the second IMU bundle adjustment.
     */
    TIMESTAMP_JUMP_AFTER_SECOND_IMU_BA = 6U,
    /*!
     * @brief           Visual tracking lost while the map is small.
     */
    VISUAL_TRACKING_LOST_SMALL_MAP = 7U,
    /*!
     * @brief           Visual tracking lost before IMU initialization.
     */
    VISUAL_TRACKING_LOST_BEFORE_IMU_INITIALIZATION = 8U,
    /*!
     * @brief           Visual tracking lost; a new map is created.
     */
    VISUAL_TRACKING_LOST_NEW_MAP = 9U,
    /*!
     * @brief           Initialization lacked enough map points.
     */
    INITIALIZATION_INSUFFICIENT_POINTS = 10U,
    /*!
     * @brief           Monocular initialization produced an invalid map.
     */
    INITIALIZATION_INVALID_MONOCULAR_MAP = 11U,
    /*!
     * @brief           Gap in the delivered IMU stream.
     */
    IMU_DELIVERY_GAP = 12U,
    /*!
     * @brief           Sensor processing could not keep up.
     */
    SENSOR_PROCESSING_OVERLOAD = 13U,
    /*!
     * @brief           Reset requested from the viewer.
     */
    VIEWER_REQUEST = 14U,
    /*!
     * @brief           Dataset changed while the map is small.
     */
    DATASET_CHANGE_SMALL_MAP = 15U,
    /*!
     * @brief           Dataset changed; a new map is created.
     */
    DATASET_CHANGE_NEW_MAP = 16U
};

} /* namespace reset_cause */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_ENUM_H */
