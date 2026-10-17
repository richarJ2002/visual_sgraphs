/*!
 * @file            GravityDirectionStatusEnum.h
 *
 * @brief           Declares the GravityDirectionStatus result of a gravity
 *                  direction update.
 */

#ifndef VS_GRAPHS_CORE_SRC_G2O_TYPES_GRAVITY_DIRECTION_OBJECTS_GRAVITY_DIRECTION_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_G2O_TYPES_GRAVITY_DIRECTION_OBJECTS_GRAVITY_DIRECTION_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace g2o_types
{
namespace gravity_direction
{

/*!
 * @brief           Result of a gravity direction update. Values are fixed and
 *                  never reordered.
 */
enum class GravityDirectionStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    GRAVITY_DIRECTION_STATUS_SUCCESS = 0U
};

} /* namespace gravity_direction */
} /* namespace g2o_types */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_G2O_TYPES_GRAVITY_DIRECTION_OBJECTS_GRAVITY_DIRECTION_STATUS_ENUM_H \
        */
