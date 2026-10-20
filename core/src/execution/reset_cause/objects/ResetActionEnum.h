/*!
 * @file            ResetActionEnum.h
 *
 * @brief           Declares the action requested or performed at an attributed
 *                  call site.
 */

#ifndef VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_ACTION_ENUM_H
#define VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_ACTION_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace reset_cause
{

/*!
 * @brief           Action requested or performed at an attributed call site.
 */
enum class ResetAction : std::uint8_t
{
    /*!
     * @brief           Attribution point at the reset request.
     */
    RESET_ACTIVE_MAP_REQUEST = 0U,
    /*!
     * @brief           Attribution point at the reset execution.
     */
    RESET_ACTIVE_MAP_EXECUTION = 1U,
    /*!
     * @brief           Attribution point at the new-map execution.
     */
    CREATE_MAP_EXECUTION = 2U
};

} /* namespace reset_cause */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_ACTION_ENUM_H */
