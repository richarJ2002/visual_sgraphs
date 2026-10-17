/*!
 * @file            ResetCauseStatusEnum.h
 *
 * @brief           Declares the status returned by every ResetCause operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace reset_cause
{

/*!
 * @brief           Result of a ResetCause operation. Values are fixed and never
 *                  reordered.
 */
enum class ResetCauseStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    RESET_CAUSE_STATUS_SUCCESS = 0U
};

} /* namespace reset_cause */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_RESET_CAUSE_OBJECTS_RESET_CAUSE_STATUS_ENUM_H */
