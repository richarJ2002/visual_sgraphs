/*!
 * @file            OptimizerEdgeLookupStatusEnum.h
 *
 * @brief           Declares the status returned by the optimizer's
 *                  edge-keyframe lookup helper.
 */

#ifndef VS_GRAPHS_CORE_SRC_OPTIMIZER_OBJECTS_OPTIMIZER_EDGE_LOOKUP_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_OPTIMIZER_OBJECTS_OPTIMIZER_EDGE_LOOKUP_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace optimizer
{

/*!
 * @brief           Result of an optimizer edge-keyframe lookup operation.
 *                  Values are fixed and never reordered.
 */
enum class OptimizerEdgeLookupStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    OPTIMIZER_EDGE_LOOKUP_STATUS_SUCCESS = 0U
};

} /* namespace optimizer */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_OPTIMIZER_OBJECTS_OPTIMIZER_EDGE_LOOKUP_STATUS_ENUM_H */
