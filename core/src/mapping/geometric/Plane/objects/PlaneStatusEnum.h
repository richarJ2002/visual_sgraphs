/*!
 * @file            PlaneStatusEnum.h
 *
 * @brief           Declares the status returned by every Plane operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_OBJECTS_PLANE_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_OBJECTS_PLANE_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

/*!
 * @brief           Result of a Plane operation. Values are fixed and never
 *                  reordered.
 */
enum class PlaneStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    PLANE_STATUS_SUCCESS = 0U
};

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_GEOMETRIC_PLANE_OBJECTS_PLANE_STATUS_ENUM_H */
