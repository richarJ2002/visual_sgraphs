/*!
 * @file            GeometricToolsStatusEnum.h
 *
 * @brief           Declares the status returned by every GeometricTools
 *                  operation.
 */

#ifndef VS_GRAPHS_CORE_SRC_GEOMETRIC_GEOMETRICTOOLS_OBJECTS_GEOMETRICTOOLS_STATUS_ENUM_H
#define VS_GRAPHS_CORE_SRC_GEOMETRIC_GEOMETRICTOOLS_OBJECTS_GEOMETRICTOOLS_STATUS_ENUM_H

#include <cstdint>

namespace vs_graphs
{
namespace core
{
namespace geometric
{

/*!
 * @brief           Result of a GeometricTools operation. Values are fixed and
 *                  never reordered.
 */
enum class GeometricToolsStatus : std::uint8_t
{
    /*!
     * @brief           The operation completed and every output was written.
     */
    GEOMETRIC_TOOLS_STATUS_SUCCESS = 0U,

    /*!
     * @brief           A numerical step did not converge or produced a
     *                  non-finite value.
     */
    GEOMETRIC_TOOLS_STATUS_NUMERICAL_FAILURE = 1U
};

} /* namespace geometric */
} /* namespace core */
} /* namespace vs_graphs */

#endif /* VS_GRAPHS_CORE_SRC_GEOMETRIC_GEOMETRICTOOLS_OBJECTS_GEOMETRICTOOLS_STATUS_ENUM_H \
        */
