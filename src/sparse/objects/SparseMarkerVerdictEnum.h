/*!
 * @File:         SparseMarkerVerdictEnum.h
 *
 * @Brief:        TODO
 *
 * @Date:         09/10/2026
 *
 */

#ifndef HPP_SPARSEMARKERVERDICTENUM_HPP
#define HPP_SPARSEMARKERVERDICTENUM_HPP

/* Function Includes */
/* None */

/* Object Include */
/* None */

#include <cstdint>

/* Generic Libraries */
/* None */

namespace vs_graphs::sparse
{

/*!
 * @brief           Typed outcome of the marker-level ingest gate.
 */
enum class SparseMarkerVerdict : std::uint8_t
{
    /*!
     * @brief           The marker carries no free-space geometry and is
     *                  deliberately ignored.
     */
    SPARSE_MARKER_IGNORED = 0U,

    /*!
     * @brief           A free-space cluster marker arrived without any point
     *                  data.
     */
    SPARSE_CLUSTER_EMPTY = 1U,

    /*!
     * @brief           A free-space cluster marker holds fewer raw points than
     *                  required.
     */
    SPARSE_CLUSTER_UNDERSIZED = 2U,

    /*!
     * @brief           A free-space cluster marker is eligible for the
     *                  transform stage.
     */
    SPARSE_CLUSTER_CANDIDATE = 3U,

    /*!
     * @brief           A skeleton edge marker arrived without any point data.
     */
    SPARSE_EDGE_EMPTY = 4U,

    /*!
     * @brief           A skeleton edge marker is eligible for the transform
     *                  stage.
     */
    SPARSE_EDGE_CANDIDATE = 5U
};

} // namespace vs_graphs::sparse

#endif /* HPP_SPARSEMARKERVERDICTENUM_HPP */
