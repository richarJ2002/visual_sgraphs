/*!
 * @File:         PublicFunctions.h
 *
 * @Brief:        TODO
 *
 * @Date:         09/10/2026
 *
 */

#ifndef VS_GRAPHS_SPARSE_PUBLIC_FUNCTIONS_H
#define VS_GRAPHS_SPARSE_PUBLIC_FUNCTIONS_H

/* Function Includes */
/* None */

#include "sparse/objects/SparseMarkerVerdictEnum.h"
#include <cstddef>
#include <string>

/* Data include */
/* None */

/* Generic Libraries */
/* None */

namespace vs_graphs::sparse
{

/*!
 * @brief           Classifies one sparse-graph marker ahead of transformation.
 *
 * @param[in]       markerType_in
 *                  Integer marker type code (SPARSE_CUBE_LIST_TYPE or
 *                  SPARSE_LINE_LIST_TYPE for eligible geometry).
 *
 * @param[in]       markerNamespace_in
 *                  Marker namespace naming the free-space component.
 *
 * @param[in]       pointCount_in
 *                  Number of raw points carried by the marker.
 *
 * @param[in]       minimumVertexCount_in
 *                  Minimum raw points required of a cluster marker.
 *
 * @return          Typed verdict describing the ingest gate outcome.
 *
 * @pre             minimumVertexCount_in is at least one.
 */
[[nodiscard]] SparseMarkerVerdict
    classifySparseMarker(const int          markerType_in,
                         const std::string &markerNamespace_in,
                         const std::size_t  pointCount_in,
                         const std::size_t  minimumVertexCount_in);

} // namespace vs_graphs::sparse

#endif /* VS_GRAPHS_SPARSE_PUBLIC_FUNCTIONS_H */
