/*!
 * @file            classifySparseMarker.cc
 *
 * @brief           Implements sparse-graph marker classification.
 */

/* Matching Declaration Include */
#include "SparseClusterVerdict.h"

namespace vs_graphs::sparse
{

SparseMarkerVerdict
    classifySparseMarker(const int          markerType_in,
                         const std::string &markerNamespace_in,
                         const std::size_t  pointCount_in,
                         const std::size_t  minimumVertexCount_in)
{
    const bool isClusterMarker =
        markerType_in == SPARSE_CUBE_LIST_TYPE &&
        markerNamespace_in.rfind(SPARSE_CLUSTER_NAMESPACE_PREFIX, 0) == 0;
    const bool isEdgeMarker = markerType_in == SPARSE_LINE_LIST_TYPE &&
                              markerNamespace_in == SPARSE_EDGE_NAMESPACE;

    /* Markers outside the free-space contract are ignored, never rejected. */
    if (!isClusterMarker && !isEdgeMarker)
    {
        return SparseMarkerVerdict::SPARSE_MARKER_IGNORED;
    }

    if (pointCount_in == 0U)
    {
        return isClusterMarker ? SparseMarkerVerdict::SPARSE_CLUSTER_EMPTY
                               : SparseMarkerVerdict::SPARSE_EDGE_EMPTY;
    }

    if (isClusterMarker && pointCount_in < minimumVertexCount_in)
    {
        return SparseMarkerVerdict::SPARSE_CLUSTER_UNDERSIZED;
    }

    return isClusterMarker ? SparseMarkerVerdict::SPARSE_CLUSTER_CANDIDATE
                           : SparseMarkerVerdict::SPARSE_EDGE_CANDIDATE;
}

} /* namespace vs_graphs::sparse */
