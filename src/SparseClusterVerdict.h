/*!
 * @file SparseClusterVerdict.h
 * @brief Declares dependency-free classification of sparse-graph markers.
 */

#ifndef VS_GRAPHS_SPARSE_CLUSTER_VERDICT_H
#define VS_GRAPHS_SPARSE_CLUSTER_VERDICT_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace vs_graphs::sparse
{

/*!
 * @brief        Marker type code of a CUBE_LIST marker, as in
 *               visualization_msgs/msg/Marker.msg. The message header is not
 *               included so this module builds without ROS.
 */
inline constexpr int SPARSE_CUBE_LIST_TYPE = 6;
/*!
 * @brief        Marker type code of a LINE_LIST marker, as in
 *               visualization_msgs/msg/Marker.msg.
 */
inline constexpr int SPARSE_LINE_LIST_TYPE = 5;

/*!
 * @brief        Prefix of the marker namespaces the skeletonizer uses for
 *               connected free-space components.
 */
inline constexpr std::string_view SPARSE_CLUSTER_NAMESPACE_PREFIX =
    "connected_vertices_";
/*!
 * @brief        Marker namespace the skeletonizer uses for raw skeleton edges.
 */
inline constexpr std::string_view SPARSE_EDGE_NAMESPACE = "edges";

/*!
 * @brief           Typed outcome of the marker-level ingest gate.
 */
enum class SparseMarkerVerdict : std::uint8_t
{
    /*! The marker carries no free-space geometry and is deliberately ignored.
     */
    SPARSE_MARKER_IGNORED = 0U,

    /*! A free-space cluster marker arrived without any point data. */
    SPARSE_CLUSTER_EMPTY = 1U,

    /*! A free-space cluster marker holds fewer raw points than required. */
    SPARSE_CLUSTER_UNDERSIZED = 2U,

    /*! A free-space cluster marker is eligible for the transform stage. */
    SPARSE_CLUSTER_CANDIDATE = 3U,

    /*! A skeleton edge marker arrived without any point data. */
    SPARSE_EDGE_EMPTY = 4U,

    /*! A skeleton edge marker is eligible for the transform stage. */
    SPARSE_EDGE_CANDIDATE = 5U
};

/*!
 * @brief           Per-callback rejection and acceptance counts.
 *
 *                  Written by the cluster ingest once per completed scan and
 *                  read back by the callback summary on the same subscription
 *                  thread. A bumped sequence with zeroed counts therefore
 *                  means the latest scan accepted nothing, not stale data.
 */
struct SparseIngestCounts
{
    /*!
     * @brief        Number of marker-array ingests so far; the latest ingest
     *               sets it, so a changed value means the other counts are
     *               fresh.
     */
    std::uint64_t sequence{0U};
    /*!
     * @brief        Markers skipped as carrying no free-space geometry.
     */
    std::size_t   ignoredMarkerCount{0U};
    /*!
     * @brief        Cluster markers skipped for having no points.
     */
    std::size_t   emptyClusterCount{0U};
    /*!
     * @brief        Cluster markers skipped for having fewer points than the
     *               minimum.
     */
    std::size_t   undersizedClusterCount{0U};
    /*!
     * @brief        Cluster markers skipped because their marker-to-world
     *               transform could not be resolved.
     */
    std::size_t   clusterTransformFailureCount{0U};
    /*!
     * @brief        Individual cluster points dropped because they could not
     *               be transformed into the world frame.
     */
    std::size_t   clusterPointDropCount{0U};
    /*!
     * @brief        Edge markers skipped for having no points.
     */
    std::size_t   emptyEdgeCount{0U};
    /*!
     * @brief        Edge markers or single edges skipped because a transform
     *               to the world frame failed.
     */
    std::size_t   edgeTransformFailureCount{0U};
    /*!
     * @brief        Edges dropped for non-finite endpoints or a length under
     *               1e-6 m.
     */
    std::size_t   degenerateEdgeCount{0U};
};

/*!
 * @brief           Classifies one sparse-graph marker ahead of transformation.
 *
 * @param[in]       markerType_in
 *                  Integer marker type code (SPARSE_CUBE_LIST_TYPE or
 *                  SPARSE_LINE_LIST_TYPE for eligible geometry).
 * @param[in]       markerNamespace_in
 *                  Marker namespace naming the free-space component.
 * @param[in]       pointCount_in
 *                  Number of raw points carried by the marker.
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

} /* namespace vs_graphs::sparse */

#endif /* VS_GRAPHS_SPARSE_CLUSTER_VERDICT_H */
