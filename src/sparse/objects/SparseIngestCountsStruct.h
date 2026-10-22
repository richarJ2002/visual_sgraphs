/*!
 * @File:         SparseIngestCountsStruct.h
 *
 * @Brief:        TODO
 *
 * @Date:         09/10/2026
 *
 */

#ifndef HPP_SPARSEINGESTCOUNTSSTRUCT_HPP
#define HPP_SPARSEINGESTCOUNTSSTRUCT_HPP

/* Function Includes */
/* None */

/* Object Include */
/* None */

#include <cstddef>
#include <cstdint>

/* Generic Libraries */
/* None */

namespace vs_graphs::sparse
{

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
  public:
    /*!
     * @brief           Number of marker-array ingests so far; the latest ingest
     *                  sets it, so a changed value means the other counts are
     *                  fresh.
     *
     */
    std::uint64_t sequence{0U};

    /*!
     * @brief           Markers skipped as carrying no free-space geometry.
     *
     */
    std::size_t ignoredMarkerCount{0U};

    /*!
     * @brief           Cluster markers skipped for having no points.
     *
     */
    std::size_t emptyClusterCount{0U};

    /*!
     * @brief           Cluster markers skipped for having fewer points than the
     *                  minimum.
     *
     */
    std::size_t undersizedClusterCount{0U};

    /*!
     * @brief           Cluster markers skipped because their marker-to-world
     *                  transform could not be resolved.
     *
     */
    std::size_t clusterTransformFailureCount{0U};

    /*!
     * @brief           Individual cluster points dropped because they could not
     *                  be transformed into the world frame.
     *
     */
    std::size_t clusterPointDropCount{0U};

    /*!
     * @brief           Edge markers skipped for having no points.
     *
     */
    std::size_t emptyEdgeCount{0U};

    /*!
     * @brief           Edge markers or single edges skipped because a transform
     *                  to the world frame failed.
     *
     */
    std::size_t edgeTransformFailureCount{0U};

    /*!
     * @brief           Edges dropped for non-finite endpoints or a length under
     *                  1e-6 m.
     *
     */
    std::size_t degenerateEdgeCount{0U};
};

} // namespace vs_graphs::sparse

#endif /* HPP_SPARSEINGESTCOUNTSSTRUCT_HPP */
