/*!
 * @File:         setVoxbloxSkeletonCluster.cpp
 *
 * @Brief:        Extracts connected free-space components and raw skeleton
 *                edges from a Voxblox visualisation marker array.
 *
 * @Date:         20/07/2026
 *
 */

#include <algorithm>
#include <utility>

/* Function Includes */
#include "Common.hpp"

/* Object Include */
/* None */

/* Data include */
/* None */

/* Generic Libraries */
/* None */

void setVoxbloxSkeletonCluster(
    const visualization_msgs::msg::MarkerArray &skeletonMarkerArray_in)
{
    /* Confirm that the SLAM system has been initialised */
    if (pSLAM == nullptr)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("visual_sgraphs"),
            "Cannot store Voxblox skeleton: SLAM system is not initialised.");

        return;
    }

    /* Obtain the configured room-segmentation parameters */
    const auto *systemParameters = ORB_SLAM3::SystemParams::GetParams();

    if (systemParameters == nullptr)
    {
        RCLCPP_WARN(rclcpp::get_logger("visual_sgraphs"),
                    "Cannot store Voxblox skeleton: system parameters are "
                    "unavailable.");

        return;
    }

    /*!
     * Prevent a negative configured value from being converted into a very
     * large unsigned integer.
     */
    const int configuredMinimumClusterVertices =
        systemParameters->room_seg.min_cluster_vertices;

    const std::size_t minimumClusterVertexCount =
        configuredMinimumClusterVertices > 0
            ? static_cast<std::size_t>(configuredMinimumClusterVertices)
            : 1U;

    /*!
     * Build new buffers locally. The global buffers are replaced only after
     * the complete marker array has been processed.
     */
    std::vector<std::vector<Eigen::Vector3d>> transformedSkeletonClusters_world;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        transformedSkeletonEdges_world;

    transformedSkeletonClusters_world.reserve(
        skeletonMarkerArray_in.markers.size());

    /* Process every marker contained in the sparse graph message */
    for (const visualization_msgs::msg::Marker &skeletonMarker :
         skeletonMarkerArray_in.markers)
    {
        /*!
         * Voxblox publishes each connected free-space component using a marker
         * namespace beginning with "connected_vertices_".
         *
         * Generic "vertices" and "closed_spaces" markers are deliberately
         * ignored because they duplicate the connected-component data.
         */
        const bool isConnectedVertexMarker =
            skeletonMarker.type == visualization_msgs::msg::Marker::CUBE_LIST &&
            skeletonMarker.ns.rfind("connected_vertices_", 0) == 0;

        /*!
         * The raw "edges" marker is used instead of connected_edges_* because
         * connected edge markers may have already been clearance filtered at
         * narrow passages.
         */
        const bool isRawEdgeMarker =
            skeletonMarker.type == visualization_msgs::msg::Marker::LINE_LIST &&
            skeletonMarker.ns == "edges";

        /* Ignore marker types that are not required by this pipeline */
        if (!isConnectedVertexMarker && !isRawEdgeMarker)
        {
            continue;
        }

        /* Ignore markers without any point data */
        if (skeletonMarker.points.empty())
        {
            continue;
        }

        /*
         * Resolve the marker-local-to-world transformation once and reuse it
         * for every point belonging to this marker.
         */
        tf2::Transform T_world_skeletonMarker;

        if (!getSkeletonMarkerWorldTransform(skeletonMarker,
                                             T_world_skeletonMarker))
        {
            continue;
        }

        /* ------------------------------------------------------------------ *
         * CONNECTED FREE-SPACE COMPONENT
         * ------------------------------------------------------------------ */

        if (isConnectedVertexMarker)
        {
            /* Ignore undersized connected components */
            if (skeletonMarker.points.size() < minimumClusterVertexCount)
            {
                continue;
            }

            std::vector<Eigen::Vector3d> connectedComponentPoints_world;

            connectedComponentPoints_world.reserve(
                skeletonMarker.points.size());

            /* Transform every connected vertex into the world frame */
            for (const geometry_msgs::msg::Point &skeletonPoint_marker :
                 skeletonMarker.points)
            {
                Eigen::Vector3d skeletonPoint_world;

                if (!transformSkeletonPoint(T_world_skeletonMarker,
                                            skeletonPoint_marker,
                                            skeletonPoint_world))
                {
                    continue;
                }

                connectedComponentPoints_world.push_back(skeletonPoint_world);
            }

            /*
             * Store the component only when enough valid transformed vertices
             * remain.
             */
            if (connectedComponentPoints_world.size() >=
                minimumClusterVertexCount)
            {
                transformedSkeletonClusters_world.push_back(
                    std::move(connectedComponentPoints_world));
            }

            continue;
        }

        /* ------------------------------------------------------------------ *
         * RAW SKELETON EDGES
         * ------------------------------------------------------------------ */

        /*!
         * Each consecutive pair of LINE_LIST points represents one independent
         * edge.
         */
        transformedSkeletonEdges_world.reserve(
            transformedSkeletonEdges_world.size() +
            skeletonMarker.points.size() / 2);

        for (std::size_t pointIndex = 0;
             pointIndex + 1 < skeletonMarker.points.size();
             pointIndex += 2)
        {
            Eigen::Vector3d edgeStart_world;
            Eigen::Vector3d edgeEnd_world;

            const bool isEdgeStartValid =
                transformSkeletonPoint(T_world_skeletonMarker,
                                       skeletonMarker.points[pointIndex],
                                       edgeStart_world);

            const bool isEdgeEndValid =
                transformSkeletonPoint(T_world_skeletonMarker,
                                       skeletonMarker.points[pointIndex + 1],
                                       edgeEnd_world);

            if (!isEdgeStartValid || !isEdgeEndValid)
            {
                continue;
            }

            const double edgeLength_m =
                (edgeEnd_world - edgeStart_world).norm();

            /* Reject invalid and effectively zero-length edges */
            if (!edgeStart_world.allFinite() || !edgeEnd_world.allFinite() ||
                edgeLength_m < 1e-6)
            {
                continue;
            }

            transformedSkeletonEdges_world.emplace_back(edgeStart_world,
                                                        edgeEnd_world);
        }
    }

    /*!
     * Replace the shared buffers only after processing has completed. This
     * avoids exposing partially rebuilt data through these global collections.
     */
    skeletonClusterPoints = std::move(transformedSkeletonClusters_world);

    skeletonEdges = std::move(transformedSkeletonEdges_world);

    /* Store the connected skeleton vertices in the active map */
    pSLAM->setSkeletonCluster(skeletonClusterPoints);

    /* Store the complete raw skeleton edges in the active map */
    pSLAM->setSkeletonEdges(skeletonEdges);

    RCLCPP_INFO(
        rclcpp::get_logger("visual_sgraphs"),
        "Stored %zu Voxblox connected components and %zu raw skeleton edges.",
        skeletonClusterPoints.size(),
        skeletonEdges.size());
}