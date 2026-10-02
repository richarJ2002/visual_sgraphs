/*!
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez,
 * and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details: https://www.gnu.org/licenses/
 */

/*!
 * @file            partitionFreeSpaceAtPassages.cc
 *
 * @brief           Implements SemanticsManager::partitionFreeSpaceAtPassages(),
 *                  declared in SemanticsManager.h.
 */

#include "SemanticsManager.h"

#include "../private_functions.h"

#include <algorithm>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

SemanticsManagerStatus SemanticsManager::partitionFreeSpaceAtPassages(
    const std::vector<std::vector<Eigen::Vector3d>>
                                              &freeSpaceClusters_world_m_in,
    std::vector<std::vector<Eigen::Vector3d>> &partitions_out) const
{
    const types::SystemParams::RoomSeg::PassagePartition &partitionParameters =
        p_sysParams->roomSeg.passagePartition;

    if (!partitionParameters.enabled || freeSpaceClusters_world_m_in.empty())
    {
        partitions_out = freeSpaceClusters_world_m_in;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    std::vector<semantic::Passage *> allPassages{};
    if (p_atlas->getAllPassages(allPassages) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllPassages returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<semantic::Passage *> confirmedOpenPassages;

    for (semantic::Passage *p_passage : allPassages)
    {
        bool passageIsBad{};
        if ((p_passage != nullptr) &&
            p_passage->isBad(passageIsBad) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isBad returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        bool passageIsPassable{};
        if ((p_passage != nullptr && !passageIsBad) &&
            p_passage->isPassable(passageIsPassable) !=
                semantic::PassageStatus::PASSAGE_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: isPassable returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        if (p_passage != nullptr && !passageIsBad && passageIsPassable)
        {
            confirmedOpenPassages.push_back(p_passage);
        }
    }

    geometric::Plane *p_groundPlane = nullptr;
    if (p_atlas->getBiggestGroundPlane(p_groundPlane) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getBiggestGroundPlane returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    bool groundPlaneIsBad{};
    if (!(confirmedOpenPassages.empty() || p_groundPlane == nullptr) &&
        p_groundPlane->isBad(groundPlaneIsBad) !=
            geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: isBad returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    if (confirmedOpenPassages.empty() || p_groundPlane == nullptr ||
        groundPlaneIsBad)
    {
        partitions_out = freeSpaceClusters_world_m_in;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    g2o::Plane3D groundPlaneGetGlobalEquation{};
    if (p_groundPlane->getGlobalEquation(groundPlaneGetGlobalEquation) !=
        geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalEquation returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    Eigen::Vector4d groundEquation_world =
        groundPlaneGetGlobalEquation.coeffs();
    const double groundNormalNorm = groundEquation_world.head<3>().norm();

    if (!groundEquation_world.allFinite() || groundNormalNorm < 1e-8)
    {
        partitions_out = freeSpaceClusters_world_m_in;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const Eigen::Vector3d groundNormal_world =
        groundEquation_world.head<3>() / groundNormalNorm;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        skeletonEdges_world_m{};
    if (p_atlas->getSkeletonEdges(skeletonEdges_world_m) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getSkeletonEdges returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }

    if (skeletonEdges_world_m.empty())
    {
        partitions_out = freeSpaceClusters_world_m_in;
        return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
    }

    const double associationDistance_m = static_cast<double>(
        partitionParameters.edgeVertexAssociationDistance_m);
    const double maximumAssociationSquaredDistance_m2 =
        associationDistance_m * associationDistance_m;
    const double minimumGraphCoverageRatio =
        static_cast<double>(partitionParameters.minimumGraphCoverageRatio);
    const double openingMargin_m =
        static_cast<double>(partitionParameters.openingMargin_m);
    const double minimumSideDistance_m =
        static_cast<double>(partitionParameters.minimumSideDistance_m);
    const std::size_t minimumClusterVertexCount =
        std::max<std::size_t>(p_sysParams->roomSeg.minClusterVertices, 1U);
    std::vector<std::vector<Eigen::Vector3d>> partitionedClusters_world_m;

    for (const std::vector<Eigen::Vector3d> &freeSpaceCluster_world_m :
         freeSpaceClusters_world_m_in)
    {
        if (freeSpaceCluster_world_m.size() < minimumClusterVertexCount)
        {
            continue;
        }

        std::vector<std::vector<std::size_t>> adjacency(
            freeSpaceCluster_world_m.size());
        std::vector<bool> graphVertexWasObserved(
            freeSpaceCluster_world_m.size(),
            false);
        std::size_t cutEdgeCount = 0U;

        const auto findNearestClusterVertex =
            [&freeSpaceCluster_world_m, maximumAssociationSquaredDistance_m2](
                const Eigen::Vector3d &edgePoint_world_m_in) -> std::size_t
        {
            std::size_t nearestVertexIndex = freeSpaceCluster_world_m.size();
            double      nearestSquaredDistance_m2 =
                maximumAssociationSquaredDistance_m2;

            for (std::size_t vertexIndex = 0U;
                 vertexIndex < freeSpaceCluster_world_m.size();
                 ++vertexIndex)
            {
                const double squaredDistance_m2 =
                    (freeSpaceCluster_world_m[vertexIndex] -
                     edgePoint_world_m_in)
                        .squaredNorm();

                if (squaredDistance_m2 <= nearestSquaredDistance_m2)
                {
                    nearestSquaredDistance_m2 = squaredDistance_m2;
                    nearestVertexIndex        = vertexIndex;
                }
            }

            return nearestVertexIndex;
        };

        for (const std::pair<Eigen::Vector3d, Eigen::Vector3d>
                 &skeletonEdge_world_m : skeletonEdges_world_m)
        {
            const std::size_t startVertexIndex =
                findNearestClusterVertex(skeletonEdge_world_m.first);
            const std::size_t endVertexIndex =
                findNearestClusterVertex(skeletonEdge_world_m.second);

            if (startVertexIndex >= freeSpaceCluster_world_m.size() ||
                endVertexIndex >= freeSpaceCluster_world_m.size() ||
                startVertexIndex == endVertexIndex)
            {
                continue;
            }

            graphVertexWasObserved[startVertexIndex] = true;
            graphVertexWasObserved[endVertexIndex]   = true;

            const bool crossesConfirmedPassage = std::any_of(
                confirmedOpenPassages.begin(),
                confirmedOpenPassages.end(),
                [&skeletonEdge_world_m,
                 &groundNormal_world,
                 openingMargin_m,
                 minimumSideDistance_m](semantic::Passage *p_passage)
                {
                    bool crossesPassageOpening{};
                    if (segmentCrossesPassageOpening(
                            skeletonEdge_world_m.first,
                            skeletonEdge_world_m.second,
                            p_passage,
                            groundNormal_world,
                            openingMargin_m,
                            minimumSideDistance_m,
                            crossesPassageOpening) !=
                        SemanticsManagerStatus::
                            SEMANTICS_MANAGER_STATUS_SUCCESS)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                                     "%s: segmentCrossesPassageOpening "
                                     "returned a failure status although it "
                                     "cannot fail; continuing as before.",
                                     __func__);
                    }
                    return crossesPassageOpening;
                });

            if (crossesConfirmedPassage)
            {
                cutEdgeCount++;
                continue;
            }

            adjacency[startVertexIndex].push_back(endVertexIndex);
            adjacency[endVertexIndex].push_back(startVertexIndex);
        }

        const std::size_t observedGraphVertexCount =
            static_cast<std::size_t>(std::count(graphVertexWasObserved.begin(),
                                                graphVertexWasObserved.end(),
                                                true));
        const double graphCoverageRatio =
            static_cast<double>(observedGraphVertexCount) /
            static_cast<double>(freeSpaceCluster_world_m.size());

        /*
         * Preserve the upstream connected component when edge-to-vertex
         * reconstruction is underconstrained or no passage edge was cut.
         */
        if (graphCoverageRatio < minimumGraphCoverageRatio ||
            cutEdgeCount == 0U)
        {
            partitionedClusters_world_m.push_back(freeSpaceCluster_world_m);
            continue;
        }

        std::vector<bool> vertexWasVisited(freeSpaceCluster_world_m.size(),
                                           false);
        const std::size_t outputClusterCountBeforePartition =
            partitionedClusters_world_m.size();

        for (std::size_t seedVertexIndex = 0U;
             seedVertexIndex < freeSpaceCluster_world_m.size();
             ++seedVertexIndex)
        {
            if (vertexWasVisited[seedVertexIndex] ||
                !graphVertexWasObserved[seedVertexIndex])
            {
                continue;
            }

            std::vector<std::size_t>     pendingVertexIndices{seedVertexIndex};
            std::vector<Eigen::Vector3d> connectedComponent_world_m;
            vertexWasVisited[seedVertexIndex] = true;

            while (!pendingVertexIndices.empty())
            {
                const std::size_t vertexIndex = pendingVertexIndices.back();
                pendingVertexIndices.pop_back();
                connectedComponent_world_m.push_back(
                    freeSpaceCluster_world_m[vertexIndex]);

                for (const std::size_t neighbourIndex : adjacency[vertexIndex])
                {
                    if (!vertexWasVisited[neighbourIndex])
                    {
                        vertexWasVisited[neighbourIndex] = true;
                        pendingVertexIndices.push_back(neighbourIndex);
                    }
                }
            }

            if (connectedComponent_world_m.size() >= minimumClusterVertexCount)
            {
                partitionedClusters_world_m.push_back(
                    std::move(connectedComponent_world_m));
            }
        }

        /*
         * Do not lose a valid upstream component when every reconstructed
         * child is below the room detector's minimum size. The passage still
         * remains in the semantic graph and will be retried as Voxblox gains
         * more observed free-space vertices.
         */
        if (partitionedClusters_world_m.size() ==
            outputClusterCountBeforePartition)
        {
            partitionedClusters_world_m.push_back(freeSpaceCluster_world_m);
        }
    }

    partitions_out = partitionedClusters_world_m;
    return SemanticsManagerStatus::SEMANTICS_MANAGER_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
