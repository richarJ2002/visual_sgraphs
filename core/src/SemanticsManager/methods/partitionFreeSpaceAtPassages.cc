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

#include "SemanticsManager.h"

#include "../private_functions.h"

#include <algorithm>

namespace vs_graphs
{
namespace core
{

std::vector<std::vector<Eigen::Vector3d>>
    SemanticsManager::partitionFreeSpaceAtPassages(
        const std::vector<std::vector<Eigen::Vector3d>>
            &freeSpaceClusters_World_m_in) const
{
    const types::SystemParams::RoomSeg::PassagePartition &partitionParameters =
        p_sysParams->roomSeg.passagePartition;

    if (!partitionParameters.enabled || freeSpaceClusters_World_m_in.empty())
    {
        return freeSpaceClusters_World_m_in;
    }

    const std::vector<semantic::Passage *> allPassages =
        p_atlas->getAllPassages();
    std::vector<semantic::Passage *> confirmedOpenPassages;

    for (semantic::Passage *p_passage : allPassages)
    {
        if (p_passage != nullptr && !p_passage->isBad() &&
            p_passage->isPassable())
        {
            confirmedOpenPassages.push_back(p_passage);
        }
    }

    geometric::Plane *p_groundPlane = p_atlas->getBiggestGroundPlane();

    if (confirmedOpenPassages.empty() || p_groundPlane == nullptr ||
        p_groundPlane->isBad())
    {
        return freeSpaceClusters_World_m_in;
    }

    Eigen::Vector4d groundEquation_World =
        p_groundPlane->getGlobalEquation().coeffs();
    const double groundNormalNorm = groundEquation_World.head<3>().norm();

    if (!groundEquation_World.allFinite() || groundNormalNorm < 1e-8)
    {
        return freeSpaceClusters_World_m_in;
    }

    const Eigen::Vector3d groundNormal_World =
        groundEquation_World.head<3>() / groundNormalNorm;
    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        skeletonEdges_World_m = p_atlas->getSkeletonEdges();

    if (skeletonEdges_World_m.empty())
    {
        return freeSpaceClusters_World_m_in;
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
    std::vector<std::vector<Eigen::Vector3d>> partitionedClusters_World_m;

    for (const std::vector<Eigen::Vector3d> &cluster_World_m :
         freeSpaceClusters_World_m_in)
    {
        if (cluster_World_m.size() < minimumClusterVertexCount)
        {
            continue;
        }

        std::vector<std::vector<std::size_t>> adjacency(cluster_World_m.size());
        std::vector<bool> graphVertexWasObserved(cluster_World_m.size(), false);
        std::size_t       cutEdgeCount = 0U;

        const auto findNearestClusterVertex =
            [&cluster_World_m, maximumAssociationSquaredDistance_m2](
                const Eigen::Vector3d &edgePoint_World_m_in) -> std::size_t
        {
            std::size_t nearestVertexIndex = cluster_World_m.size();
            double      nearestSquaredDistance_m2 =
                maximumAssociationSquaredDistance_m2;

            for (std::size_t vertexIndex = 0U;
                 vertexIndex < cluster_World_m.size();
                 ++vertexIndex)
            {
                const double squaredDistance_m2 =
                    (cluster_World_m[vertexIndex] - edgePoint_World_m_in)
                        .squaredNorm();

                if (squaredDistance_m2 <= nearestSquaredDistance_m2)
                {
                    nearestSquaredDistance_m2 = squaredDistance_m2;
                    nearestVertexIndex        = vertexIndex;
                }
            }

            return nearestVertexIndex;
        };

        for (const auto &skeletonEdge_World_m : skeletonEdges_World_m)
        {
            const std::size_t startVertexIndex =
                findNearestClusterVertex(skeletonEdge_World_m.first);
            const std::size_t endVertexIndex =
                findNearestClusterVertex(skeletonEdge_World_m.second);

            if (startVertexIndex >= cluster_World_m.size() ||
                endVertexIndex >= cluster_World_m.size() ||
                startVertexIndex == endVertexIndex)
            {
                continue;
            }

            graphVertexWasObserved[startVertexIndex] = true;
            graphVertexWasObserved[endVertexIndex]   = true;

            const bool crossesConfirmedPassage = std::any_of(
                confirmedOpenPassages.begin(),
                confirmedOpenPassages.end(),
                [&skeletonEdge_World_m,
                 &groundNormal_World,
                 openingMargin_m,
                 minimumSideDistance_m](semantic::Passage *p_passage)
                {
                    return segmentCrossesPassageOpening(
                        skeletonEdge_World_m.first,
                        skeletonEdge_World_m.second,
                        p_passage,
                        groundNormal_World,
                        openingMargin_m,
                        minimumSideDistance_m);
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
            static_cast<double>(cluster_World_m.size());

        /*
         * Preserve the upstream connected component when edge-to-vertex
         * reconstruction is underconstrained or no passage edge was cut.
         */
        if (graphCoverageRatio < minimumGraphCoverageRatio ||
            cutEdgeCount == 0U)
        {
            partitionedClusters_World_m.push_back(cluster_World_m);
            continue;
        }

        std::vector<bool> vertexWasVisited(cluster_World_m.size(), false);
        const std::size_t outputClusterCountBeforePartition =
            partitionedClusters_World_m.size();

        for (std::size_t seedVertexIndex = 0U;
             seedVertexIndex < cluster_World_m.size();
             ++seedVertexIndex)
        {
            if (vertexWasVisited[seedVertexIndex] ||
                !graphVertexWasObserved[seedVertexIndex])
            {
                continue;
            }

            std::vector<std::size_t>     pendingVertexIndices{seedVertexIndex};
            std::vector<Eigen::Vector3d> component_World_m;
            vertexWasVisited[seedVertexIndex] = true;

            while (!pendingVertexIndices.empty())
            {
                const std::size_t vertexIndex = pendingVertexIndices.back();
                pendingVertexIndices.pop_back();
                component_World_m.push_back(cluster_World_m[vertexIndex]);

                for (const std::size_t neighbourIndex : adjacency[vertexIndex])
                {
                    if (!vertexWasVisited[neighbourIndex])
                    {
                        vertexWasVisited[neighbourIndex] = true;
                        pendingVertexIndices.push_back(neighbourIndex);
                    }
                }
            }

            if (component_World_m.size() >= minimumClusterVertexCount)
            {
                partitionedClusters_World_m.push_back(
                    std::move(component_World_m));
            }
        }

        /*
         * Do not lose a valid upstream component when every reconstructed
         * child is below the room detector's minimum size. The passage still
         * remains in the semantic graph and will be retried as Voxblox gains
         * more observed free-space vertices.
         */
        if (partitionedClusters_World_m.size() ==
            outputClusterCountBeforePartition)
        {
            partitionedClusters_World_m.push_back(cluster_World_m);
        }
    }

    return partitionedClusters_World_m;
}

} // namespace core
} // namespace vs_graphs
