/*!
 * @file            addWallCornerEdges.cc
 *
 * @brief           Implements addWallCornerEdges(), declared in
 *                  private_functions.h.
 */

#include "scene_understanding/semantic_factors/private_functions/private_functions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/types/vertex_plane.h>

#include "configuration/types/types.h"
#include "scene_understanding/semantic_factors/EdgeWallCorner/EdgeWallCorner.h"

namespace vs_graphs
{
namespace core
{

void addWallCornerEdges(g2o::SparseOptimizer             &optimizer_inout,
                        const std::vector<WallSegment3d> &wallSegments_in,
                        const Eigen::Vector3d            &upDirection_world_in,
                        const types::SystemParams        *p_systemParams_in,
                        double                            huberThreshold2d_in,
                        std::size_t                      &edgeCount_out,
                        double                           &initialChi2_out)
{
    edgeCount_out   = 0U;
    initialChi2_out = 0.0;
    const types::SystemParams::Semantics::Factors::WallEdge
        &wallEdgeParameters   = p_systemParams_in->semantics.factors.wallEdge;
    const double maximumGap_m = wallEdgeParameters.maxGap_m;

    /*!
     * Crossing means closer to perpendicular than to parallel, the rule the
     * room footprint uses; parallel walls (twin faces, a wall split in two)
     * have no corner.
     */
    const double crossingCosineLimit = std::cos(M_PI / 4.0);

    for (std::size_t firstIndex = 0U; firstIndex < wallSegments_in.size();
         firstIndex++)
    {
        const WallSegment3d &firstWall = wallSegments_in[firstIndex];
        const std::array<Eigen::Vector3d, 2> firstEnds_world_m = {
            firstWall.wallStart_world_m,
            firstWall.wallEnd_world_m};
        for (std::size_t secondIndex = firstIndex + 1U;
             secondIndex < wallSegments_in.size();
             secondIndex++)
        {
            const WallSegment3d &secondWall = wallSegments_in[secondIndex];
            bool                 doWallsShareRoom = false;
            for (const int firstRoomId : firstWall.roomIds)
            {
                doWallsShareRoom =
                    doWallsShareRoom ||
                    std::binary_search(secondWall.roomIds.begin(),
                                       secondWall.roomIds.end(),
                                       firstRoomId);
            }
            if (!doWallsShareRoom ||
                std::abs(firstWall.horizontalNormal_world.dot(
                    secondWall.horizontalNormal_world)) >= crossingCosineLimit)
            {
                continue;
            }

            /* The nearest pair of end points, measured horizontally. */
            const std::array<Eigen::Vector3d, 2> secondEnds_world_m = {
                secondWall.wallStart_world_m,
                secondWall.wallEnd_world_m};
            double      nearestGap_m     = std::numeric_limits<double>::max();
            std::size_t nearestFirstEnd  = 0U;
            std::size_t nearestSecondEnd = 0U;
            for (std::size_t firstEnd = 0U; firstEnd < 2U; firstEnd++)
            {
                for (std::size_t secondEnd = 0U; secondEnd < 2U; secondEnd++)
                {
                    const Eigen::Vector3d endOffset_world_m =
                        firstEnds_world_m[firstEnd] -
                        secondEnds_world_m[secondEnd];
                    const double gap_m =
                        (endOffset_world_m -
                         endOffset_world_m.dot(upDirection_world_in) *
                             upDirection_world_in)
                            .norm();
                    if (gap_m < nearestGap_m)
                    {
                        nearestGap_m     = gap_m;
                        nearestFirstEnd  = firstEnd;
                        nearestSecondEnd = secondEnd;
                    }
                }
            }
            if (nearestGap_m > maximumGap_m)
            {
                continue;
            }

            optimizable_types::EdgeWallCorner::EdgeWallCorner *p_cornerEdge =
                new optimizable_types::EdgeWallCorner::EdgeWallCorner();
            p_cornerEdge->setVertex(0, firstWall.p_wallVertex);
            p_cornerEdge->setVertex(1, secondWall.p_wallVertex);
            p_cornerEdge->endPointA_world_m =
                firstEnds_world_m[nearestFirstEnd];
            p_cornerEdge->endPointB_world_m =
                secondEnds_world_m[nearestSecondEnd];
            p_cornerEdge->setInformation(Eigen::Matrix2d::Identity() *
                                         wallEdgeParameters.informationGain);
            g2o::RobustKernelHuber *p_cornerKernel = new g2o::RobustKernelHuber;
            p_cornerKernel->setDelta(huberThreshold2d_in);
            p_cornerEdge->setRobustKernel(p_cornerKernel);
            optimizer_inout.addEdge(p_cornerEdge);
            p_cornerEdge->computeError();
            initialChi2_out += p_cornerEdge->chi2();
            edgeCount_out++;
        }
    }
}

} /* namespace core */
} /* namespace vs_graphs */
