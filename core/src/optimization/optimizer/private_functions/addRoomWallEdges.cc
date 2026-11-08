/*!
 * @file            addRoomWallEdges.cc
 *
 * @brief           Implements addRoomWallEdges(), declared in
 *                  private_functions.h. Ties each room's centre vertex to the
 *                  facing wall pairs that bound it.
 */

#include "optimization/optimizer/private_functions/private_functions.h"

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "configuration/types/types.h"
#include "mapping/geometric/Plane/objects/PlaneClass.h"
#include "mapping/geometry_helpers/utils.h"
#include "optimization/optimizable_types/EdgeVertex4PlaneProjectSE3Room/EdgeVertex4PlaneProjectSE3Room.h"
#include "optimization/optimizable_types/EdgeVertexNPlaneProjectSE3Room/EdgeVertexNPlaneProjectSE3Room.h"
#include "scene_understanding/scene_graph/room/Room.h"
#include <vs_graphs_g2o/types/types_six_dof_expmap.h>
#include <vs_graphs_g2o/types/vertex_plane.h>

namespace vs_graphs
{
namespace core
{

namespace
{

/*!
 * @brief           Returns the optimiser plane vertex of a wall, or nullptr
 *                  when the wall has no plane vertex in this optimisation.
 */
g2o::VertexPlane *
    findWallVertex(g2o::SparseOptimizer                         &optimizer_in,
                   geometric::Plane                             *p_wall_in,
                   const std::function<int(geometric::Plane *)> &getVertexId_in)
{
    const int vertexId = getVertexId_in(p_wall_in);
    if (vertexId < 0)
    {
        return nullptr;
    }
    /* g2o ids are shared across vertex types; check the type. */
    return dynamic_cast<g2o::VertexPlane *>(optimizer_in.vertex(vertexId));
}

/*!
 * @brief           Information matrix that weighs only the room-centre error
 *                  along a wall normal (gain * n * n^T): the edge's centre
 *                  estimate is exact only along the normals of its walls.
 */
Eigen::Matrix3d
    computeNormalInformation(const g2o::VertexPlane *p_wallVertex_in,
                             double                  informationGain_in)
{
    const Eigen::Vector3d wallNormal =
        p_wallVertex_in->estimate().normal().normalized();
    return informationGain_in * wallNormal * wallNormal.transpose();
}

} /* namespace */

[[nodiscard]] optimizer::OptimizerStatus addRoomWallEdges(
    g2o::SparseOptimizer                         &optimizer_inout,
    const std::vector<semantic::Room *>          &rooms_in,
    const std::function<int(geometric::Plane *)> &getWallVertexId_in,
    const std::function<int(semantic::Room *)>   &getRoomVertexId_in,
    const types::SystemParams                    *p_systemParams_in,
    double                                        huberThreshold3d_in)
{
    if (p_systemParams_in == nullptr ||
        !p_systemParams_in->optimization.roomWall.enabled)
    {
        return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }
    const double informationGain =
        p_systemParams_in->optimization.roomWall.informationGain;

    for (semantic::Room *p_room : rooms_in)
    {
        g2o::VertexSE3Expmap *p_roomVertex =
            dynamic_cast<g2o::VertexSE3Expmap *>(
                optimizer_inout.vertex(getRoomVertexId_in(p_room)));
        /*
         * Only a closed boundary places the room centre between its walls;
         * an open room's centre would pull its walls off their points.
         */
        bool                            isBoundaryComplete{};
        std::vector<geometric::Plane *> roomWalls{};
        if (p_roomVertex == nullptr ||
            p_room->isBoundaryComplete(isBoundaryComplete) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS ||
            !isBoundaryComplete ||
            p_room->getWalls(roomWalls) !=
                semantic::RoomStatus::ROOM_STATUS_SUCCESS)
        {
            continue;
        }

        /* Wall vertices of this optimisation; nullptr for a wall without. */
        std::vector<g2o::VertexPlane *> wallVertices{};
        for (geometric::Plane *p_wall : roomWalls)
        {
            wallVertices.push_back(
                findWallVertex(optimizer_inout, p_wall, getWallVertexId_in));
        }

        /*
         * Pair the walls that face each other across the room; each wall joins
         * at most one pair. Only a facing pair fixes the centre: its midpoint
         * along the shared normal is the room centre on that axis.
         */
        std::vector<std::pair<size_t, size_t>> facingPairs{};
        std::vector<bool> isWallPaired(roomWalls.size(), false);
        for (size_t firstIndex = 0; firstIndex < roomWalls.size(); firstIndex++)
        {
            for (size_t secondIndex = firstIndex + 1;
                 !isWallPaired[firstIndex] && secondIndex < roomWalls.size();
                 secondIndex++)
            {
                bool areWallsParallel{};
                bool areWallsFacing{};
                if (wallVertices[firstIndex] == nullptr ||
                    wallVertices[secondIndex] == nullptr ||
                    isWallPaired[secondIndex] ||
                    utils::utils::Utils::arePlanesParallel(
                        roomWalls[firstIndex],
                        roomWalls[secondIndex],
                        areWallsParallel) !=
                        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS ||
                    !areWallsParallel ||
                    utils::utils::Utils::arePlanesFacingEachOther(
                        roomWalls[firstIndex],
                        roomWalls[secondIndex],
                        areWallsFacing) !=
                        utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS ||
                    !areWallsFacing)
                {
                    continue;
                }
                isWallPaired[firstIndex]  = true;
                isWallPaired[secondIndex] = true;
                facingPairs.emplace_back(firstIndex, secondIndex);
            }
        }

        /* A four-wall room bounded by two crossing pairs: one 4-plane edge. */
        bool arePairsPerpendicular{};
        if (roomWalls.size() == 4 && facingPairs.size() == 2 &&
            utils::utils::Utils::arePlanesPerpendicular(
                roomWalls[facingPairs[0].first],
                roomWalls[facingPairs[1].first],
                arePairsPerpendicular) ==
                utils::utils::UtilsStatus::UTILS_STATUS_SUCCESS &&
            arePairsPerpendicular)
        {
            optimizable_types::EdgeVertex4PlaneProjectSE3Room::
                EdgeVertex4PlaneProjectSE3Room *p_edge =
                    new optimizable_types::EdgeVertex4PlaneProjectSE3Room::
                        EdgeVertex4PlaneProjectSE3Room();
            p_edge->setVertex(0, p_roomVertex);
            p_edge->setVertex(1, wallVertices[facingPairs[0].first]);
            p_edge->setVertex(2, wallVertices[facingPairs[0].second]);
            p_edge->setVertex(3, wallVertices[facingPairs[1].first]);
            p_edge->setVertex(4, wallVertices[facingPairs[1].second]);
            p_edge->setInformation(
                computeNormalInformation(wallVertices[facingPairs[0].first],
                                         informationGain) +
                computeNormalInformation(wallVertices[facingPairs[1].first],
                                         informationGain));
            g2o::RobustKernelHuber *p_robustKernel = new g2o::RobustKernelHuber;
            p_robustKernel->setDelta(huberThreshold3d_in);
            p_edge->setRobustKernel(p_robustKernel);
            optimizer_inout.addEdge(p_edge);
            continue;
        }

        /* Any other room: one N-plane edge per facing pair. */
        for (const std::pair<size_t, size_t> &facingPair : facingPairs)
        {
            optimizable_types::EdgeVertexNPlaneProjectSE3Room::
                EdgeVertexNPlaneProjectSE3Room *p_edge =
                    new optimizable_types::EdgeVertexNPlaneProjectSE3Room::
                        EdgeVertexNPlaneProjectSE3Room();
            p_edge->resize(3);
            p_edge->setVertex(0, p_roomVertex);
            p_edge->setVertex(1, wallVertices[facingPair.first]);
            p_edge->setVertex(2, wallVertices[facingPair.second]);
            p_edge->setInformation(
                computeNormalInformation(wallVertices[facingPair.first],
                                         informationGain));
            g2o::RobustKernelHuber *p_robustKernel = new g2o::RobustKernelHuber;
            p_robustKernel->setDelta(huberThreshold3d_in);
            p_edge->setRobustKernel(p_robustKernel);
            optimizer_inout.addEdge(p_edge);
        }
    }

    return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
