/*!
 * @file            addTwinFaceThicknessEdges.cc
 *
 * @brief           Implements addTwinFaceThicknessEdges(), declared in
 *                  public_functions.h. Adds one EdgeWallFaceThickness plus one
 *                  EdgeVertexPlaneParallelism between each symmetric twin plane
 *                  pair visible to the optimiser.
 */

#include "scene_understanding/semantic_factors/public_functions/public_functions.h"

#include <functional>
#include <optional>

#include "configuration/types/types.h"
#include "mapping/geometric/Plane/objects/PlaneClass.h"
#include "scene_understanding/semantic_factors/EdgeVertexPlaneParallelism/EdgeVertexPlaneParallelism.h"
#include "scene_understanding/semantic_factors/EdgeWallFaceThickness/EdgeWallFaceThickness.h"
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/types/vertex_plane.h>

namespace vs_graphs
{
namespace core
{

[[nodiscard]] optimizer::OptimizerStatus addTwinFaceThicknessEdges(
    g2o::SparseOptimizer                         &optimizer_inout,
    const std::vector<geometric::Plane *>        &planes_in,
    const std::function<int(geometric::Plane *)> &getVertexId_in,
    const types::SystemParams                    *p_systemParams_in,
    double                                        huberThreshold1d_in)
{
    if (p_systemParams_in == nullptr)
    {
        return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }

    /* The thickness constraint is disabled when enabled == false. */
    if (!p_systemParams_in->optimization.wallThickness.enabled)
    {
        return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
    }

    const double informationGain =
        p_systemParams_in->optimization.wallThickness.informationGain;

    for (geometric::Plane *pA : planes_in)
    {
        /* Get the twin face. */
        geometric::Plane *pB = nullptr;
        if (pA->getTwinFace(pB) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            pB == nullptr)
        {
            continue;
        }

        /* Both faces must not be bad. */
        bool aBad = false, bBad = false;
        if (pA->isBad(aBad) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            pB->isBad(bBad) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            aBad || bBad)
        {
            continue;
        }

        /* Symmetry: pB's twin must be pA. */
        geometric::Plane *pB_twin = nullptr;
        if (pB->getTwinFace(pB_twin) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            pB_twin != pA)
        {
            continue;
        }

        /* One edge per pair: lower id first. */
        int idA = 0, idB = 0;
        if (pA->getId(idA) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            pB->getId(idB) != geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            idA >= idB)
        {
            continue;
        }

        /* Anchor: midpoint of centroids. */
        Eigen::Vector3d        centroidA, centroidB;
        geometric::PlaneStatus csA = pA->getCentroid(centroidA);
        geometric::PlaneStatus csB = pB->getCentroid(centroidB);
        if (csA != geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            csB != geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            continue;
        }
        const Eigen::Vector3d anchor_m = (centroidA + centroidB) / 2.0;

        /* Plane equations for sign computation. */
        g2o::Plane3D planeEqA, planeEqB;
        if (pA->getGlobalEquation(planeEqA) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            pB->getGlobalEquation(planeEqB) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            continue;
        }
        const Eigen::Vector4d coeffsA = planeEqA.coeffs();
        const Eigen::Vector4d coeffsB = planeEqB.coeffs();
        const Eigen::Vector3d nA      = coeffsA.head<3>();
        const Eigen::Vector3d nB      = coeffsB.head<3>();
        const double          dA      = coeffsA(3);
        const double          dB      = coeffsB(3);

        /* Signs: σ chosen so σ·(n·o + c) > 0 at the observation origin. */
        std::optional<Eigen::Vector3d> originA, originB;
        if (pA->getObservationOrigin_world(originA) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS ||
            pB->getObservationOrigin_world(originB) !=
                geometric::PlaneStatus::PLANE_STATUS_SUCCESS)
        {
            continue;
        }

        /* A missing origin would default the sign to +1 and could pull the
         * faces together instead of apart, so skip the pair. */
        if (!originA.has_value() || !originB.has_value())
        {
            continue;
        }

        const double valA   = nA.dot(originA.value()) + dA;
        const double sigmaA = (valA > 0.0) ? 1.0 : -1.0;
        const double valB   = nB.dot(originB.value()) + dB;
        const double sigmaB = (valB > 0.0) ? 1.0 : -1.0;

        /* Vertices: the caller maps a plane to its optimiser vertex id, which
         * is getOpId() in local BA and getOpIdG() in global BA. */
        const int vertexIdA = getVertexId_in(pA);
        const int vertexIdB = getVertexId_in(pB);
        if (vertexIdA < 0 || vertexIdB < 0)
        {
            continue;
        }

        g2o::OptimizableGraph::Vertex *p_aBaseVertex =
            optimizer_inout.vertex(vertexIdA);
        g2o::OptimizableGraph::Vertex *p_bBaseVertex =
            optimizer_inout.vertex(vertexIdB);

        /* g2o ids are shared across vertex types, and a plane outside the
         * local window may carry an unset or stale optimisation id, so an id
         * alone does not prove the vertex is a VertexPlane. Rejecting a wrong
         * type here, before pB's fixed vertex is created, also ensures a
         * failed lookup never leaves an orphan fixed vertex in the graph. */
        g2o::VertexPlane *p_aVertex =
            dynamic_cast<g2o::VertexPlane *>(p_aBaseVertex);
        if (p_aVertex == nullptr)
        {
            continue;
        }
        g2o::VertexPlane *p_bVertex =
            dynamic_cast<g2o::VertexPlane *>(p_bBaseVertex);
        if (p_bVertex == nullptr && p_bBaseVertex != nullptr)
        {
            continue;
        }

        /* Add a fixed vertex for pB if it has none yet. */
        if (p_bVertex == nullptr)
        {
            p_bVertex = new g2o::VertexPlane();
            p_bVertex->setId(vertexIdB);
            p_bVertex->setEstimate(planeEqB);
            p_bVertex->setFixed(true);
            optimizer_inout.addVertex(p_bVertex);
        }

        /* Fix any plane vertex with no edges before adding the thickness edge,
         * so the unobserved twin does not absorb all correction. */
        if (p_aVertex->edges().empty())
        {
            p_aVertex->setFixed(true);
        }
        if (p_bVertex->edges().empty())
        {
            p_bVertex->setFixed(true);
        }

        /* Thickness edge between the twins. */
        optimizable_types::EdgeWallFaceThickness::EdgeWallFaceThickness
            *p_thicknessEdge = new optimizable_types::EdgeWallFaceThickness::
                EdgeWallFaceThickness();
        p_thicknessEdge->setVertex(
            0,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_aVertex));
        p_thicknessEdge->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_bVertex));
        p_thicknessEdge->anchor_m      = anchor_m;
        p_thicknessEdge->signA_world_m = sigmaA * nA;
        p_thicknessEdge->signB_world_m = sigmaB * nB;
        p_thicknessEdge->minimumThickness_m =
            p_systemParams_in->optimization.minimumThickness_m;
        p_thicknessEdge->setInformation(
            Eigen::Matrix<double, 1, 1>::Identity() * informationGain);
        g2o::RobustKernelHuber *p_thicknessKernel = new g2o::RobustKernelHuber;
        p_thicknessKernel->setDelta(huberThreshold1d_in);
        p_thicknessEdge->setRobustKernel(p_thicknessKernel);
        optimizer_inout.addEdge(p_thicknessEdge);

        /* Parallelism edge between the twins. */
        optimizable_types::EdgeVertexPlaneParallelism::
            EdgeVertexPlaneParallelism *p_parallelEdge = new optimizable_types::
                EdgeVertexPlaneParallelism::EdgeVertexPlaneParallelism();
        p_parallelEdge->setVertex(
            0,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_aVertex));
        p_parallelEdge->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex *>(p_bVertex));
        p_parallelEdge->setMeasurement(0.0);
        p_parallelEdge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                                       1e3);
        g2o::RobustKernelHuber *p_parallelKernel = new g2o::RobustKernelHuber;
        p_parallelKernel->setDelta(huberThreshold1d_in);
        p_parallelEdge->setRobustKernel(p_parallelKernel);
        optimizer_inout.addEdge(p_parallelEdge);
    }

    return optimizer::OptimizerStatus::OPTIMIZER_STATUS_SUCCESS;
}

} /* namespace core */
} /* namespace vs_graphs */
