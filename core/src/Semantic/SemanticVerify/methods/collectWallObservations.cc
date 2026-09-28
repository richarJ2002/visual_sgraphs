

#include "Semantic/SemanticVerify.h"

#include "Geometric/Plane.h"
#include "LoopClosing.h"
#include "Map.h"
#include "OptimizableTypes.h"
#include "Semantic/Room.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "Types/objects/SystemParams.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "../private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus SemanticVerify::collectWallObservations(
    const Room                         *p_room_in,
    const SemanticVerifyConfig         &configuration_in,
    std::vector<VerifyWallObservation> &observations_out)
{
    std::vector<VerifyWallObservation> observations;
    if (p_room_in == nullptr)
    {
        observations_out = observations;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    Eigen::Vector3d roomCentroid_World{};
    if (p_room_in->getCentroid(roomCentroid_World) !=
        RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getCentroid cannot fail; continue as before.
    }
    if (!isFiniteVector(roomCentroid_World))
    {
        observations_out = observations;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }

    std::vector<geometric::Plane *> room_inWalls{};
    if (p_room_in->getWalls(room_inWalls) != RoomStatus::ROOM_STATUS_SUCCESS)
    {
        // getWalls cannot fail; continue as before.
    }
    for (geometric::Plane *p_wall : room_inWalls)
    {
        if (observations.size() == configuration_in.maxWallsPerRoom)
        {
            break;
        }
        if (p_wall == nullptr || p_wall->isBad())
        {
            continue;
        }

        /* Re-derive the SAME oriented (n,d) pair Room::
         * getWallNormalTowardRoom_World computes internally, but keep d
         * paired with the (possibly sign-flipped) normal -- the existing
         * getter returns only the oriented normal, not a paired oriented d,
         * and n^T x + d = 0 requires both to flip together. */
        Eigen::Vector4d coeffs     = p_wall->getGlobalEquation().coeffs();
        const double    normalNorm = coeffs.head<3>().norm();
        if (!std::isfinite(normalNorm) || normalNorm < 1e-8)
        {
            continue;
        }
        coeffs /= normalNorm;
        if (!coeffs.allFinite())
        {
            continue;
        }
        const double signedDistance =
            coeffs.head<3>().dot(roomCentroid_World) + coeffs(3);
        if (!std::isfinite(signedDistance))
        {
            continue;
        }
        if (signedDistance < 0.0)
        {
            coeffs = -coeffs;
        }

        VerifyWallObservation observation;
        observation.wallId         = p_wall->getId();
        observation.normal_World   = coeffs.head<3>();
        observation.d              = coeffs(3);
        observation.centroid_World = p_wall->getCentroid();
        if (!isFiniteVector(observation.centroid_World))
        {
            continue;
        }

        const geometric::Plane::GeometrySnapshot snapshot =
            p_wall->getGeometrySnapshot();
        if (snapshot.supportCloud && !snapshot.supportCloud->empty())
        {
            const std::size_t total  = snapshot.supportCloud->size();
            const std::size_t stride = std::max<std::size_t>(
                1U,
                total / configuration_in.maxSupportSamplePerWall);
            for (std::size_t index = 0U;
                 index < total && observation.supportSample_World.size() <
                                      configuration_in.maxSupportSamplePerWall;
                 index += stride)
            {
                const auto &point = snapshot.supportCloud->points[index];
                if (!pcl::isFinite(point))
                {
                    continue;
                }
                observation.supportSample_World.emplace_back(
                    static_cast<double>(point.x),
                    static_cast<double>(point.y),
                    static_cast<double>(point.z));
            }
        }

        observations.push_back(std::move(observation));
    }

    observations_out = observations;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
