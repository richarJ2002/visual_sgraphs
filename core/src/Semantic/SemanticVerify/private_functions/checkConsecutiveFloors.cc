

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

bool checkConsecutiveFloors(core::Map       *p_survivingMap_in,
                            core::Map       *p_absorbedMap_in,
                            const g2o::Sim3 &transform_in,
                            double           maximumOffset_m_in,
                            std::string     &decision_out)
{
    Floor *p_survivingFloor = nullptr;
    if (Floor::selectBestObservedFloor(p_survivingMap_in->getAllFloors(),
                                       p_survivingFloor) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // selectBestObservedFloor cannot fail; continue as before.
    }
    Floor *p_absorbedFloor = nullptr;
    if (Floor::selectBestObservedFloor(p_absorbedMap_in->getAllFloors(),
                                       p_absorbedFloor) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // selectBestObservedFloor cannot fail; continue as before.
    }
    std::optional<Floor::PlaneIdentity> survivingFloorPlaneIdentity{};
    if ((p_survivingFloor != nullptr) &&
        p_survivingFloor->getPlaneIdentity(survivingFloorPlaneIdentity) !=
            FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getPlaneIdentity cannot fail; continue as before.
    }
    const std::optional<Floor::PlaneIdentity> survivingIdentity =
        p_survivingFloor != nullptr ? survivingFloorPlaneIdentity
                                    : std::nullopt;
    std::optional<Floor::PlaneIdentity> absorbedFloorPlaneIdentity{};
    if ((p_absorbedFloor != nullptr) &&
        p_absorbedFloor->getPlaneIdentity(absorbedFloorPlaneIdentity) !=
            FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // getPlaneIdentity cannot fail; continue as before.
    }
    const std::optional<Floor::PlaneIdentity> absorbedIdentity =
        p_absorbedFloor != nullptr ? absorbedFloorPlaneIdentity : std::nullopt;
    if (!survivingIdentity.has_value() || !absorbedIdentity.has_value())
    {
        decision_out = "DEFERRED";
        return false;
    }
    std::optional<Floor::PlaneIdentity> transformedIdentity{};
    if (Floor::transformPlaneIdentity(*absorbedIdentity,
                                      transform_in,
                                      transformedIdentity) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // transformPlaneIdentity cannot fail; continue as before.
    }
    double normalAngle_deg = std::numeric_limits<double>::infinity();
    double offset_m        = std::numeric_limits<double>::infinity();
    bool   isMatch{};
    if ((transformedIdentity.has_value()) &&
        Floor::planeIdentitiesMatch(*survivingIdentity,
                                    *transformedIdentity,
                                    Floor::kMergeMaxPlaneNormalAngle_deg,
                                    maximumOffset_m_in,
                                    normalAngle_deg,
                                    offset_m,
                                    isMatch) !=
            FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        // planeIdentitiesMatch cannot fail; continue as before.
    }
    const bool floorsMatch = transformedIdentity.has_value() && isMatch;
    std::cout << "[ConsecutiveMerge] Floor check: "
              << (floorsMatch ? "ACCEPTED" : "REJECTED")
              << " (angle=" << normalAngle_deg << " deg, offset=" << offset_m
              << " m; limits=" << Floor::kMergeMaxPlaneNormalAngle_deg
              << " deg/" << maximumOffset_m_in << " m)." << std::endl;
    decision_out = floorsMatch ? "ACCEPTED" : "REJECTED";
    return floorsMatch;
}

} // namespace semantic
} // namespace core
} // namespace vs_graphs
