/*!
 * @file            checkConsecutiveFloors.cc
 *
 * @brief           Implements checkConsecutiveFloors(), declared in
 *                  Semantic/SemanticVerify/private_functions.h.
 */

#include "scene_understanding/semantic/semantic_verify/semantic_verify.h"
#include "map_state/map/objects/MapStatusEnum.h"
#include "scene_understanding/semantic/floor/floor.h"
#include "scene_understanding/semantic/floor/objects/FloorStatusEnum.h"

#include "mapping/geometric/Plane/Plane.h"
#include "place_recognition_and_loop_closure/loop_closing/loop_closing.h"
#include "map_state/map/map.h"

#include <vs_graphs_g2o/core/block_solver.h>
#include <vs_graphs_g2o/core/optimization_algorithm_levenberg.h>
#include <vs_graphs_g2o/core/robust_kernel_impl.h>
#include <vs_graphs_g2o/core/sparse_optimizer.h>
#include <vs_graphs_g2o/solvers/linear_solver_eigen.h>
#include <vs_graphs_g2o/types/sim3.h>
#include "configuration/types/types.h"
#include "scene_understanding/semantic/room/objects/RoomClass.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <rclcpp/logging.hpp>
#include <set>

#include "scene_understanding/semantic/semantic_verify/private_functions/private_functions.h"

namespace vs_graphs
{
namespace core
{
namespace semantic
{

SemanticVerifyStatus checkConsecutiveFloors(map::Map       *p_survivingMap_in,
                                            map::Map       *p_absorbedMap_in,
                                            const g2o::Sim3 &transform_in,
                                            double           maximumOffset_m_in,
                                            std::string     &decision_out,
                                            bool            &floorsMatch_out)
{
    Floor                         *p_survivingFloor = nullptr;
    std::vector<semantic::Floor *> survivingMapAllFloors{};
    if (p_survivingMap_in->getAllFloors(survivingMapAllFloors) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (Floor::selectBestObservedFloor(survivingMapAllFloors,
                                       p_survivingFloor) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    Floor                         *p_absorbedFloor = nullptr;
    std::vector<semantic::Floor *> absorbedMapAllFloors{};
    if (p_absorbedMap_in->getAllFloors(absorbedMapAllFloors) !=
        map::MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (Floor::selectBestObservedFloor(absorbedMapAllFloors, p_absorbedFloor) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    std::optional<Floor::PlaneIdentity> survivingFloorPlaneIdentity{};
    if ((p_survivingFloor != nullptr) &&
        p_survivingFloor->getPlaneIdentity(survivingFloorPlaneIdentity) !=
            FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::optional<Floor::PlaneIdentity> survivingIdentity =
        p_survivingFloor != nullptr ? survivingFloorPlaneIdentity
                                    : std::nullopt;
    std::optional<Floor::PlaneIdentity> absorbedFloorPlaneIdentity{};
    if ((p_absorbedFloor != nullptr) &&
        p_absorbedFloor->getPlaneIdentity(absorbedFloorPlaneIdentity) !=
            FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::optional<Floor::PlaneIdentity> absorbedIdentity =
        p_absorbedFloor != nullptr ? absorbedFloorPlaneIdentity : std::nullopt;
    if (!survivingIdentity.has_value() || !absorbedIdentity.has_value())
    {
        decision_out    = "DEFERRED";
        floorsMatch_out = false;
        return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
    }
    std::optional<Floor::PlaneIdentity> transformedIdentity{};
    if (Floor::transformPlaneIdentity(*absorbedIdentity,
                                      transform_in,
                                      transformedIdentity) !=
        FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: transformPlaneIdentity returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
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
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: planeIdentitiesMatch returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const bool floorsMatch = transformedIdentity.has_value() && isMatch;
    std::cout << "[ConsecutiveMerge] Floor check: "
              << (floorsMatch ? "ACCEPTED" : "REJECTED")
              << " (angle=" << normalAngle_deg << " deg, offset=" << offset_m
              << " m; limits=" << Floor::kMergeMaxPlaneNormalAngle_deg
              << " deg/" << maximumOffset_m_in << " m)." << std::endl;
    decision_out    = floorsMatch ? "ACCEPTED" : "REJECTED";
    floorsMatch_out = floorsMatch;
    return SemanticVerifyStatus::SEMANTIC_VERIFY_STATUS_SUCCESS;
}

} /* namespace semantic */
} /* namespace core */
} /* namespace vs_graphs */
