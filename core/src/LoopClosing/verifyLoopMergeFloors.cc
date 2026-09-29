/*!
 * This file is a modified version of a file from ORB-SLAM3.
 *
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger
 * Voos
 *
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 *
 * This file is part of vS-Graphs, which is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "LoopClosing.h"

#include <limits>
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

/* Declared in LoopClosing.h: shared with SemanticVerify. */
bool verifyLoopMergeFloors(
    Map             *p_survivingMap_in,
    Map             *p_absorbedMap_in,
    const g2o::Sim3 &transform_absorbedWorldToSurvivingWorld_in,
    std::string     &result_out)
{
    semantic::Floor               *p_survivingFloor = nullptr;
    std::vector<semantic::Floor *> survivingMapAllFloors{};
    if (p_survivingMap_in->getAllFloors(survivingMapAllFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (semantic::Floor::selectBestObservedFloor(survivingMapAllFloors,
                                                 p_survivingFloor) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    semantic::Floor               *p_absorbedFloor = nullptr;
    std::vector<semantic::Floor *> absorbedMapAllFloors{};
    if (p_absorbedMap_in->getAllFloors(absorbedMapAllFloors) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllFloors returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    if (semantic::Floor::selectBestObservedFloor(absorbedMapAllFloors,
                                                 p_absorbedFloor) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: selectBestObservedFloor returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }

    std::optional<semantic::Floor::PlaneIdentity> survivingFloorPlaneIdentity{};
    if ((p_survivingFloor != nullptr) &&
        p_survivingFloor->getPlaneIdentity(survivingFloorPlaneIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::optional<semantic::Floor::PlaneIdentity> survivingIdentity =
        p_survivingFloor != nullptr ? survivingFloorPlaneIdentity
                                    : std::nullopt;
    std::optional<semantic::Floor::PlaneIdentity> absorbedFloorPlaneIdentity{};
    if ((p_absorbedFloor != nullptr) &&
        p_absorbedFloor->getPlaneIdentity(absorbedFloorPlaneIdentity) !=
            semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getPlaneIdentity returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    const std::optional<semantic::Floor::PlaneIdentity> absorbedIdentity =
        p_absorbedFloor != nullptr ? absorbedFloorPlaneIdentity : std::nullopt;

    if (!survivingIdentity.has_value() || !absorbedIdentity.has_value())
    {
        result_out = "DEFERRED";
        unsigned long survivingMapId{};
        if (p_survivingMap_in->getId(survivingMapId) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long absorbedMapId{};
        if (p_absorbedMap_in->getId(absorbedMapId) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cout << "[FloorVerify] Map#" << survivingMapId << " and Map#"
                  << absorbedMapId << " floor verification deferred (current="
                  << (survivingIdentity.has_value() ? "valid" : "missing")
                  << ", merge="
                  << (absorbedIdentity.has_value() ? "valid" : "missing")
                  << "); result=DEFERRED committed=0" << std::endl;
        return false;
    }

    std::optional<semantic::Floor::PlaneIdentity> transformedAbsorbedIdentity{};
    if (semantic::Floor::transformPlaneIdentity(
            *absorbedIdentity,
            transform_absorbedWorldToSurvivingWorld_in,
            transformedAbsorbedIdentity) !=
        semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: transformPlaneIdentity returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    double floorNormalAngle_deg = std::numeric_limits<double>::infinity();
    double floorOffset_m        = std::numeric_limits<double>::infinity();

    bool isMatch{};
    if ((transformedAbsorbedIdentity.has_value()) &&
        semantic::Floor::planeIdentitiesMatch(
            *survivingIdentity,
            *transformedAbsorbedIdentity,
            semantic::Floor::kMergeMaxPlaneNormalAngle_deg,
            semantic::Floor::kMergeMaxPlaneOffset_m,
            floorNormalAngle_deg,
            floorOffset_m,
            isMatch) != semantic::FloorStatus::FLOOR_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: planeIdentitiesMatch returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    const bool floorsMatch = transformedAbsorbedIdentity.has_value() && isMatch;

    if (!floorsMatch)
    {
        result_out = "REJECTED";
        unsigned long survivingMapId2{};
        if (p_survivingMap_in->getId(survivingMapId2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        unsigned long absorbedMapId2{};
        if (p_absorbedMap_in->getId(absorbedMapId2) !=
            MapStatus::MAP_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                         "%s: getId returned a failure status although it "
                         "cannot fail; continuing as before.",
                         __func__);
        }
        std::cerr << "[FloorVerify] Rejecting loop merge: Map#"
                  << survivingMapId2 << " and Map#" << absorbedMapId2
                  << " floor planes mismatch (angle=" << floorNormalAngle_deg
                  << " deg, offset=" << floorOffset_m << " m; limits="
                  << semantic::Floor::kMergeMaxPlaneNormalAngle_deg << " deg/"
                  << semantic::Floor::kMergeMaxPlaneOffset_m
                  << " m). result=REJECTED committed=0" << std::endl;
        return false;
    }

    unsigned long survivingMapId3{};
    if (p_survivingMap_in->getId(survivingMapId3) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    unsigned long absorbedMapId3{};
    if (p_absorbedMap_in->getId(absorbedMapId3) !=
        MapStatus::MAP_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getId returned a failure status although it cannot "
                     "fail; continuing as before.",
                     __func__);
    }
    std::cout << "[FloorVerify] Map#" << survivingMapId3 << " and Map#"
              << absorbedMapId3
              << " floor planes match (angle=" << floorNormalAngle_deg
              << " deg, offset=" << floorOffset_m
              << " m). result=ACCEPTED committed=0" << std::endl;
    result_out = "ACCEPTED";
    return true;
}

} // namespace core
} // namespace vs_graphs
