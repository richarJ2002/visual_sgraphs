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

/*!
 * @file            findPointsCloseToMarker.cc
 *
 * @brief           Implements Tracking::findPointsCloseToMarker(), declared in
 *                  Tracking.h.
 */

#include "Tracking.h"
#include <rclcpp/logging.hpp>

namespace vs_graphs
{
namespace core
{

// Semantic Entities
TrackingStatus Tracking::findPointsCloseToMarker(
    const semantic::Marker  *p_currentMarker_in,
    std::vector<MapPoint *> &pointsClose_out)
{
    // Get all map points
    std::vector<MapPoint *> allmapPoints{};
    if (p_atlas->getAllMapPoints(allmapPoints) !=
        AtlasStatus::ATLAS_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getAllMapPoints returned a failure status although "
                     "it cannot fail; continuing as before.",
                     __func__);
    }
    // Get all map points close to the marker
    Sophus::SE3f currentMarker_inGlobalPose{};
    if (p_currentMarker_in->getGlobalPose(currentMarker_inGlobalPose) !=
        semantic::MarkerStatus::MARKER_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: getGlobalPose returned a failure status although it "
                     "cannot fail; continuing as before.",
                     __func__);
    }
    std::vector<MapPoint *> closePoints{};
    if (findPointsCloseToLocation(allmapPoints,
                                  currentMarker_inGlobalPose.translation(),
                                  0.1,
                                  closePoints) !=
        TrackingStatus::TRACKING_STATUS_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("vs_graphs"),
                     "%s: findPointsCloseToLocation returned a failure status "
                     "although it cannot fail; continuing as before.",
                     __func__);
    }
    // Return the close points
    pointsClose_out = closePoints;
    return TrackingStatus::TRACKING_STATUS_SUCCESS;
}

} // namespace core
} // namespace vs_graphs
