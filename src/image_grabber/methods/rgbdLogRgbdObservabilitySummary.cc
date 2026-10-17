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
 * @file            rgbdLogRgbdObservabilitySummary.cc
 *
 * @brief           Logs a snapshot of RGB-D frontend observability.
 */

#include "common/common.h"
#include "image_grabber/objects/RgbdImageGrabberClass.h"
#include "observability/observability.h"

#include <rclcpp/logging.hpp>

namespace vs_graphs::image_grabber
{

void RgbdImageGrabber::logRgbdObservabilitySummary(
    const std::string &event_in) const
{
    const vs_graphs::observability::RgbdObservabilitySnapshot snapshot =
        rgbdObservability.snapshot();
    const std::string summary =
        vs_graphs::observability::formatRgbdObservabilitySummary(snapshot,
                                                                 event_in);
    RCLCPP_INFO(get_logger(), "%s", summary.c_str());
}

} /* namespace vs_graphs::image_grabber */
